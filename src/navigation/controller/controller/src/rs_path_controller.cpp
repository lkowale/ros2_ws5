// Stanley-style path controller for Ackermann robots on Reeds-Shepp paths.
//
// Algorithm:
//   1. Find closest waypoint to the robot's tool_link (rear axle) position.
//   2. Compute heading error: robot_yaw − path_tangent_yaw at that point.
//   3. Compute cross-track error: signed lateral distance from path.
//   4. steering = k_heading*heading_err + atan2(k_cross*cross_err, v)
//   5. Detect reverse segments (waypoint yaw flipped π from tangent) and
//      negate linear velocity + adjust error signs accordingly.

#include "tool_line_follower_controller/rs_path_controller.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include "nav2_util/node_utils.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

namespace rs_path_controller
{

static double wrap(double a)
{
  while (a >  M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

static double quatToYaw(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

// ─── Controller lifecycle ─────────────────────────────────────────────────────

void RsPathController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  tf_ = tf;
  plugin_name_ = name;
  costmap_ros_ = costmap_ros;
  auto node = parent.lock();
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  global_frame_ = costmap_ros->getGlobalFrameID();

  auto d = [&](const std::string & p, auto v) {
    nav2_util::declare_parameter_if_not_declared(node, name + "." + p, rclcpp::ParameterValue(v));
  };
  d("desired_linear_vel", 1.0);
  d("max_angular_vel", 1.0);
  d("k_heading",    1.5);
  d("k_cross",      1.0);
  d("k_cross_gain", 1.0);
  d("approach_dist", 3.0);
  d("min_approach_vel", 0.3);
  d("align_heading_thresh", 0.52);
  d("align_exit_thresh",    0.17);
  d("transform_tolerance", 0.1);

  node->get_parameter(name + ".desired_linear_vel",    desired_linear_vel_);
  node->get_parameter(name + ".max_angular_vel",       max_angular_vel_);
  node->get_parameter(name + ".k_heading",             k_heading_);
  node->get_parameter(name + ".k_cross",               k_cross_);
  node->get_parameter(name + ".k_cross_gain",          k_cross_gain_);
  node->get_parameter(name + ".approach_dist",         approach_dist_);
  node->get_parameter(name + ".min_approach_vel",      min_approach_vel_);
  node->get_parameter(name + ".align_heading_thresh",  align_heading_thresh_);
  node->get_parameter(name + ".align_exit_thresh",     align_exit_thresh_);
  node->get_parameter(name + ".transform_tolerance",   transform_tolerance_);

  debug_pub_ = node->create_publisher<std_msgs::msg::String>("/rs_ctrl_debug", 10);

  RCLCPP_INFO(logger_,
    "RsPathController: v=%.2f max_w=%.2f k_h=%.2f k_c=%.2f k_cg=%.2f approach=%.1fm",
    desired_linear_vel_, max_angular_vel_, k_heading_, k_cross_, k_cross_gain_, approach_dist_);
}

void RsPathController::cleanup() {}
void RsPathController::activate() {}
void RsPathController::deactivate() {}

void RsPathController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  current_idx_ = 0;
  aligning_ = false;  // let first cycle evaluate — don't pre-set to avoid deadband trap
}

void RsPathController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  speed_limit_ = speed_limit;
  speed_limit_is_percentage_ = percentage;
}

// ─── Geometry helpers ─────────────────────────────────────────────────────────

size_t RsPathController::closestIndex(double px, double py) const
{
  if (global_plan_.poses.empty()) return 0;

  // Search from current_idx_ forward — also allow stepping back a few points
  // in case of localization noise, but never regress more than 5 waypoints.
  const size_t search_start = current_idx_ > 5 ? current_idx_ - 5 : 0;
  double best_d2 = std::numeric_limits<double>::max();
  size_t best_i = current_idx_;

  for (size_t i = search_start; i < global_plan_.poses.size(); ++i) {
    const auto & p = global_plan_.poses[i].pose.position;
    double dx = px - p.x, dy = py - p.y;
    double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) { best_d2 = d2; best_i = i; }
    // Stop searching once distance starts growing past a threshold (past closest)
    if (d2 > best_d2 + 25.0) break;
  }
  return best_i;
}

double RsPathController::tangentYaw(size_t idx) const
{
  const auto & poses = global_plan_.poses;
  if (poses.size() < 2) return quatToYaw(poses[0].pose.orientation);

  // Use the segment direction idx→idx+1 (or idx-1→idx at the end).
  size_t a = idx;
  size_t b = (idx + 1 < poses.size()) ? idx + 1 : idx;
  if (a == b && a > 0) a = a - 1;

  const auto & pa = poses[a].pose.position;
  const auto & pb = poses[b].pose.position;
  return std::atan2(pb.y - pa.y, pb.x - pa.x);
}

double RsPathController::crossTrackError(double px, double py, size_t idx) const
{
  const auto & poses = global_plan_.poses;
  const double tan_yaw = tangentYaw(idx);
  const auto & pp = poses[idx].pose.position;
  // Cross-track = component of (robot - path_point) perpendicular to tangent.
  // Positive = robot is to the left of the path direction.
  const double dx = px - pp.x;
  const double dy = py - pp.y;
  return -std::sin(tan_yaw) * dx + std::cos(tan_yaw) * dy;
}

bool RsPathController::isReverse(size_t idx) const
{
  // RS path reverse segments have waypoint yaw flipped π relative to tangent.
  const double wp_yaw = quatToYaw(global_plan_.poses[idx].pose.orientation);
  const double tan_yaw = tangentYaw(idx);
  return std::abs(wrap(wp_yaw - tan_yaw)) > M_PI / 2.0;
}

// ─── Main control loop ────────────────────────────────────────────────────────

geometry_msgs::msg::TwistStamped RsPathController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header = pose.header;

  if (global_plan_.poses.empty()) return cmd;

  // ── Get tool_link (rear axle) position in global frame ────────────────────
  double rx = pose.pose.position.x;
  double ry = pose.pose.position.y;
  double ryaw = quatToYaw(pose.pose.orientation);

  // Try to use tool_link frame if available; fall back to pose (base_footprint)
  try {
    geometry_msgs::msg::PoseStamped tool_src, tool_out;
    tool_src.header.frame_id = "tool_link";
    tool_src.header.stamp = pose.header.stamp;
    tool_src.pose.orientation.w = 1.0;
    tf_->transform(tool_src, tool_out, global_frame_,
                   tf2::durationFromSec(transform_tolerance_));
    rx   = tool_out.pose.position.x;
    ry   = tool_out.pose.position.y;
    ryaw = quatToYaw(tool_out.pose.orientation);
  } catch (const tf2::TransformException &) {
    // fall back to base_footprint pose — still functional
  }

  // ── Find closest waypoint ──────────────────────────────────────────────────
  current_idx_ = closestIndex(rx, ry);
  const size_t N = global_plan_.poses.size();

  // ── Detect reverse segment ────────────────────────────────────────────────
  const bool rev = isReverse(current_idx_);

  // ── Heading error ─────────────────────────────────────────────────────────
  // For reverse: robot drives backward, effective heading is ryaw+π.
  const double eff_yaw = rev ? wrap(ryaw + M_PI) : ryaw;
  const double tan_yaw = tangentYaw(current_idx_);
  const double heading_err = wrap(tan_yaw - eff_yaw);

  // ── Cross-track error ─────────────────────────────────────────────────────
  double cte = crossTrackError(rx, ry, current_idx_);
  // For reverse: sign convention flips (we're tracking the same path backward)
  if (rev) cte = -cte;

  // ── Distance to end of path ───────────────────────────────────────────────
  const auto & last = global_plan_.poses.back().pose.position;
  const double dist_to_end = std::hypot(rx - last.x, ry - last.y);

  // ── Heading alignment pre-phase ───────────────────────────────────────────
  // If heading error is large, stop and turn in place until aligned.
  // Uses hysteresis: enter above align_heading_thresh_, exit below align_exit_thresh_.
  const double abs_herr = std::abs(heading_err);
  if (abs_herr > align_heading_thresh_) {
    aligning_ = true;
  } else if (abs_herr < align_exit_thresh_) {
    aligning_ = false;
  }

  const double stanley_w = std::copysign(max_angular_vel_, heading_err);

  if (aligning_) {
    // Spin in place: no forward motion, full angular toward path tangent.
    cmd.twist.linear.x  = 0.0;
    cmd.twist.angular.z = stanley_w;

    RCLCPP_DEBUG(logger_, "ALIGNING  h_err=%.1f°  w=%.2f",
      heading_err * 180.0 / M_PI, stanley_w);

    // Publish debug and return early — no Stanley computation needed.
    char buf[128];
    std::snprintf(buf, sizeof(buf),
      "%zu,%zu,%d,%.5f,%.5f,%.5f,%.4f,%.4f,%.3f",
      current_idx_, N, (int)rev,
      cte, heading_err, heading_err,
      0.0, stanley_w, dist_to_end);
    std_msgs::msg::String dbg;
    dbg.data = buf;
    debug_pub_->publish(dbg);
    return cmd;
  }

  // ── Speed ─────────────────────────────────────────────────────────────────
  double v_cmd = desired_linear_vel_;
  if (speed_limit_is_percentage_) {
    v_cmd = desired_linear_vel_ * speed_limit_;
  } else if (speed_limit_ < desired_linear_vel_) {
    v_cmd = speed_limit_;
  }

  // Slow down approaching goal
  if (dist_to_end < approach_dist_) {
    const double t = dist_to_end / approach_dist_;
    v_cmd = std::max(min_approach_vel_, v_cmd * t);
  }
  if (rev) v_cmd = -v_cmd;

  // ── Stanley steering ──────────────────────────────────────────────────────
  // Two independent terms with separate gains:
  //   w = k_heading * heading_err + k_cross_gain * atan2(k_cross * cte, v)
  const double v_denom = std::max(std::abs(v_cmd), 0.3);
  const double cte_term = std::atan2(k_cross_ * cte, v_denom);
  const double stanley  = heading_err + cte_term;
  double w_cmd = k_heading_ * heading_err + k_cross_gain_ * cte_term;
  w_cmd = std::clamp(w_cmd, -max_angular_vel_, max_angular_vel_);

  cmd.twist.linear.x  = v_cmd;
  cmd.twist.angular.z = w_cmd;

  // Publish debug CSV for rs_controller_logger.py
  // format: idx,n,rev,cte,heading_err_rad,stanley_rad,v_cmd,w_cmd,dist_to_end
  {
    char buf[128];
    std::snprintf(buf, sizeof(buf),
      "%zu,%zu,%d,%.5f,%.5f,%.5f,%.4f,%.4f,%.3f",
      current_idx_, N, (int)rev,
      cte, heading_err, stanley,
      v_cmd, w_cmd, dist_to_end);
    std_msgs::msg::String dbg;
    dbg.data = buf;
    debug_pub_->publish(dbg);
  }

  return cmd;
}

}  // namespace rs_path_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rs_path_controller::RsPathController, nav2_core::Controller)
