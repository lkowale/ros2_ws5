// MPC-style path controller for Ackermann robots on RS paths.
//
// Reference frame: base_footprint = rear axle centre (Ackermann pivot).
//
// Algorithm each cycle:
//   1. Find closest path waypoint to rear axle (forward-only scan).
//   2. Compute adaptive lookahead L in [min_look, max_look], proportional
//      to distance from robot to current_idx_.
//   3. Simulate robot arc forward from current state at candidate steering
//      angles to find the cross point: where the predicted arc intersects
//      the path segment near the lookahead point.
//   4. Cross point closer than L  → reduce steering angle (arc too sharp).
//      Cross point farther than L or no intersection → increase steering angle.
//   5. Binary-search the steering angle that places the cross point at L.
//   6. Reverse segments: negate v, flip steering sign.

#include "tool_line_follower_controller/rs_path_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include "nav2_util/node_utils.hpp"
#include "std_msgs/msg/string.hpp"
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
  d("desired_linear_vel",  1.0);
  d("max_angular_vel",     1.0);
  d("min_lookahead_dist",  0.5);
  d("max_lookahead_dist",  2.0);
  d("wheelbase",           1.2);
  d("max_steering_angle",  0.7);
  d("sim_steps",          20);
  d("sim_step_len",        0.1);
  d("approach_dist",       3.0);
  d("min_approach_vel",    0.3);
  d("transform_tolerance", 0.1);

  node->get_parameter(name + ".desired_linear_vel",  desired_linear_vel_);
  node->get_parameter(name + ".max_angular_vel",     max_angular_vel_);
  node->get_parameter(name + ".min_lookahead_dist",  min_lookahead_dist_);
  node->get_parameter(name + ".max_lookahead_dist",  max_lookahead_dist_);
  node->get_parameter(name + ".wheelbase",           wheelbase_);
  node->get_parameter(name + ".max_steering_angle",  max_steering_angle_);
  node->get_parameter(name + ".sim_steps",           sim_steps_);
  node->get_parameter(name + ".sim_step_len",        sim_step_len_);
  node->get_parameter(name + ".approach_dist",       approach_dist_);
  node->get_parameter(name + ".min_approach_vel",    min_approach_vel_);
  node->get_parameter(name + ".transform_tolerance", transform_tolerance_);

  debug_pub_ = node->create_publisher<std_msgs::msg::String>("/rs_ctrl_debug", 10);

  RCLCPP_INFO(logger_,
    "RsPathController (MPC): v=%.2f max_w=%.2f L=[%.1f,%.1f]m wb=%.2f delta_max=%.2frad "
    "sim=%d×%.2fm",
    desired_linear_vel_, max_angular_vel_,
    min_lookahead_dist_, max_lookahead_dist_,
    wheelbase_, max_steering_angle_,
    sim_steps_, sim_step_len_);
}

void RsPathController::cleanup() {}
void RsPathController::activate() {}
void RsPathController::deactivate() {}

void RsPathController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  current_idx_ = 0;
  prev_rev_ = false;
  current_steering_angle_ = 0.0;
  if (!path.poses.empty()) {
    const auto & p0 = path.poses.front().pose.position;
    const auto & pN = path.poses.back().pose.position;
    RCLCPP_INFO(logger_,
      "setPlan: N=%zu  path[0]=(%.3f,%.3f)  path[N]=(%.3f,%.3f)",
      path.poses.size(), p0.x, p0.y, pN.x, pN.y);
  }
}

void RsPathController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  speed_limit_ = speed_limit;
  speed_limit_is_percentage_ = percentage;
}

// ─── Geometry helpers ─────────────────────────────────────────────────────────

size_t RsPathController::closestIndex(double px, double py) const
{
  const auto & poses = global_plan_.poses;
  double best_d2 = std::numeric_limits<double>::max();
  size_t best_i  = current_idx_;
  double arc     = 0.0;

  // Search within current segment only. This prevents RS paths (which curve
  // back on themselves) from jumping to a next-segment waypoint that is
  // geometrically closer but physically unreaached.
  // Exception: if current_idx_ is already at seg_end (single-waypoint segment
  // or robot has reached the boundary), open search to the full path so the
  // index can advance into the next segment.
  const size_t seg_end = segmentEndIndex(current_idx_);
  const size_t search_end = (current_idx_ >= seg_end)
    ? poses.size() - 1
    : seg_end;

  for (size_t i = current_idx_; i <= search_end; ++i) {
    const auto & p = poses[i].pose.position;
    double dx = px - p.x, dy = py - p.y;
    double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) { best_d2 = d2; best_i = i; }

    if (i + 1 < poses.size()) {
      arc += std::hypot(poses[i+1].pose.position.x - p.x,
                        poses[i+1].pose.position.y - p.y);
    }
    if (arc > max_lookahead_dist_ * 3.0) break;
  }
  return best_i;
}

size_t RsPathController::lookaheadIndex(size_t from_idx, double dist) const
{
  const auto & poses = global_plan_.poses;
  double arc = 0.0;
  for (size_t i = from_idx; i + 1 < poses.size(); ++i) {
    const auto & a = poses[i].pose.position;
    const auto & b = poses[i + 1].pose.position;
    arc += std::hypot(b.x - a.x, b.y - a.y);
    if (arc >= dist) return i + 1;
  }
  return poses.size() - 1;
}

// Last index of the current forward/reverse segment starting at from_idx.
// Stops before any waypoint whose direction flips relative to from_idx.
size_t RsPathController::segmentEndIndex(size_t from_idx) const
{
  const auto & poses = global_plan_.poses;
  const bool rev0 = isReverse(from_idx);
  for (size_t i = from_idx + 1; i < poses.size(); ++i) {
    if (isReverse(i) != rev0) return i - 1;
  }
  return poses.size() - 1;
}

double RsPathController::tangentYaw(size_t idx) const
{
  const auto & poses = global_plan_.poses;
  if (poses.size() < 2) return quatToYaw(poses[0].pose.orientation);
  size_t a = idx;
  size_t b = (idx + 1 < poses.size()) ? idx + 1 : idx;
  if (a == b && a > 0) a = a - 1;
  const auto & pa = poses[a].pose.position;
  const auto & pb = poses[b].pose.position;
  return std::atan2(pb.y - pa.y, pb.x - pa.x);
}

bool RsPathController::isReverse(size_t idx) const
{
  const double wp_yaw  = quatToYaw(global_plan_.poses[idx].pose.orientation);
  const double tan_yaw = tangentYaw(idx);
  return std::abs(wrap(wp_yaw - tan_yaw)) > M_PI / 2.0;
}

// Simulate the Ackermann bicycle model forward from (x,y,yaw) with constant
// steering angle delta for sim_steps_ steps of sim_step_len_ metres each.
// Returns the arc as a vector of (x,y) points.
std::vector<std::array<double,2>> RsPathController::simulateArc(
  double x, double y, double yaw, double delta, bool reverse) const
{
  std::vector<std::array<double,2>> arc;
  arc.reserve(sim_steps_ + 1);
  arc.push_back({x, y});

  const double ds = reverse ? -sim_step_len_ : sim_step_len_;
  for (int i = 0; i < sim_steps_; ++i) {
    x   += ds * std::cos(yaw);
    y   += ds * std::sin(yaw);
    yaw += (ds / wheelbase_) * std::tan(delta);
    arc.push_back({x, y});
  }
  return arc;
}

// Find the first intersection of the simulated arc with path segments
// [from_idx, to_idx). Returns distance from robot to intersection, or -1.
double RsPathController::arcPathIntersection(
  const std::vector<std::array<double,2>> & arc,
  double robot_x, double robot_y,
  size_t from_idx, size_t to_idx) const
{
  const auto & poses = global_plan_.poses;
  const size_t N = poses.size();
  to_idx = std::min(to_idx, N - 1);

  // For each arc segment, test against each path segment.
  // Return distance from robot to first intersection found.
  for (size_t ai = 0; ai + 1 < arc.size(); ++ai) {
    double ax1 = arc[ai][0],   ay1 = arc[ai][1];
    double ax2 = arc[ai+1][0], ay2 = arc[ai+1][1];

    for (size_t pi = from_idx; pi < to_idx; ++pi) {
      const auto & pa = poses[pi].pose.position;
      const auto & pb = poses[pi + 1].pose.position;
      double bx1 = pa.x, by1 = pa.y;
      double bx2 = pb.x, by2 = pb.y;

      // Segment–segment intersection (Cramer's rule).
      double dx1 = ax2 - ax1, dy1 = ay2 - ay1;
      double dx2 = bx2 - bx1, dy2 = by2 - by1;
      double denom = dx1 * dy2 - dy1 * dx2;
      if (std::abs(denom) < 1e-9) continue;  // parallel

      double t = ((bx1 - ax1) * dy2 - (by1 - ay1) * dx2) / denom;
      double u = ((bx1 - ax1) * dy1 - (by1 - ay1) * dx1) / denom;
      if (t < 0.0 || t > 1.0 || u < 0.0 || u > 1.0) continue;

      double ix = ax1 + t * dx1;
      double iy = ay1 + t * dy1;
      return std::hypot(ix - robot_x, iy - robot_y);
    }
  }
  return -1.0;  // no intersection
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

  const double rx   = pose.pose.position.x;
  const double ry   = pose.pose.position.y;
  const double ryaw = quatToYaw(pose.pose.orientation);

  // ── Closest waypoint ─────────────────────────────────────────────────────
  current_idx_ = closestIndex(rx, ry);
  const size_t N = global_plan_.poses.size();
  const bool rev  = isReverse(current_idx_);
  const double eff_yaw = rev ? wrap(ryaw + M_PI) : ryaw;

  // ── Adaptive lookahead distance ───────────────────────────────────────────
  // Grows with distance from robot to closest path point so the arc has
  // more room to converge when the robot is far off-path.
  const auto & cp = global_plan_.poses[current_idx_].pose.position;
  const double dist_to_path = std::hypot(rx - cp.x, ry - cp.y);
  const double lookahead = std::clamp(
    min_lookahead_dist_ + dist_to_path,
    min_lookahead_dist_,
    max_lookahead_dist_);

  // Never let the lookahead cross a forward/reverse segment boundary — doing so
  // puts the PP target on a segment driven in the opposite direction, causing a
  // huge angle_to_look and maximum steering command at every transition.
  const size_t seg_end = segmentEndIndex(current_idx_);

  // look_idx: path waypoint at lookahead arc distance, capped at segment end.
  const size_t look_idx = std::min(lookaheadIndex(current_idx_, lookahead), seg_end);

  // arc_end_idx: path waypoint covering the full simulated arc length,
  // so the intersection search spans the entire predicted trajectory.
  const double arc_len = sim_steps_ * sim_step_len_;
  const size_t arc_end_idx = std::min(lookaheadIndex(current_idx_, arc_len), seg_end);

  // ── Distance to end ───────────────────────────────────────────────────────
  const auto & last = global_plan_.poses.back().pose.position;
  const double dist_to_end = std::hypot(rx - last.x, ry - last.y);

  prev_rev_ = rev;

  // ── Speed ─────────────────────────────────────────────────────────────────
  double v_cmd = desired_linear_vel_;
  if (speed_limit_is_percentage_) {
    v_cmd = desired_linear_vel_ * speed_limit_;
  } else if (speed_limit_ < desired_linear_vel_) {
    v_cmd = speed_limit_;
  }
  if (dist_to_end < approach_dist_) {
    v_cmd = std::max(min_approach_vel_, v_cmd * (dist_to_end / approach_dist_));
  }
  if (rev) v_cmd = -v_cmd;

  // ── CTE and heading error ────────────────────────────────────────────────
  const double tan_yaw = tangentYaw(current_idx_);
  const double dx_to_cp = rx - cp.x, dy_to_cp = ry - cp.y;
  // CTE: signed lateral distance from path. Positive = robot left of path.
  const double cte = -std::sin(tan_yaw) * dx_to_cp + std::cos(tan_yaw) * dy_to_cp;
  const double heading_err = wrap(tan_yaw - eff_yaw);

  // ── Steering: pure-pursuit + CTE correction ──────────────────────────────
  // Pure-pursuit aims at look_idx. On straight segments this gives near-zero
  // delta even with significant CTE (the lookahead point is ahead but offset
  // perpendicular). Add an explicit CTE correction term so lateral error is
  // always driven out regardless of path curvature.
  //
  // delta_cte = -atan(cte / lookahead): steer back to path proportional to
  // how far off we are relative to lookahead distance.

  const auto & lp = global_plan_.poses[look_idx].pose.position;
  const double angle_to_look = wrap(std::atan2(lp.y - ry, lp.x - rx) - eff_yaw);
  const double chord = std::hypot(lp.x - rx, lp.y - ry);
  double delta_pp = (chord > 0.01)
    ? std::atan2(2.0 * wheelbase_ * std::sin(angle_to_look), chord)
    : 0.0;

  // CTE correction: proportional to lateral error, normalised by lookahead.
  const double delta_cte = -std::atan(cte / lookahead);
  delta_pp = std::clamp(delta_pp + delta_cte, -max_steering_angle_, max_steering_angle_);


  auto crossDist = [&](double delta) -> double {
    auto arc = simulateArc(rx, ry, eff_yaw, delta, rev);
    return arcPathIntersection(arc, rx, ry, current_idx_, arc_end_idx);
  };

  double best_delta = delta_pp;
  const double cd_pp = crossDist(delta_pp);

  if (cd_pp > 0.0 && cd_pp < lookahead) {
    // Arc overshoots: cross point closer than target → reduce |delta|.
    // Binary search between 0 and |delta_pp|, keeping the sign of delta_pp.
    const double sign = (delta_pp >= 0.0) ? 1.0 : -1.0;
    double dlo = 0.0, dhi = std::abs(delta_pp);
    for (int iter = 0; iter < 8; ++iter) {
      double dmid = 0.5 * (dlo + dhi);
      double cd   = crossDist(dmid * sign);
      if (cd > 0.0 && cd < lookahead) {
        dhi = dmid;   // still too curved → less steer
      } else {
        dlo = dmid;   // misses or lands past lookahead → more steer
      }
    }
    best_delta = 0.5 * (dlo + dhi) * sign;
  }
  // If cd_pp <= 0 (no intersection) or cd_pp >= lookahead: keep delta_pp.

  best_delta = std::clamp(best_delta, -max_steering_angle_, max_steering_angle_);
  current_steering_angle_ = best_delta;

  // Convert steering angle to yaw rate: w = v * tan(delta) / L
  const double w_cmd = std::clamp(
    std::abs(v_cmd) * std::tan(best_delta) / wheelbase_,
    -max_angular_vel_, max_angular_vel_);

  cmd.twist.linear.x  = v_cmd;
  cmd.twist.angular.z = w_cmd;

  // Debug: idx,n,rev,cte,heading_err_deg,lookahead,delta_deg,v_cmd,w_cmd,dist_to_end,dist_to_path
  {
    char buf[192];
    std::snprintf(buf, sizeof(buf),
      "%zu,%zu,%d,%.4f,%.2f,%.3f,%.2f,%.4f,%.4f,%.3f,%.3f",
      current_idx_, N, (int)rev,
      cte, heading_err * 180.0 / M_PI,
      lookahead, best_delta * 180.0 / M_PI,
      v_cmd, w_cmd, dist_to_end, dist_to_path);
    std_msgs::msg::String dbg;
    dbg.data = buf;
    debug_pub_->publish(dbg);
  }

  return cmd;
}

}  // namespace rs_path_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rs_path_controller::RsPathController, nav2_core::Controller)
