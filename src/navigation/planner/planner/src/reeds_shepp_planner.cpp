// Reeds-Shepp planner plugin for Nav2.
//
// Uses OMPL's ReedsSheppStateSpace to find the shortest valid RS path (all 48
// word families checked, correct turning-radius constraint enforced).
// The path is sampled at `step_` metre intervals into PoseStamped waypoints.
// On reverse segments the yaw is rotated by π so that RPP drives backward.

#include "planner/reeds_shepp_planner.hpp"

#include <cmath>
#include <string>
#include <vector>

#include "nav2_util/node_utils.hpp"
#include "ompl/base/spaces/ReedsSheppStateSpace.h"
#include "ompl/base/spaces/SE2StateSpace.h"

namespace planner
{

// ─── helpers ─────────────────────────────────────────────────────────────────

static double wrap(double a)
{
  while (a >  M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

static geometry_msgs::msg::Quaternion yawToQuat(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0; q.y = 0.0;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
  return q;
}

static double quatToYaw(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

// ─── Pose propagation ────────────────────────────────────────────────────────
// Advance pose by one arc/straight step of ds metres (positive = forward).
// Uses the same geometry as OMPL's interpolate(): L-turn center left of heading,
// R-turn center right of heading.
static void stepPose(
  ompl::base::ReedsSheppStateSpace::ReedsSheppPathSegmentType type,
  double ds,    // metres, signed (positive forward, negative backward)
  double rho,
  double & cx, double & cy, double & cyaw)
{
  using T = ompl::base::ReedsSheppStateSpace::ReedsSheppPathSegmentType;
  switch (type) {
    case T::RS_STRAIGHT:
      cx   += ds * std::cos(cyaw);
      cy   += ds * std::sin(cyaw);
      break;
    case T::RS_LEFT: {
      const double dphi = ds / rho;
      cx   += rho * (std::sin(cyaw + dphi) - std::sin(cyaw));
      cy   += rho * (-std::cos(cyaw + dphi) + std::cos(cyaw));
      cyaw  = wrap(cyaw + dphi);
      break;
    }
    case T::RS_RIGHT: {
      const double dphi = ds / rho;
      cx   += rho * (-std::sin(cyaw - dphi) + std::sin(cyaw));
      cy   += rho * ( std::cos(cyaw - dphi) - std::cos(cyaw));
      cyaw  = wrap(cyaw - dphi);
      break;
    }
    default: break;
  }
}

// ─── Plugin implementation ────────────────────────────────────────────────────

void ReedsSheppPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> /*tf*/,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  global_frame_ = costmap_ros->getGlobalFrameID();

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".min_turning_radius",
    rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".interpolation_resolution",
    rclcpp::ParameterValue(0.05));

  node_->get_parameter(name_ + ".min_turning_radius", rho_);
  node_->get_parameter(name_ + ".interpolation_resolution", step_);

  auto qos = rclcpp::QoS(1).transient_local();
  fwd_pub_ = node_->create_publisher<nav_msgs::msg::Path>("/plan_forward", qos);
  rev_pub_ = node_->create_publisher<nav_msgs::msg::Path>("/plan_reverse", qos);

  RCLCPP_INFO(node_->get_logger(),
    "ReedsSheppPlanner configured (OMPL backend): rho=%.2f m  step=%.3f m", rho_, step_);
}

void ReedsSheppPlanner::cleanup() {}
void ReedsSheppPlanner::activate() {}
void ReedsSheppPlanner::deactivate() {}

nav_msgs::msg::Path ReedsSheppPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  nav_msgs::msg::Path path;
  path.header.stamp = node_->now();
  path.header.frame_id = global_frame_;

  if (start.header.frame_id != global_frame_ ||
      goal.header.frame_id  != global_frame_)
  {
    RCLCPP_ERROR(node_->get_logger(),
      "ReedsSheppPlanner: start/goal must be in frame '%s'", global_frame_.c_str());
    return path;
  }

  const double sx   = start.pose.position.x;
  const double sy   = start.pose.position.y;
  const double syaw = quatToYaw(start.pose.orientation);
  const double gx   = goal.pose.position.x;
  const double gy   = goal.pose.position.y;
  const double gyaw = quatToYaw(goal.pose.orientation);

  if (std::hypot(gx - sx, gy - sy) < 1e-4 && std::abs(wrap(gyaw - syaw)) < 1e-3) {
    return path;
  }

  // ── Use OMPL to find the shortest Reeds-Shepp path ────────────────────────
  ompl::base::ReedsSheppStateSpace rs(rho_);
  auto * s_from = rs.allocState()->as<ompl::base::SE2StateSpace::StateType>();
  auto * s_to   = rs.allocState()->as<ompl::base::SE2StateSpace::StateType>();
  s_from->setX(sx); s_from->setY(sy); s_from->setYaw(syaw);
  s_to->setX(gx);   s_to->setY(gy);   s_to->setYaw(gyaw);

  const auto rs_path = rs.reedsShepp(s_from, s_to);
  rs.freeState(s_from);
  rs.freeState(s_to);

  using T = ompl::base::ReedsSheppStateSpace::ReedsSheppPathSegmentType;

  // ── Sample the path ────────────────────────────────────────────────────────
  nav_msgs::msg::Path fwd_path, rev_path;
  fwd_path.header = rev_path.header = path.header;

  double cx = sx, cy2 = sy, cyaw = syaw;

  std::string seg_str;
  for (int i = 0; i < 5; ++i) {
    const T   type = rs_path.type_[i];
    const double seg_len = rs_path.length_[i];  // signed, in normalised units (×rho = metres)

    if (type == T::RS_NOP || std::abs(seg_len) < 1e-9) continue;
    if (cancel_checker && cancel_checker()) return path;

    const double seg_m = seg_len * rho_;  // signed metres
    const bool   rev   = (seg_m < 0.0);
    const double dist  = std::abs(seg_m);

    // Build segment description for logging
    seg_str += (rev ? '-' : '+');
    switch (type) {
      case T::RS_LEFT:     seg_str += 'L'; break;
      case T::RS_RIGHT:    seg_str += 'R'; break;
      case T::RS_STRAIGHT: seg_str += 'S'; break;
      default: break;
    }
    seg_str += '(' + std::to_string(static_cast<int>(std::round(dist * 10.0) / 10.0)) + "dm) ";

    const std::size_t before = path.poses.size();
    double travelled = 0.0;
    while (travelled + step_ < dist - 1e-9) {
      stepPose(type, rev ? -step_ : step_, rho_, cx, cy2, cyaw);
      travelled += step_;

      geometry_msgs::msg::PoseStamped p;
      p.header = path.header;
      p.pose.position.x = cx; p.pose.position.y = cy2; p.pose.position.z = 0.0;
      p.pose.orientation = yawToQuat(rev ? wrap(cyaw + M_PI) : cyaw);
      path.poses.push_back(p);
    }
    // Final sub-step: advance exactly to segment end
    const double remaining = dist - travelled;
    if (remaining > 1e-9) {
      stepPose(type, rev ? -remaining : remaining, rho_, cx, cy2, cyaw);

      geometry_msgs::msg::PoseStamped p;
      p.header = path.header;
      p.pose.position.x = cx; p.pose.position.y = cy2; p.pose.position.z = 0.0;
      p.pose.orientation = yawToQuat(rev ? wrap(cyaw + M_PI) : cyaw);
      path.poses.push_back(p);
    }

    // Split forward/reverse for Mapviz visualisation
    for (std::size_t k = before; k < path.poses.size(); ++k) {
      if (rev) rev_path.poses.push_back(path.poses[k]);
      else     fwd_path.poses.push_back(path.poses[k]);
    }
  }

  // Verify end-point accuracy and close any sub-step gap
  const double end_gap = std::hypot(cx - gx, cy2 - gy);
  const double yaw_err = std::abs(wrap(cyaw - gyaw)) * 180.0 / M_PI;
  RCLCPP_INFO(node_->get_logger(),
    "RS: (%.2f,%.2f,%.1f°)→(%.2f,%.2f,%.1f°) %s pts=%zu gap=%.4fm yaw_err=%.2f°",
    sx, sy, syaw * 180.0 / M_PI,
    gx, gy, gyaw * 180.0 / M_PI,
    seg_str.c_str(), path.poses.size(), end_gap, yaw_err);

  if (!path.poses.empty() && end_gap > 1e-3) {
    RCLCPP_WARN(node_->get_logger(),
      "RS path end gap %.4fm > 1mm — appending exact goal point", end_gap);
    geometry_msgs::msg::PoseStamped gp;
    gp.header = path.header;
    gp.pose.position.x = gx; gp.pose.position.y = gy; gp.pose.position.z = 0.0;
    // Use actual last segment direction for reverse flag
    bool last_rev = false;
    for (int i = 4; i >= 0; --i) {
      if (rs_path.type_[i] != T::RS_NOP && std::abs(rs_path.length_[i]) > 1e-9) {
        last_rev = rs_path.length_[i] < 0.0;
        break;
      }
    }
    gp.pose.orientation = last_rev
      ? yawToQuat(wrap(gyaw + M_PI))
      : goal.pose.orientation;
    path.poses.push_back(gp);
    if (last_rev) rev_path.poses.push_back(gp);
    else          fwd_path.poses.push_back(gp);
  }

  fwd_pub_->publish(fwd_path);
  rev_pub_->publish(rev_path);

  return path;
}

}  // namespace planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(planner::ReedsSheppPlanner, nav2_core::GlobalPlanner)
