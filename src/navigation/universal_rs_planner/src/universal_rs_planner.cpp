// Universal Reeds-Shepp planner — shortest RS path, no turn constraints.
//
// This is the baseline RS planner before any one_line_nav tuning.
// Use this for field_nav approach and headland turns where the shortest
// valid RS path is always correct.

#include "universal_rs_planner/universal_rs_planner.hpp"

#include <cmath>
#include <string>

#include "nav2_util/node_utils.hpp"
#include "ompl/base/spaces/ReedsSheppStateSpace.h"
#include "ompl/base/spaces/SE2StateSpace.h"

namespace universal_rs_planner
{

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

static void stepPose(
  ompl::base::ReedsSheppStateSpace::ReedsSheppPathSegmentType type,
  double ds, double rho,
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

void UniversalReedsSheppPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> /*tf*/,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  global_frame_ = costmap_ros->getGlobalFrameID();

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".min_turning_radius", rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.05));

  node_->get_parameter(name_ + ".min_turning_radius", rho_);
  node_->get_parameter(name_ + ".interpolation_resolution", step_);

  auto qos = rclcpp::QoS(1).transient_local();
  fwd_pub_ = node_->create_publisher<nav_msgs::msg::Path>("/plan_forward", qos);
  rev_pub_ = node_->create_publisher<nav_msgs::msg::Path>("/plan_reverse", qos);

  RCLCPP_INFO(node_->get_logger(),
    "UniversalReedsSheppPlanner configured: rho=%.2f m  step=%.3f m", rho_, step_);
}

void UniversalReedsSheppPlanner::cleanup() {}
void UniversalReedsSheppPlanner::activate() {}
void UniversalReedsSheppPlanner::deactivate() {}

nav_msgs::msg::Path UniversalReedsSheppPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  nav_msgs::msg::Path path;
  path.header.stamp = node_->now();
  path.header.frame_id = global_frame_;

  if (start.header.frame_id != global_frame_ ||
      goal.header.frame_id  != global_frame_) {
    RCLCPP_ERROR(node_->get_logger(),
      "UniversalReedsSheppPlanner: start/goal must be in frame '%s'", global_frame_.c_str());
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

  ompl::base::ReedsSheppStateSpace rs(rho_);
  auto * s_from = rs.allocState()->as<ompl::base::SE2StateSpace::StateType>();
  auto * s_to   = rs.allocState()->as<ompl::base::SE2StateSpace::StateType>();
  s_from->setX(sx); s_from->setY(sy); s_from->setYaw(syaw);
  s_to->setX(gx);   s_to->setY(gy);   s_to->setYaw(gyaw);

  const auto rs_path = rs.reedsShepp(s_from, s_to);
  rs.freeState(s_from);
  rs.freeState(s_to);

  using T = ompl::base::ReedsSheppStateSpace::ReedsSheppPathSegmentType;

  nav_msgs::msg::Path fwd_path, rev_path;
  fwd_path.header = rev_path.header = path.header;

  double cx = sx, cy = sy, cyaw = syaw;
  std::string seg_str;

  for (int i = 0; i < 5; ++i) {
    const T      type    = rs_path.type_[i];
    const double seg_len = rs_path.length_[i];

    if (type == T::RS_NOP || std::abs(seg_len) < 1e-9) continue;
    if (cancel_checker && cancel_checker()) return path;

    const double seg_m = seg_len * rho_;
    const bool   rev   = (seg_m < 0.0);
    const double dist  = std::abs(seg_m);


    seg_str += (rev ? '-' : '+');
    switch (type) {
      case T::RS_LEFT:     seg_str += 'L'; break;
      case T::RS_RIGHT:    seg_str += 'R'; break;
      case T::RS_STRAIGHT: seg_str += 'S'; break;
      default: break;
    }
    char sbuf[16];
    std::snprintf(sbuf, sizeof(sbuf), "(%.2fm) ", dist);
    seg_str += sbuf;

    const std::size_t before = path.poses.size();
    double travelled = 0.0;
    while (travelled + step_ < dist - 1e-9) {
      stepPose(type, rev ? -step_ : step_, rho_, cx, cy, cyaw);
      travelled += step_;

      geometry_msgs::msg::PoseStamped p;
      p.header = path.header;
      p.pose.position.x = cx; p.pose.position.y = cy; p.pose.position.z = 0.0;
      p.pose.orientation = yawToQuat(cyaw);
      path.poses.push_back(p);
    }
    const double remaining = dist - travelled;
    if (remaining > 1e-9) {
      stepPose(type, rev ? -remaining : remaining, rho_, cx, cy, cyaw);

      geometry_msgs::msg::PoseStamped p;
      p.header = path.header;
      p.pose.position.x = cx; p.pose.position.y = cy; p.pose.position.z = 0.0;
      p.pose.orientation = yawToQuat(cyaw);
      path.poses.push_back(p);
    }

    for (std::size_t k = before; k < path.poses.size(); ++k) {
      if (rev) rev_path.poses.push_back(path.poses[k]);
      else     fwd_path.poses.push_back(path.poses[k]);
    }
  }

  const double end_gap = std::hypot(cx - gx, cy - gy);
  const double yaw_err = std::abs(wrap(cyaw - gyaw)) * 180.0 / M_PI;
  RCLCPP_INFO(node_->get_logger(),
    "RS: (%.2f,%.2f,%.1f°)→(%.2f,%.2f,%.1f°) %s pts=%zu gap=%.4fm yaw_err=%.2f°",
    sx, sy, syaw * 180.0 / M_PI,
    gx, gy, gyaw * 180.0 / M_PI,
    seg_str.c_str(), path.poses.size(), end_gap, yaw_err);

  if (!path.poses.empty() && end_gap > 1e-3) {
    RCLCPP_WARN(node_->get_logger(), "RS end gap %.4fm — appending exact goal", end_gap);
    geometry_msgs::msg::PoseStamped gp;
    gp.header = path.header;
    gp.pose.position.x = gx; gp.pose.position.y = gy; gp.pose.position.z = 0.0;
    bool last_rev = false;
    for (int i = 4; i >= 0; --i) {
      if (rs_path.type_[i] != T::RS_NOP && std::abs(rs_path.length_[i]) > 1e-9) {
        last_rev = rs_path.length_[i] < 0.0;
        break;
      }
    }
    gp.pose.orientation = goal.pose.orientation;
    path.poses.push_back(gp);
    if (last_rev) rev_path.poses.push_back(gp);
    else          fwd_path.poses.push_back(gp);
  }

  fwd_pub_->publish(fwd_path);
  rev_pub_->publish(rev_path);

  return path;
}

}  // namespace universal_rs_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  universal_rs_planner::UniversalReedsSheppPlanner, nav2_core::GlobalPlanner)
