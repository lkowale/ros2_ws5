#ifndef PLANNER__SWATH_TURN_PLANNER_HPP_
#define PLANNER__SWATH_TURN_PLANNER_HPP_

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace planner
{

// Swath headland turn planner.
//
// Plans a fixed 3-segment headland turn + lead-in from the end of one swath
// to the start of the next.  All geometry is computed analytically — no
// search, no costmap.
//
// Turn shape (swath bearing α, min_turning_radius ρ):
//   1. Forward left arc  90°  radius ρ   (exit swath, clear headland)
//   2. Reverse right arc 60°  radius ρ   (back into position)
//   3. Forward connecting arc (variable)  to reach lead-in start
//   4. Forward straight  lead_in_length  ending at swath start, bearing α
//
// The goal pose encodes the NEXT swath: position = swath start, yaw = swath bearing.
// The start pose is the robot's current pose at the end of the previous swath.
//
// Parameters (set under the plugin name in nav2_params.yaml):
//   min_turning_radius   [m]   default 1.5
//   interpolation_resolution [m] default 0.05
//   lead_in_length       [m]   default 0.5
class SwathTurnPlanner : public nav2_core::GlobalPlanner
{
public:
  SwathTurnPlanner() = default;
  ~SwathTurnPlanner() = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

private:
  nav2_util::LifecycleNode::SharedPtr node_;
  std::string global_frame_;
  std::string name_;
  double rho_;          // min turning radius [m]
  double step_;         // waypoint spacing [m]
  double lead_in_;      // lead-in straight length [m]

  struct Pose2D { double x, y, yaw; };

  // Append sampled arc waypoints (forward or reverse).
  void appendArc(
    std::vector<geometry_msgs::msg::PoseStamped> & poses,
    Pose2D start, double radius, double angle_rad,
    bool left, bool forward) const;

  // Append sampled straight waypoints (forward or reverse).
  void appendStraight(
    std::vector<geometry_msgs::msg::PoseStamped> & poses,
    Pose2D start, double length, bool forward) const;

  // Compute end pose after an arc.
  Pose2D arcEnd(Pose2D p, double radius, double angle_rad, bool left, bool forward) const;

  // Compute end pose after a straight.
  Pose2D straightEnd(Pose2D p, double length, bool forward) const;

  geometry_msgs::msg::PoseStamped toPoseStamped(Pose2D p, bool reverse) const;

  static double wrap(double a);
};

}  // namespace planner

#endif  // PLANNER__SWATH_TURN_PLANNER_HPP_
