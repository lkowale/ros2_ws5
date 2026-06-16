// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#include "tool_line_follower_controller/tool_arc_follower_controller.hpp"
#include "nav2_util/node_utils.hpp"
#include <cmath>

namespace tool_arc_follower_controller
{

void ToolArcFollowerController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  ToolLineFollowerController::configure(parent, name, tf, costmap_ros);

  auto node = parent.lock();
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".curvature_ff_gain", rclcpp::ParameterValue(1.0));
  node->get_parameter(name + ".curvature_ff_gain", curvature_ff_gain_);

  RCLCPP_INFO(node->get_logger(),
    "ToolArcFollowerController: curvature_ff_gain=%.2f", curvature_ff_gain_);
}

double ToolArcFollowerController::computePathCurvature(size_t idx) const
{
  const size_t n = global_plan_.poses.size();
  if (n < 2) {return 0.0;}

  size_t i0 = (idx > 0) ? idx - 1 : 0;
  size_t i1 = idx;
  size_t i2 = (idx + 1 < n) ? idx + 1 : n - 1;
  if (i0 == i1 || i1 == i2) {return 0.0;}

  const auto & p0 = global_plan_.poses[i0].pose.position;
  const auto & p1 = global_plan_.poses[i1].pose.position;
  const auto & p2 = global_plan_.poses[i2].pose.position;

  double ax = p1.x - p0.x, ay = p1.y - p0.y;
  double bx = p2.x - p1.x, by = p2.y - p1.y;
  double cross = ax * by - ay * bx;   // positive = left turn
  double la = std::hypot(ax, ay);
  double lb = std::hypot(bx, by);
  double lc = std::hypot(p2.x - p0.x, p2.y - p0.y);
  if (la < 1e-6 || lb < 1e-6 || lc < 1e-6) {return 0.0;}

  return 2.0 * cross / (la * lb * lc);
}

geometry_msgs::msg::TwistStamped ToolArcFollowerController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  auto cmd = ToolLineFollowerController::computeVelocityCommands(pose, velocity, goal_checker);

  double kappa = computePathCurvature(current_path_idx_);
  double ff = cmd.twist.linear.x * kappa * curvature_ff_gain_;
  cmd.twist.angular.z = std::clamp(
    cmd.twist.angular.z + ff, -max_angular_vel_, max_angular_vel_);

  return cmd;
}

}  // namespace tool_arc_follower_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  tool_arc_follower_controller::ToolArcFollowerController,
  nav2_core::Controller)
