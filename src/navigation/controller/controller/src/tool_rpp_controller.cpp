// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#include "tool_line_follower_controller/tool_rpp_controller.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace tool_rpp_controller
{

void ToolAnchoredRPPController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  tf_ = tf;
  auto node = parent.lock();
  logger_ = node->get_logger();

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".gps_frame_id", rclcpp::ParameterValue(std::string("gps_link")));
  node->get_parameter(name + ".gps_frame_id", gps_frame_id_);

  RegulatedPurePursuitController::configure(parent, name, tf, costmap_ros);

  RCLCPP_INFO(logger_,
    "ToolAnchoredRPPController: anchored to frame '%s'", gps_frame_id_.c_str());
}

geometry_msgs::msg::TwistStamped ToolAnchoredRPPController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & speed,
  nav2_core::GoalChecker * goal_checker)
{
  geometry_msgs::msg::PoseStamped anchor_pose = pose;
  try {
    geometry_msgs::msg::PoseStamped src;
    src.header.frame_id = gps_frame_id_;
    src.header.stamp = pose.header.stamp;
    src.pose.orientation.w = 1.0;
    tf_->transform(src, anchor_pose, pose.header.frame_id, tf2::durationFromSec(0.1));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 2000,
      "ToolAnchoredRPPController: could not get '%s' pose: %s — using base_footprint",
      gps_frame_id_.c_str(), ex.what());
  }

  return RegulatedPurePursuitController::computeVelocityCommands(anchor_pose, speed, goal_checker);
}

}  // namespace tool_rpp_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(tool_rpp_controller::ToolAnchoredRPPController, nav2_core::Controller)
