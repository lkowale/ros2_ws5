// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#include <string>
#include <memory>
#include <vector>

#include "approach_swath_nav/approach_swath_navigator.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

namespace approach_swath_nav
{

bool ApproachSwathNavigator::configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
  std::shared_ptr<nav2_util::OdomSmoother> /*odom_smoother*/)
{
  auto node = parent_node.lock();
  if (!node) {
    return false;
  }

  nav2_util::declare_parameter_if_not_declared(
    node, getName() + ".default_planner_id",
    rclcpp::ParameterValue("GridBased"));
  nav2_util::declare_parameter_if_not_declared(
    node, getName() + ".default_controller_id",
    rclcpp::ParameterValue("FollowPath"));
  nav2_util::declare_parameter_if_not_declared(
    node, getName() + ".default_path_pub_topic",
    rclcpp::ParameterValue("to_start_path"));

  node->get_parameter(getName() + ".default_planner_id", default_planner_id_);
  node->get_parameter(getName() + ".default_controller_id", default_controller_id_);
  node->get_parameter(getName() + ".default_path_pub_topic", default_path_pub_topic_);

  return true;
}

std::string ApproachSwathNavigator::getDefaultBTFilepath(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node)
{
  std::string default_bt_xml_filename;
  auto node = parent_node.lock();

  if (!node) {
    return default_bt_xml_filename;
  }

  if (!node->has_parameter("default_approach_swath_bt_xml")) {
    std::string pkg_share_dir =
      ament_index_cpp::get_package_share_directory("approach_swath_nav");
    node->declare_parameter<std::string>(
      "default_approach_swath_bt_xml",
      pkg_share_dir + "/behavior_trees/approach_swath.xml");
  }

  node->get_parameter("default_approach_swath_bt_xml", default_bt_xml_filename);

  return default_bt_xml_filename;
}

bool ApproachSwathNavigator::goalReceived(ActionT::Goal::ConstSharedPtr goal)
{
  auto bt_xml_filename = goal->behavior_tree;

  if (bt_xml_filename.empty()) {
    bt_xml_filename = bt_action_server_->getCurrentBTFilename();
  }

  if (!bt_action_server_->loadBehaviorTree(bt_xml_filename)) {
    RCLCPP_ERROR(
      logger_, "BT file not found: %s. Navigation canceled.",
      bt_xml_filename.c_str());
    return false;
  }

  if (goal->map_points.empty()) {
    RCLCPP_ERROR(logger_, "No map_points provided");
    return false;
  }

  auto blackboard = bt_action_server_->getBlackboard();

  // Set map_points on blackboard (BT will extract goal_pose)
  blackboard->set<std::vector<geometry_msgs::msg::PoseStamped>>("map_points", goal->map_points);

  std::string planner_id = goal->planner_id.empty() ? default_planner_id_ : goal->planner_id;
  std::string controller_id = goal->controller_id.empty() ? default_controller_id_ : goal->controller_id;
  std::string path_pub_topic = goal->path_pub_topic.empty() ? default_path_pub_topic_ : goal->path_pub_topic;

  blackboard->set<std::string>("planner_id", planner_id);
  blackboard->set<std::string>("controller_id", controller_id);
  blackboard->set<std::string>("path_pub_topic", path_pub_topic);

  RCLCPP_INFO(logger_, "ApproachSwathNavigator received %zu map_points", goal->map_points.size());
  RCLCPP_INFO(logger_, "First point: [x: %.2f, y: %.2f]",
    goal->map_points[0].pose.position.x, goal->map_points[0].pose.position.y);
  RCLCPP_INFO(logger_, "Using planner_id: %s, controller_id: %s, path_pub_topic: %s",
    planner_id.c_str(), controller_id.c_str(), path_pub_topic.c_str());

  return true;
}

void ApproachSwathNavigator::onLoop()
{
  auto feedback_msg = std::make_shared<ActionT::Feedback>();
  auto blackboard = bt_action_server_->getBlackboard();

  float distance_remaining = 0.0f;
  (void)blackboard->get("distance_remaining", distance_remaining);

  feedback_msg->distance_remaining = distance_remaining;

  bt_action_server_->publishFeedback(feedback_msg);
}

void ApproachSwathNavigator::onPreempt(ActionT::Goal::ConstSharedPtr goal)
{
  RCLCPP_INFO(logger_, "Received goal preemption request");

  auto blackboard = bt_action_server_->getBlackboard();

  blackboard->set<std::vector<geometry_msgs::msg::PoseStamped>>("map_points", goal->map_points);

  std::string planner_id = goal->planner_id.empty() ? default_planner_id_ : goal->planner_id;
  std::string controller_id = goal->controller_id.empty() ? default_controller_id_ : goal->controller_id;
  std::string path_pub_topic = goal->path_pub_topic.empty() ? default_path_pub_topic_ : goal->path_pub_topic;

  blackboard->set<std::string>("planner_id", planner_id);
  blackboard->set<std::string>("controller_id", controller_id);
  blackboard->set<std::string>("path_pub_topic", path_pub_topic);
}

void ApproachSwathNavigator::goalCompleted(
  typename ActionT::Result::SharedPtr result,
  const nav2_behavior_tree::BtStatus final_bt_status)
{
  if (final_bt_status == nav2_behavior_tree::BtStatus::SUCCEEDED) {
    result->error_code = ActionT::Result::NONE;
  } else if (final_bt_status == nav2_behavior_tree::BtStatus::CANCELED) {
    result->error_code = ActionT::Result::CANCELLED;
  } else {
    result->error_code = ActionT::Result::FAILED;
  }

  RCLCPP_INFO(
    logger_, "ApproachSwathNavigator completed with status: %d",
    static_cast<int>(final_bt_status));
}

}  // namespace approach_swath_nav

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(approach_swath_nav::ApproachSwathNavigator, nav2_core::NavigatorBase)
