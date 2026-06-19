// Copyright (c) 2024, Licensed under the Apache License, Version 2.0

#include "rs_test_nav/rs_test_navigator.hpp"

#include <string>
#include <memory>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using geometry_msgs::msg::PoseStamped;
using nav_msgs::msg::Odometry;

namespace rs_test_nav
{

bool RsTestNavigator::configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
  std::shared_ptr<nav2_util::OdomSmoother> /*odom_smoother*/)
{
  auto node = parent_node.lock();
  if (!node) {return false;}

  odom_sub_ = node->create_subscription<Odometry>(
    "/odom", 10,
    [this](Odometry::SharedPtr msg) {
      latest_robot_pose_.header = msg->header;
      latest_robot_pose_.header.frame_id = "map";
      latest_robot_pose_.pose = msg->pose.pose;
      auto bb = bt_action_server_->getBlackboard();
      if (bb) {
        bb->set<PoseStamped>("rs_robot_pose", latest_robot_pose_);
      }
    });

  latest_robot_pose_.header.frame_id = "map";
  latest_robot_pose_.pose.orientation.w = 1.0;

  return true;
}

std::string RsTestNavigator::getDefaultBTFilepath(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node)
{
  auto node = parent_node.lock();
  if (!node) {return {};}

  std::string param_name = getName() + ".default_bt_xml";
  if (!node->has_parameter(param_name)) {
    std::string pkg = ament_index_cpp::get_package_share_directory("rs_test_nav");
    node->declare_parameter<std::string>(
      param_name,
      pkg + "/behavior_trees/rs_test.xml");
  }
  std::string path;
  node->get_parameter(param_name, path);
  return path;
}

bool RsTestNavigator::goalReceived(ActionT::Goal::ConstSharedPtr goal)
{
  auto bt_xml = goal->behavior_tree;
  if (bt_xml.empty()) {
    bt_xml = bt_action_server_->getCurrentBTFilename();
  }
  if (!bt_action_server_->loadBehaviorTree(bt_xml)) {
    RCLCPP_ERROR(logger_, "RsTestNavigator: BT file not found: %s", bt_xml.c_str());
    return false;
  }
  if (goal->goals.empty()) {
    RCLCPP_ERROR(logger_, "RsTestNavigator: goal list is empty");
    return false;
  }

  goals_  = goal->goals;
  labels_ = goal->labels;
  while (labels_.size() < goals_.size()) {
    labels_.push_back("goal_" + std::to_string(labels_.size()));
  }
  current_idx_ = 0;

  auto bb = bt_action_server_->getBlackboard();
  bb->set<std::vector<PoseStamped>>("rs_test_goals", goals_);
  bb->set<std::vector<std::string>>("rs_test_labels", labels_);
  bb->set<int>("rs_test_index", 0);
  bb->set<PoseStamped>("rs_robot_pose", latest_robot_pose_);

  RCLCPP_INFO(logger_, "RsTestNavigator: starting %zu-goal test suite", goals_.size());
  return true;
}

void RsTestNavigator::onLoop()
{
  auto bb = bt_action_server_->getBlackboard();
  int idx = 0;
  (void)bb->get<int>("rs_test_index", idx);
  size_t display = static_cast<size_t>(std::max(0, idx - 1));
  current_idx_ = display;

  auto fb = std::make_shared<ActionT::Feedback>();
  fb->current_goal_index = static_cast<uint32_t>(display);
  fb->current_label = display < labels_.size() ? labels_[display] : "";
  float dist = 0.0f;
  (void)bb->get<float>("distance_remaining", dist);
  fb->distance_remaining = dist;
  bt_action_server_->publishFeedback(fb);
}

void RsTestNavigator::onPreempt(ActionT::Goal::ConstSharedPtr /*goal*/)
{
  RCLCPP_WARN(logger_, "RsTestNavigator: preempt not supported — ignoring");
}

void RsTestNavigator::goalCompleted(
  typename ActionT::Result::SharedPtr result,
  const nav2_behavior_tree::BtStatus final_bt_status)
{
  if (final_bt_status == nav2_behavior_tree::BtStatus::SUCCEEDED) {
    result->error_code = ActionT::Result::NONE;
    RCLCPP_INFO(logger_, "RsTestNavigator: test suite SUCCEEDED");
  } else if (final_bt_status == nav2_behavior_tree::BtStatus::CANCELED) {
    result->error_code = ActionT::Result::CANCELLED;
    RCLCPP_WARN(logger_, "RsTestNavigator: test suite CANCELLED");
  } else {
    result->error_code = ActionT::Result::FAILED;
    RCLCPP_ERROR(logger_, "RsTestNavigator: FAILED at goal %zu '%s'",
      current_idx_ + 1,
      current_idx_ < labels_.size() ? labels_[current_idx_].c_str() : "?");
  }
}

}  // namespace rs_test_nav

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rs_test_nav::RsTestNavigator, nav2_core::NavigatorBase)
