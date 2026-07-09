// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#include <string>
#include <memory>

#include "pause_bt_nodes/pause_condition.hpp"

namespace pause_bt_nodes
{

PauseCondition::PauseCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  // Get the node from the blackboard
  auto blackboard = config().blackboard;
  if (!blackboard) {
    throw std::runtime_error("PauseCondition: Blackboard is null!");
  }

  if (!blackboard->get<rclcpp::Node::SharedPtr>("node", node_)) {
    throw std::runtime_error("PauseCondition: 'node' not found on blackboard!");
  }

  if (!node_) {
    throw std::runtime_error("PauseCondition: Node pointer is null!");
  }

  RCLCPP_INFO(node_->get_logger(), "PauseCondition: Initializing...");

  std::string pause_topic;
  getInput("pause_topic", pause_topic);

  // Use default QoS (reliable, volatile) for compatibility with ros2 topic pub
  auto qos = rclcpp::QoS(10);
  pause_sub_ = node_->create_subscription<solbot5_msgs::msg::Pause>(
    pause_topic,
    qos,
    std::bind(&PauseCondition::pauseCallback, this, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(), "PauseCondition: Subscribed to %s", pause_topic.c_str());
}

void PauseCondition::pauseCallback(const solbot5_msgs::msg::Pause::SharedPtr msg)
{
  bool was_paused = is_paused_;
  is_paused_ = msg->paused;
  pause_source_ = msg->source;
  pause_reason_ = msg->reason;

  // Only log when state changes
  if (is_paused_ && !was_paused) {
    RCLCPP_WARN(node_->get_logger(), "PAUSED by %s: %s",
      pause_source_.c_str(), pause_reason_.c_str());
  } else if (!is_paused_ && was_paused) {
    RCLCPP_INFO(node_->get_logger(), "RESUMED by %s: %s",
      pause_source_.c_str(), pause_reason_.c_str());
  }
}

BT::NodeStatus PauseCondition::tick()
{
  // Process pending callbacks to update pause state
  rclcpp::spin_some(node_);

  if (is_paused_) {
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

WaitUntilNotPaused::WaitUntilNotPaused(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(name, conf)
{
  auto blackboard = config().blackboard;
  if (!blackboard) {
    throw std::runtime_error("WaitUntilNotPaused: Blackboard is null!");
  }

  if (!blackboard->get<rclcpp::Node::SharedPtr>("node", node_)) {
    throw std::runtime_error("WaitUntilNotPaused: 'node' not found on blackboard!");
  }

  if (!node_) {
    throw std::runtime_error("WaitUntilNotPaused: Node pointer is null!");
  }

  std::string pause_topic;
  getInput("pause_topic", pause_topic);

  auto qos = rclcpp::QoS(10);
  pause_sub_ = node_->create_subscription<solbot5_msgs::msg::Pause>(
    pause_topic,
    qos,
    std::bind(&WaitUntilNotPaused::pauseCallback, this, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(), "WaitUntilNotPaused: Subscribed to %s", pause_topic.c_str());
}

void WaitUntilNotPaused::pauseCallback(const solbot5_msgs::msg::Pause::SharedPtr msg)
{
  is_paused_ = msg->paused;
}

BT::NodeStatus WaitUntilNotPaused::onStart()
{
  rclcpp::spin_some(node_);
  if (is_paused_) {
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "Waiting for unpause...");
    return BT::NodeStatus::RUNNING;
  }
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus WaitUntilNotPaused::onRunning()
{
  rclcpp::spin_some(node_);
  if (is_paused_) {
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "Still paused, waiting...");
    return BT::NodeStatus::RUNNING;
  }
  RCLCPP_INFO(node_->get_logger(), "Unpaused, resuming execution");
  return BT::NodeStatus::SUCCESS;
}

void WaitUntilNotPaused::onHalted()
{
  RCLCPP_DEBUG(node_->get_logger(), "WaitUntilNotPaused halted");
}

}  // namespace pause_bt_nodes

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pause_bt_nodes::PauseCondition>("NotPaused");
  factory.registerNodeType<pause_bt_nodes::WaitUntilNotPaused>("WaitUntilNotPaused");
}
