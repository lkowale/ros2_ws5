// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#ifndef PAUSE_BT_NODES__PAUSE_CONDITION_HPP_
#define PAUSE_BT_NODES__PAUSE_CONDITION_HPP_

#include <string>
#include <memory>
#include <atomic>

#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "solbot5_msgs/msg/pause.hpp"

namespace pause_bt_nodes
{

/**
 * @brief A BT condition node that monitors a pause topic.
 * Returns SUCCESS when not paused (execution should continue).
 * Returns FAILURE when paused (execution should stop).
 */
class PauseCondition : public BT::ConditionNode
{
public:
  PauseCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  ~PauseCondition() override = default;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("pause_topic", "/pause", "Topic to subscribe for pause commands"),
    };
  }

private:
  void pauseCallback(const solbot5_msgs::msg::Pause::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<solbot5_msgs::msg::Pause>::SharedPtr pause_sub_;
  std::atomic<bool> is_paused_{false};
  std::string pause_source_;
  std::string pause_reason_;
};

/**
 * @brief A BT action node that blocks (returns RUNNING) while paused.
 * Returns RUNNING when paused, SUCCESS when not paused.
 * Used with a Fallback after NotPaused so the tree waits instead of failing.
 */
class WaitUntilNotPaused : public BT::StatefulActionNode
{
public:
  WaitUntilNotPaused(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  ~WaitUntilNotPaused() override = default;

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("pause_topic", "/pause", "Topic to subscribe for pause commands"),
    };
  }

private:
  void pauseCallback(const solbot5_msgs::msg::Pause::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<solbot5_msgs::msg::Pause>::SharedPtr pause_sub_;
  std::atomic<bool> is_paused_{false};
};

}  // namespace pause_bt_nodes

#endif  // PAUSE_BT_NODES__PAUSE_CONDITION_HPP_
