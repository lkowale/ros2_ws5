// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"
#include "nav_msgs/msg/path.hpp"

class PublishPath : public BT::SyncActionNode
{
public:
  PublishPath(
    const std::string & name,
    const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
    node_ = rclcpp::Node::make_shared("publish_path_bt_node");
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic_name", "/path", "Topic to publish the path on"),
      BT::InputPort<nav_msgs::msg::Path>("path", "The path to publish")
    };
  }

  BT::NodeStatus tick() override
  {
    std::string topic_name;
    if (!getInput("topic_name", topic_name)) {
      topic_name = "/path";
    }

    nav_msgs::msg::Path path;
    if (!getInput("path", path)) {
      RCLCPP_ERROR(node_->get_logger(), "Missing input [path]");
      return BT::NodeStatus::FAILURE;
    }

    auto publisher = getOrCreatePublisher(topic_name);
    path.header.stamp = node_->now();
    publisher->publish(path);

    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr getOrCreatePublisher(
    const std::string & topic_name)
  {
    auto it = publishers_.find(topic_name);
    if (it != publishers_.end()) {
      return it->second;
    }
    auto qos = rclcpp::QoS(10).transient_local();
    auto pub = node_->create_publisher<nav_msgs::msg::Path>(topic_name, qos);
    publishers_[topic_name] = pub;
    return pub;
  }

  rclcpp::Node::SharedPtr node_;
  std::unordered_map<std::string, rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr> publishers_;
};

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<PublishPath>("PublishPath");
}
