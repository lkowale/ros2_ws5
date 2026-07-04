// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#include <string>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"

class ClearOneLineProgress : public BT::SyncActionNode
{
public:
  ClearOneLineProgress(
    const std::string & name,
    const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
    node_ = rclcpp::Node::make_shared("clear_one_line_progress_bt_node");
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("fields_directory", "/home/aa/ros2_ws4/src/fields",
        "Base directory for field files"),
      BT::InputPort<std::string>("field_name", "Field name for progress file location"),
    };
  }

  BT::NodeStatus tick() override
  {
    std::string fields_directory;
    std::string field_name;

    if (!getInput("fields_directory", fields_directory)) {
      fields_directory = "/home/aa/ros2_ws4/src/fields";
    }

    if (!getInput("field_name", field_name) || field_name.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "Missing required input [field_name]");
      return BT::NodeStatus::FAILURE;
    }

    // Build progress file path
    std::string progress_file = fields_directory + "/" + field_name + "/progress.json";

    // Delete the file if it exists
    if (std::filesystem::exists(progress_file)) {
      try {
        std::filesystem::remove(progress_file);
        RCLCPP_INFO(node_->get_logger(), "Cleared progress file: %s", progress_file.c_str());
      } catch (const std::exception & e) {
        RCLCPP_WARN(node_->get_logger(), "Failed to delete progress file %s: %s",
          progress_file.c_str(), e.what());
        // Don't fail - just warn
      }
    } else {
      RCLCPP_DEBUG(node_->get_logger(), "No progress file to clear at: %s", progress_file.c_str());
    }

    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
};

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<ClearOneLineProgress>("ClearOneLineProgress");
}
