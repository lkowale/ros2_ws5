// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#include <string>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"
#include <nlohmann/json.hpp>

class SaveFieldProgress : public BT::SyncActionNode
{
public:
  SaveFieldProgress(
    const std::string & name,
    const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("fields_directory", "Base directory for field files"),
      BT::InputPort<std::string>("field_name", "Name of the field"),
      BT::InputPort<int32_t>("current_line_index", "Current line index"),
      BT::InputPort<std::string>("stage", "Current stage"),
    };
  }

  BT::NodeStatus tick() override
  {
    std::string fields_directory, field_name, stage;
    int32_t current_line_index;

    if (!getInput("fields_directory", fields_directory) ||
        !getInput("field_name", field_name) ||
        !getInput("current_line_index", current_line_index) ||
        !getInput("stage", stage))
    {
      return BT::NodeStatus::FAILURE;
    }

    std::string file_path = fields_directory + "/" + field_name + "/field_progress.json";

    try {
      nlohmann::json j;
      j["current_line_index"] = current_line_index;
      j["stage"] = stage;

      std::ofstream file(file_path);
      if (!file.is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("SaveFieldProgress"),
          "Failed to open progress file for writing: %s", file_path.c_str());
        return BT::NodeStatus::FAILURE;
      }

      file << j.dump(2);

      RCLCPP_INFO(rclcpp::get_logger("SaveFieldProgress"),
        "Saved progress: line %d, stage %s", current_line_index, stage.c_str());

      return BT::NodeStatus::SUCCESS;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(rclcpp::get_logger("SaveFieldProgress"),
        "Failed to save progress: %s", e.what());
      return BT::NodeStatus::FAILURE;
    }
  }
};

class ClearFieldProgress : public BT::SyncActionNode
{
public:
  ClearFieldProgress(
    const std::string & name,
    const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("fields_directory", "Base directory for field files"),
      BT::InputPort<std::string>("field_name", "Name of the field"),
    };
  }

  BT::NodeStatus tick() override
  {
    std::string fields_directory, field_name;

    if (!getInput("fields_directory", fields_directory) ||
        !getInput("field_name", field_name))
    {
      return BT::NodeStatus::FAILURE;
    }

    std::string file_path = fields_directory + "/" + field_name + "/field_progress.json";

    if (std::remove(file_path.c_str()) == 0) {
      RCLCPP_INFO(rclcpp::get_logger("ClearFieldProgress"),
        "Cleared progress file: %s", file_path.c_str());
    }

    return BT::NodeStatus::SUCCESS;
  }
};

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<SaveFieldProgress>("SaveFieldProgress");
  factory.registerNodeType<ClearFieldProgress>("ClearFieldProgress");
}
