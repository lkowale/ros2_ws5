// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#include <string>
#include <fstream>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"
#include "geographic_msgs/msg/geo_point.hpp"
#include "field_nav/field_line.hpp"
#include <nlohmann/json.hpp>

using field_nav::FieldLine;

class LoadFieldProgress : public BT::SyncActionNode
{
public:
  LoadFieldProgress(
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
      BT::InputPort<std::vector<FieldLine>>("field_lines", "All field lines from navigator"),
      BT::OutputPort<int32_t>("current_line_index", "Current line index"),
      BT::OutputPort<std::string>("stage", "Current stage"),
      BT::OutputPort<std::vector<geographic_msgs::msg::GeoPoint>>("geo_points", "Geo points for current line"),
      BT::OutputPort<std::string>("turn_direction", "Turn direction for current line"),
      BT::OutputPort<bool>("has_progress", "True if progress was loaded"),
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

    std::vector<FieldLine> field_lines;
    if (!getInput("field_lines", field_lines)) {
      return BT::NodeStatus::FAILURE;
    }

    std::string file_path = fields_directory + "/" + field_name + "/field_progress.json";

    std::ifstream file(file_path);
    if (!file.is_open()) {
      // No progress file - start from beginning
      setOutput("has_progress", false);
      return BT::NodeStatus::FAILURE;
    }

    try {
      nlohmann::json j;
      file >> j;

      int32_t current_line_index = j["current_line_index"].get<int32_t>();
      std::string stage = j["stage"].get<std::string>();

      // Validate line index
      if (current_line_index < 0 ||
          static_cast<size_t>(current_line_index) >= field_lines.size()) {
        setOutput("has_progress", false);
        return BT::NodeStatus::FAILURE;
      }

      // Set outputs
      setOutput("current_line_index", current_line_index);
      setOutput("stage", stage);

      // Set geo_points for current line
      std::vector<geographic_msgs::msg::GeoPoint> geo_points = {
        field_lines[current_line_index].start,
        field_lines[current_line_index].end
      };
      setOutput("geo_points", geo_points);
      setOutput("turn_direction", field_lines[current_line_index].turn);
      setOutput("has_progress", true);

      RCLCPP_INFO(rclcpp::get_logger("LoadFieldProgress"),
        "Loaded progress: line %d, stage %s", current_line_index, stage.c_str());

      return BT::NodeStatus::SUCCESS;
    } catch (const std::exception & e) {
      RCLCPP_WARN(rclcpp::get_logger("LoadFieldProgress"),
        "Failed to parse progress file: %s", e.what());
      setOutput("has_progress", false);
      return BT::NodeStatus::FAILURE;
    }
  }
};

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<LoadFieldProgress>("LoadFieldProgress");
}
