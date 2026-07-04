// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#include <string>
#include <fstream>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/condition_node.h"
#include "geographic_msgs/msg/geo_point.hpp"
#include "field_nav/field_line.hpp"
#include <nlohmann/json.hpp>

using field_nav::FieldLine;

class NextFieldLine : public BT::SyncActionNode
{
public:
  NextFieldLine(
    const std::string & name,
    const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::vector<FieldLine>>("field_lines", "All field lines"),
      BT::InputPort<int32_t>("current_line_index", "Current line index"),
      BT::InputPort<int32_t>("total_lines", "Total number of lines"),
      BT::OutputPort<int32_t>("next_line_index", "Next line index"),
      BT::OutputPort<std::vector<geographic_msgs::msg::GeoPoint>>("geo_points", "Geo points for next line"),
      BT::OutputPort<std::string>("turn_direction", "Turn direction for next line"),
      BT::OutputPort<bool>("all_lines_completed", "True if all lines are completed"),
    };
  }

  BT::NodeStatus tick() override
  {
    std::vector<FieldLine> field_lines;
    int32_t current_line_index, total_lines;

    if (!getInput("field_lines", field_lines) ||
        !getInput("current_line_index", current_line_index) ||
        !getInput("total_lines", total_lines))
    {
      return BT::NodeStatus::FAILURE;
    }

    int32_t next_line_index = current_line_index + 1;

    if (next_line_index >= total_lines) {
      // All lines completed — set turn_direction="end" so CheckSkipTurn skips the turn
      setOutput("all_lines_completed", true);
      setOutput("next_line_index", next_line_index);
      setOutput("turn_direction", std::string("end"));
      RCLCPP_INFO(rclcpp::get_logger("NextFieldLine"),
        "All %d lines completed", total_lines);
      return BT::NodeStatus::SUCCESS;
    }

    // Move to next line
    setOutput("all_lines_completed", false);
    setOutput("next_line_index", next_line_index);

    // Set geo_points for next line
    std::vector<geographic_msgs::msg::GeoPoint> geo_points = {
      field_lines[next_line_index].start,
      field_lines[next_line_index].end
    };
    setOutput("geo_points", geo_points);
    setOutput("turn_direction", field_lines[next_line_index].turn);

    RCLCPP_INFO(rclcpp::get_logger("NextFieldLine"),
      "Moving to line %d/%d, turn: %s",
      next_line_index, total_lines, field_lines[next_line_index].turn.c_str());

    return BT::NodeStatus::SUCCESS;
  }
};

class CheckAllLinesCompleted : public BT::ConditionNode
{
public:
  CheckAllLinesCompleted(
    const std::string & name,
    const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int32_t>("current_line_index", "Current line index"),
      BT::InputPort<int32_t>("total_lines", "Total number of lines"),
    };
  }

  BT::NodeStatus tick() override
  {
    int32_t current_line_index, total_lines;

    if (!getInput("current_line_index", current_line_index) ||
        !getInput("total_lines", total_lines))
    {
      return BT::NodeStatus::FAILURE;
    }

    // Return SUCCESS if all lines are completed (triggers parent Fallback to stop)
    if (current_line_index >= total_lines) {
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
  }
};

class SetCurrentLine : public BT::SyncActionNode
{
public:
  SetCurrentLine(
    const std::string & name,
    const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::vector<FieldLine>>("field_lines", "All field lines"),
      BT::InputPort<int32_t>("line_index", "Line index to set"),
      BT::OutputPort<std::vector<geographic_msgs::msg::GeoPoint>>("geo_points", "Geo points for line"),
      BT::OutputPort<std::string>("turn_direction", "Turn direction for line"),
    };
  }

  BT::NodeStatus tick() override
  {
    std::vector<FieldLine> field_lines;
    int32_t line_index;

    if (!getInput("field_lines", field_lines) ||
        !getInput("line_index", line_index))
    {
      return BT::NodeStatus::FAILURE;
    }

    if (line_index < 0 || static_cast<size_t>(line_index) >= field_lines.size()) {
      return BT::NodeStatus::FAILURE;
    }

    std::vector<geographic_msgs::msg::GeoPoint> geo_points = {
      field_lines[line_index].start,
      field_lines[line_index].end
    };
    setOutput("geo_points", geo_points);
    setOutput("turn_direction", field_lines[line_index].turn);

    return BT::NodeStatus::SUCCESS;
  }
};

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<NextFieldLine>("NextFieldLine");
  factory.registerNodeType<CheckAllLinesCompleted>("CheckAllLinesCompleted");
  factory.registerNodeType<SetCurrentLine>("SetCurrentLine");
}
