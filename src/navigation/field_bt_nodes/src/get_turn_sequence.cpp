// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/condition_node.h"

class GetTurnSequenceFile : public BT::SyncActionNode
{
public:
  GetTurnSequenceFile(
    const std::string & name,
    const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("turn_direction", "Turn direction (left, right, end)"),
      BT::InputPort<std::string>("left_sequence", "turn_left_07.json", "Sequence file for left turn"),
      BT::InputPort<std::string>("right_sequence", "turn_right_07.json", "Sequence file for right turn"),
      BT::OutputPort<std::string>("sequence_file", "Selected sequence file"),
      BT::OutputPort<bool>("skip_turn", "True if no turn needed (end of field)"),
    };
  }

  BT::NodeStatus tick() override
  {
    std::string turn_direction;
    std::string left_sequence, right_sequence;

    if (!getInput("turn_direction", turn_direction)) {
      return BT::NodeStatus::FAILURE;
    }

    getInput("left_sequence", left_sequence);
    getInput("right_sequence", right_sequence);

    if (turn_direction == "end") {
      // Last line - no turn needed
      setOutput("skip_turn", true);
      setOutput("sequence_file", "");
      RCLCPP_INFO(rclcpp::get_logger("GetTurnSequenceFile"),
        "End of field - no turn needed");
      return BT::NodeStatus::SUCCESS;
    }

    setOutput("skip_turn", false);

    if (turn_direction == "left") {
      setOutput("sequence_file", left_sequence);
      RCLCPP_INFO(rclcpp::get_logger("GetTurnSequenceFile"),
        "Left turn - using %s", left_sequence.c_str());
    } else if (turn_direction == "right") {
      setOutput("sequence_file", right_sequence);
      RCLCPP_INFO(rclcpp::get_logger("GetTurnSequenceFile"),
        "Right turn - using %s", right_sequence.c_str());
    } else {
      RCLCPP_WARN(rclcpp::get_logger("GetTurnSequenceFile"),
        "Unknown turn direction: %s, defaulting to left", turn_direction.c_str());
      setOutput("sequence_file", left_sequence);
    }

    return BT::NodeStatus::SUCCESS;
  }
};

class CheckSkipTurn : public BT::ConditionNode
{
public:
  CheckSkipTurn(
    const std::string & name,
    const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("turn_direction", "Turn direction (left, right, end)"),
    };
  }

  BT::NodeStatus tick() override
  {
    std::string turn_direction;

    if (!getInput("turn_direction", turn_direction)) {
      return BT::NodeStatus::FAILURE;
    }

    // Return SUCCESS if we should skip the turn (end of field)
    if (turn_direction == "end") {
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
  }
};

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<GetTurnSequenceFile>("GetTurnSequenceFile");
  factory.registerNodeType<CheckSkipTurn>("CheckSkipTurn");
}
