// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/condition_node.h"

// CheckStageIs: Returns SUCCESS if current stage matches the expected stage
class CheckStageIs : public BT::ConditionNode
{
public:
  CheckStageIs(
    const std::string & name,
    const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("stage", "{stage}", "Current stage from blackboard"),
      BT::InputPort<std::string>("expected", "Expected stage value"),
    };
  }

  BT::NodeStatus tick() override
  {
    std::string current_stage;
    std::string expected_stage;

    if (!getInput("stage", current_stage)) {
      current_stage = "approach";  // Default
    }

    if (!getInput("expected", expected_stage)) {
      return BT::NodeStatus::FAILURE;
    }

    return (current_stage == expected_stage) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

// CheckStagePast: Returns SUCCESS if current stage is past (has completed) the expected stage
// Stage order: approach -> swath -> turn -> complete
class CheckStagePast : public BT::ConditionNode
{
public:
  CheckStagePast(
    const std::string & name,
    const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("stage", "{stage}", "Current stage from blackboard"),
      BT::InputPort<std::string>("check", "Stage to check if past"),
    };
  }

  BT::NodeStatus tick() override
  {
    std::string current_stage;
    std::string check_stage;

    if (!getInput("stage", current_stage)) {
      current_stage = "approach";
    }

    if (!getInput("check", check_stage)) {
      return BT::NodeStatus::FAILURE;
    }

    int current_order = stage_order(current_stage);
    int check_order = stage_order(check_stage);

    // Return SUCCESS if current stage is strictly past the check stage
    return (current_order > check_order) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  int stage_order(const std::string & stage)
  {
    if (stage == "approach") return 0;
    if (stage == "swath") return 1;
    if (stage == "turn") return 2;
    if (stage == "complete") return 3;
    return -1;  // Unknown
  }
};

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<CheckStageIs>("CheckStageIs");
  factory.registerNodeType<CheckStagePast>("CheckStagePast");
}
