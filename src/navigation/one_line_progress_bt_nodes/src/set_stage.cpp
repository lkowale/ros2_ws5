// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"

class SetStage : public BT::SyncActionNode
{
public:
  SetStage(
    const std::string & name,
    const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("stage", "Stage value to set"),
      BT::OutputPort<std::string>("stage_out", "{stage}", "Blackboard key to write to"),
    };
  }

  BT::NodeStatus tick() override
  {
    std::string stage;
    if (!getInput("stage", stage)) {
      return BT::NodeStatus::FAILURE;
    }

    setOutput("stage_out", stage);
    return BT::NodeStatus::SUCCESS;
  }
};

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<SetStage>("SetStage");
}
