// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#include <string>
#include <memory>

#include "navigator_action_bt_nodes/approach_swath_action.hpp"

namespace navigator_action_bt_nodes
{

ApproachSwathAction::ApproachSwathAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<solbot5_msgs::action::RunApproachSwath>(xml_tag_name, action_name, conf)
{
}

void ApproachSwathAction::on_tick()
{
  getInput("map_points", goal_.map_points);
  getInput("planner_id", goal_.planner_id);
  getInput("controller_id", goal_.controller_id);
  getInput("path_pub_topic", goal_.path_pub_topic);
  getInput("behavior_tree", goal_.behavior_tree);
  geometry_msgs::msg::PoseStamped goal_override;
  if (getInput("goal_override", goal_override)) {
    goal_.goal_override = goal_override;
    goal_.use_goal_override = true;
  } else {
    goal_.use_goal_override = false;
  }
  geometry_msgs::msg::PoseStamped start_override;
  if (getInput("start_override", start_override)) {
    goal_.start_override = start_override;
    goal_.use_start_override = true;
  } else {
    goal_.use_start_override = false;
  }
}

BT::NodeStatus ApproachSwathAction::on_success()
{
  setOutput("error_code", result_.result->error_code);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ApproachSwathAction::on_aborted()
{
  setOutput("error_code", result_.result->error_code);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ApproachSwathAction::on_cancelled()
{
  setOutput("error_code", static_cast<uint16_t>(0));
  return BT::NodeStatus::SUCCESS;
}

}  // namespace navigator_action_bt_nodes

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<navigator_action_bt_nodes::ApproachSwathAction>(
        name, "approach_swath", config);
    };

  factory.registerBuilder<navigator_action_bt_nodes::ApproachSwathAction>("ApproachSwathAction", builder);
}
