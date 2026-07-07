// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#include <string>
#include <memory>
#include <vector>

#include "navigator_action_bt_nodes/run_swath_action.hpp"

namespace navigator_action_bt_nodes
{

RunSwathAction::RunSwathAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<solbot5_msgs::action::RunSwath>(xml_tag_name, action_name, conf)
{
}

void RunSwathAction::on_tick()
{
  getInput("map_points", goal_.map_points);
  getInput("planner_id", goal_.planner_id);
  getInput("controller_id", goal_.controller_id);
  getInput("path_pub_topic", goal_.path_pub_topic);
  getInput("behavior_tree", goal_.behavior_tree);
}

BT::NodeStatus RunSwathAction::on_success()
{
  setOutput("error_code", result_.result->error_code);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus RunSwathAction::on_aborted()
{
  setOutput("error_code", result_.result->error_code);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus RunSwathAction::on_cancelled()
{
  setOutput("error_code", static_cast<uint16_t>(0));
  return BT::NodeStatus::SUCCESS;
}

void RunSwathAction::on_wait_for_result(
  std::shared_ptr<const solbot5_msgs::action::RunSwath::Feedback> feedback)
{
  if (feedback) {
    setOutput("distance_traveled", feedback->distance_traveled);
    setOutput("total_distance", feedback->total_distance);
    setOutput("current_waypoint", feedback->current_waypoint);
  }
}

}  // namespace navigator_action_bt_nodes

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<navigator_action_bt_nodes::RunSwathAction>(
        name, "run_swath", config);
    };

  factory.registerBuilder<navigator_action_bt_nodes::RunSwathAction>("RunSwathAction", builder);
}
