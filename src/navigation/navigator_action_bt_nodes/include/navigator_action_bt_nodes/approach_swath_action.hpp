// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#ifndef NAVIGATOR_ACTION_BT_NODES__APPROACH_SWATH_ACTION_HPP_
#define NAVIGATOR_ACTION_BT_NODES__APPROACH_SWATH_ACTION_HPP_

#include <string>
#include <memory>
#include <vector>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "solbot5_msgs/action/run_approach_swath.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace navigator_action_bt_nodes
{

class ApproachSwathAction : public nav2_behavior_tree::BtActionNode<solbot5_msgs::action::RunApproachSwath>
{
public:
  ApproachSwathAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;

  BT::NodeStatus on_success() override;

  BT::NodeStatus on_aborted() override;

  BT::NodeStatus on_cancelled() override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("map_points", "Map points (poses in map frame)"),
      BT::InputPort<std::string>("planner_id", "GridBased", "Planner plugin to use"),
      BT::InputPort<std::string>("controller_id", "FollowPath", "Controller plugin to use"),
      BT::InputPort<std::string>("path_pub_topic", "to_start_path", "Topic to publish path"),
      BT::InputPort<std::string>("behavior_tree", "", "BT file to use (empty for default)"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal_override", "Override goal pose instead of map_points[0]"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("start_override", "Override path start pose instead of TF lookup"),
      BT::OutputPort<uint16_t>("error_code", "Error code from action result"),
    });
  }
};

}  // namespace navigator_action_bt_nodes

#endif  // NAVIGATOR_ACTION_BT_NODES__APPROACH_SWATH_ACTION_HPP_
