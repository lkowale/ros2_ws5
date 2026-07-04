// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#ifndef NAVIGATOR_ACTION_BT_NODES__RUN_SWATH_ACTION_HPP_
#define NAVIGATOR_ACTION_BT_NODES__RUN_SWATH_ACTION_HPP_

#include <string>
#include <memory>
#include <vector>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "solbot5_msgs/action/run_swath.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace navigator_action_bt_nodes
{

class RunSwathAction : public nav2_behavior_tree::BtActionNode<solbot5_msgs::action::RunSwath>
{
public:
  RunSwathAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;

  BT::NodeStatus on_success() override;

  BT::NodeStatus on_aborted() override;

  BT::NodeStatus on_cancelled() override;

  void on_wait_for_result(
    std::shared_ptr<const solbot5_msgs::action::RunSwath::Feedback> feedback) override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("map_points", "Map points (poses in map frame)"),
      BT::InputPort<std::string>("planner_id", "StraightLine", "Planner plugin to use"),
      BT::InputPort<std::string>("controller_id", "FollowPath", "Controller plugin to use"),
      BT::InputPort<std::string>("path_pub_topic", "swath_path", "Topic to publish path"),
      BT::InputPort<std::string>("behavior_tree", "", "BT file to use (empty for default)"),
      BT::OutputPort<uint16_t>("error_code", "Error code from action result"),
      BT::OutputPort<float>("distance_traveled", "Distance traveled feedback"),
      BT::OutputPort<float>("total_distance", "Total distance feedback"),
      BT::OutputPort<uint32_t>("current_waypoint", "Current waypoint index"),
    });
  }
};

}  // namespace navigator_action_bt_nodes

#endif  // NAVIGATOR_ACTION_BT_NODES__RUN_SWATH_ACTION_HPP_
