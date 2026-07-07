// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#ifndef NAVIGATOR_ACTION_BT_NODES__COMPUTE_ARC_TURN_POSES_HPP_
#define NAVIGATOR_ACTION_BT_NODES__COMPUTE_ARC_TURN_POSES_HPP_

#include <string>
#include <memory>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "rclcpp/rclcpp.hpp"

namespace navigator_action_bt_nodes
{

// Computes backup distance for the 3-point arc turn using path-translation geometry.
//
// Given the current robot heading (from gps_link TF) and the next swath
// start/heading (map_points[0]), mirrors the ArcTurnPlanner geometry to compute:
//   turn_sign    — +1=left, -1=right
//   backup_dist  — reverse distance between first and second arc segments
//
// ArcTurn1 and ArcTurn2 planners independently compute the same geometry when
// called with the robot's current pose as start and map_points[0] as goal.
class ComputeArcTurnPoses : public BT::SyncActionNode
{
public:
  ComputeArcTurnPoses(const std::string & name, const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>(
        "map_points", "Next swath poses (map frame)"),
      BT::InputPort<double>("min_turning_radius", 3.5, "Minimum arc radius (m)"),
      BT::InputPort<double>("swath_entry_length", 1.0, "Lead-out / lead-in length (m)"),
      BT::InputPort<std::string>("gps_frame_id", "gps_link", "GPS antenna TF frame"),
      BT::OutputPort<double>("turn_sign", "Turn direction: +1=left, -1=right"),
      BT::OutputPort<double>("backup_dist", "Reverse distance between arc segments (m)"),
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

}  // namespace navigator_action_bt_nodes

#endif  // NAVIGATOR_ACTION_BT_NODES__COMPUTE_ARC_TURN_POSES_HPP_
