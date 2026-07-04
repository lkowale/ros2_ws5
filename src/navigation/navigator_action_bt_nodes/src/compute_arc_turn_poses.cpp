// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#include <cmath>
#include <string>
#include <memory>
#include <vector>

#include "navigator_action_bt_nodes/compute_arc_turn_poses.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"
#include "nav2_behavior_tree/bt_utils.hpp"

namespace navigator_action_bt_nodes
{

static double quatToYaw(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

static double normAngle(double a)
{
  while (a >  M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

ComputeArcTurnPoses::ComputeArcTurnPoses(
  const std::string & name, const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
  auto node = conf.blackboard->get<rclcpp::Node::SharedPtr>("node");
  node_ = node;
  tf_buffer_ = conf.blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
}

BT::NodeStatus ComputeArcTurnPoses::tick()
{
  std::vector<geometry_msgs::msg::PoseStamped> map_points;
  if (!getInput("map_points", map_points) || map_points.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "ComputeArcTurnPoses: map_points empty");
    return BT::NodeStatus::FAILURE;
  }

  double min_turning_radius, swath_entry_length;
  std::string gps_frame_id;
  getInput("min_turning_radius", min_turning_radius);
  getInput("swath_entry_length", swath_entry_length);
  getInput("gps_frame_id", gps_frame_id);

  // Goal = next swath start
  const auto & goal_pose = map_points[0];
  const double gx = goal_pose.pose.position.x;
  const double gy = goal_pose.pose.position.y;
  const double yaw_g = quatToYaw(goal_pose.pose.orientation);
  const double cos_g = std::cos(yaw_g);
  const double sin_g = std::sin(yaw_g);

  // Start = current gps_link position
  double sx, sy, yaw_s;
  try {
    auto tf = tf_buffer_->lookupTransform(
      "map", gps_frame_id, rclcpp::Time(0), rclcpp::Duration::from_seconds(0.5));
    sx = tf.transform.translation.x;
    sy = tf.transform.translation.y;
    yaw_s = std::atan2(
      2.0 * (tf.transform.rotation.w * tf.transform.rotation.z +
             tf.transform.rotation.x * tf.transform.rotation.y),
      1.0 - 2.0 * (tf.transform.rotation.y * tf.transform.rotation.y +
                   tf.transform.rotation.z * tf.transform.rotation.z));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(node_->get_logger(), "ComputeArcTurnPoses: TF lookup failed: %s", ex.what());
    return BT::NodeStatus::FAILURE;
  }

  const double cos_s = std::cos(yaw_s);
  const double sin_s = std::sin(yaw_s);

  // Mirror the path-translation geometry from ArcTurnPlanner::createPlan
  // Lead-out far end
  const double lox = sx + swath_entry_length * cos_s;
  const double loy = sy + swath_entry_length * sin_s;

  // Lead-in near end (ideal)
  const double lix = gx - swath_entry_length * cos_g;
  const double liy = gy - swath_entry_length * sin_g;

  const double dx = lix - lox;
  const double dy = liy - loy;
  const double lateral = -dx * sin_s + dy * cos_s;
  const double row_spacing = std::abs(-dx * sin_g + dy * cos_g);

  double turn_sign;
  if (std::abs(lateral) > 0.1) {
    turn_sign = (lateral > 0) ? 1.0 : -1.0;
  } else {
    turn_sign = (cos_s * dy - sin_s * dx >= 0) ? 1.0 : -1.0;
  }

  double r = row_spacing / 2.0;
  if (r < min_turning_radius) r = min_turning_radius;

  // Arc1: from lox/loy tangent yaw_s — same as ArcTurnPlanner
  const double cx1     = lox + r * turn_sign * (-sin_s);
  const double cy1     = loy + r * turn_sign * ( cos_s);
  const double a1_start = std::atan2(loy - cy1, lox - cx1);
  const double sweep    = turn_sign * M_PI;
  const double a1_mid   = a1_start + sweep / 2.0;
  const double m1x = cx1 + r * std::cos(a1_mid);
  const double m1y = cy1 + r * std::sin(a1_mid);

  // Arc2: anchored to lix/liy tangent yaw_g
  const double cx2     = lix + r * turn_sign * (-sin_g);
  const double cy2     = liy + r * turn_sign * ( cos_g);
  const double a2_start = std::atan2(liy - cy2, lix - cx2);
  const double a2_mid   = a2_start - sweep / 2.0;
  const double m2x = cx2 + r * std::cos(a2_mid);
  const double m2y = cy2 + r * std::sin(a2_mid);

  // BackUp distance = straight distance between the two arc midpoints
  const double backup_dist = std::hypot(m2x - m1x, m2y - m1y);

  setOutput("turn_sign", turn_sign);
  setOutput("backup_dist", backup_dist);

  RCLCPP_INFO(node_->get_logger(),
    "ComputeArcTurnPoses: r=%.2f turn=%s m1=(%.2f,%.2f) m2=(%.2f,%.2f) backup_dist=%.2f",
    r, turn_sign > 0 ? "left" : "right", m1x, m1y, m2x, m2y, backup_dist);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace navigator_action_bt_nodes

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<navigator_action_bt_nodes::ComputeArcTurnPoses>("ComputeArcTurnPoses");
}
