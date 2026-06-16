// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#include <string>
#include <memory>
#include <vector>
#include <cmath>
#include <fstream>

#include "one_line_nav/one_line_navigator.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <nlohmann/json.hpp>

namespace one_line_nav
{

bool OneLineNavigator::configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
  std::shared_ptr<nav2_util::OdomSmoother> /*odom_smoother*/)
{
  auto node = parent_node.lock();
  if (!node) {return false;}

  nav2_util::declare_parameter_if_not_declared(
    node, getName() + ".fields_directory",
    rclcpp::ParameterValue("/home/aa/ros2_ws5/src/fields"));
  node->get_parameter(getName() + ".fields_directory", fields_directory_);

  return true;
}

std::string OneLineNavigator::getDefaultBTFilepath(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node)
{
  auto node = parent_node.lock();
  if (!node) {return {};}

  if (!node->has_parameter("default_one_line_bt_xml")) {
    std::string pkg = ament_index_cpp::get_package_share_directory("one_line_nav");
    node->declare_parameter<std::string>(
      "default_one_line_bt_xml",
      pkg + "/behavior_trees/one_line.xml");
  }

  std::string path;
  node->get_parameter("default_one_line_bt_xml", path);
  return path;
}

bool OneLineNavigator::loadLineFromFile(
  const std::string & field_name,
  double & start_x, double & start_y, double & start_yaw,
  double & end_x, double & end_y, double & end_yaw)
{
  std::string file_path = fields_directory_ + "/" + field_name + "/line.json";
  std::ifstream file(file_path);
  if (!file.is_open()) {
    RCLCPP_ERROR(logger_, "Cannot open line file: %s", file_path.c_str());
    return false;
  }

  try {
    nlohmann::json j;
    file >> j;
    start_x   = j["start"]["x"].get<double>();
    start_y   = j["start"]["y"].get<double>();
    start_yaw = j["start"].value("yaw", 0.0);
    end_x     = j["end"]["x"].get<double>();
    end_y     = j["end"]["y"].get<double>();
    end_yaw   = j["end"].value("yaw", 0.0);
    RCLCPP_INFO(logger_, "Loaded line from %s", file_path.c_str());
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Failed to parse %s: %s", file_path.c_str(), e.what());
    return false;
  }
}

static geometry_msgs::msg::PoseStamped makePose(
  double x, double y, double yaw, const std::string & frame)
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = frame;
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.z = std::sin(yaw / 2.0);
  p.pose.orientation.w = std::cos(yaw / 2.0);
  return p;
}

bool OneLineNavigator::goalReceived(ActionT::Goal::ConstSharedPtr goal)
{
  auto bt_xml = goal->behavior_tree;
  if (bt_xml.empty()) {bt_xml = bt_action_server_->getCurrentBTFilename();}

  if (!bt_action_server_->loadBehaviorTree(bt_xml)) {
    RCLCPP_ERROR(logger_, "BT file not found: %s", bt_xml.c_str());
    return false;
  }

  double sx = goal->start_x, sy = goal->start_y, syaw = goal->start_yaw;
  double ex = goal->end_x,   ey = goal->end_y,   eyaw = goal->end_yaw;

  bool coords_provided = !(sx == 0.0 && sy == 0.0 && ex == 0.0 && ey == 0.0);

  if (!coords_provided) {
    if (goal->field_name.empty()) {
      RCLCPP_ERROR(logger_, "No coordinates and no field_name provided");
      return false;
    }
    if (!loadLineFromFile(goal->field_name, sx, sy, syaw, ex, ey, eyaw)) {
      return false;
    }
  }

  // Compute swath heading from start→end if yaw is not specified
  if (syaw == 0.0 && eyaw == 0.0) {
    double heading = std::atan2(ey - sy, ex - sx);
    syaw = heading;
    eyaw = heading;
  }

  auto blackboard = bt_action_server_->getBlackboard();

  std::vector<geometry_msgs::msg::PoseStamped> map_points = {
    makePose(sx, sy, syaw, "map"),
    makePose(ex, ey, eyaw, "map"),
  };
  blackboard->set<std::vector<geometry_msgs::msg::PoseStamped>>("map_points", map_points);
  blackboard->set<std::string>("field_name", goal->field_name);
  blackboard->set<std::string>("fields_directory", fields_directory_);

  RCLCPP_INFO(logger_,
    "OneLineNavigator: start=(%.2f, %.2f, %.2f°) end=(%.2f, %.2f, %.2f°)",
    sx, sy, syaw * 180.0 / M_PI,
    ex, ey, eyaw * 180.0 / M_PI);

  return true;
}

void OneLineNavigator::onLoop()
{
  auto feedback_msg = std::make_shared<ActionT::Feedback>();
  auto blackboard = bt_action_server_->getBlackboard();
  float dist_trav = 0.0f, total_dist = 0.0f;
  (void)blackboard->get("distance_traveled", dist_trav);
  (void)blackboard->get("total_distance", total_dist);
  feedback_msg->distance_traveled = dist_trav;
  feedback_msg->total_distance = total_dist;
  bt_action_server_->publishFeedback(feedback_msg);
}

void OneLineNavigator::onPreempt(ActionT::Goal::ConstSharedPtr goal)
{
  RCLCPP_INFO(logger_, "OneLineNavigator: goal preempted");

  double sx = goal->start_x, sy = goal->start_y, syaw = goal->start_yaw;
  double ex = goal->end_x,   ey = goal->end_y,   eyaw = goal->end_yaw;
  bool coords_provided = !(sx == 0.0 && sy == 0.0 && ex == 0.0 && ey == 0.0);

  if (!coords_provided && !goal->field_name.empty()) {
    loadLineFromFile(goal->field_name, sx, sy, syaw, ex, ey, eyaw);
  }

  if (syaw == 0.0 && eyaw == 0.0) {
    double heading = std::atan2(ey - sy, ex - sx);
    syaw = heading; eyaw = heading;
  }

  std::vector<geometry_msgs::msg::PoseStamped> map_points = {
    makePose(sx, sy, syaw, "map"),
    makePose(ex, ey, eyaw, "map"),
  };
  bt_action_server_->getBlackboard()->set<
    std::vector<geometry_msgs::msg::PoseStamped>>("map_points", map_points);
}

void OneLineNavigator::goalCompleted(
  typename ActionT::Result::SharedPtr result,
  const nav2_behavior_tree::BtStatus final_bt_status)
{
  if (final_bt_status == nav2_behavior_tree::BtStatus::SUCCEEDED) {
    result->error_code = ActionT::Result::NONE;
  } else if (final_bt_status == nav2_behavior_tree::BtStatus::CANCELED) {
    result->error_code = ActionT::Result::CANCELLED;
  } else {
    result->error_code = ActionT::Result::FAILED;
  }
  RCLCPP_INFO(logger_, "OneLineNavigator completed with status: %d",
    static_cast<int>(final_bt_status));
}

}  // namespace one_line_nav

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(one_line_nav::OneLineNavigator, nav2_core::NavigatorBase)
