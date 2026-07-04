// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#include <string>
#include <memory>
#include <vector>
#include <fstream>
#include <algorithm>

#include "geojson_line_nav/geojson_line_navigator.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <nlohmann/json.hpp>

namespace geojson_line_nav
{

bool GeoJsonLineNavigator::configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
  std::shared_ptr<nav2_util::OdomSmoother> /*odom_smoother*/)
{
  auto node = parent_node.lock();
  if (!node) {
    return false;
  }

  return true;
}

std::string GeoJsonLineNavigator::getDefaultBTFilepath(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node)
{
  std::string default_bt_xml_filename;
  auto node = parent_node.lock();

  if (!node) {
    return default_bt_xml_filename;
  }

  if (!node->has_parameter("default_geojson_line_bt_xml")) {
    std::string pkg_share_dir =
      ament_index_cpp::get_package_share_directory("geojson_line_nav");
    node->declare_parameter<std::string>(
      "default_geojson_line_bt_xml",
      pkg_share_dir + "/behavior_trees/geojson_line.xml");
  }

  node->get_parameter("default_geojson_line_bt_xml", default_bt_xml_filename);

  return default_bt_xml_filename;
}

bool GeoJsonLineNavigator::loadGeoJsonLine(
  const std::string & file_path,
  std::vector<geographic_msgs::msg::GeoPoint> & geo_points)
{
  std::ifstream file(file_path);
  if (!file.is_open()) {
    RCLCPP_ERROR(logger_, "Failed to open GeoJSON file: %s", file_path.c_str());
    return false;
  }

  try {
    nlohmann::json j;
    file >> j;

    // Extract LineString coordinates from the first feature
    const auto & features = j["features"];
    if (features.empty()) {
      RCLCPP_ERROR(logger_, "GeoJSON file has no features: %s", file_path.c_str());
      return false;
    }

    const auto & geometry = features[0]["geometry"];
    if (geometry["type"] != "LineString") {
      RCLCPP_ERROR(logger_, "GeoJSON geometry is not a LineString: %s", file_path.c_str());
      return false;
    }

    const auto & coordinates = geometry["coordinates"];
    geo_points.clear();
    for (const auto & coord : coordinates) {
      geographic_msgs::msg::GeoPoint point;
      point.longitude = coord[0].get<double>();
      point.latitude = coord[1].get<double>();
      point.altitude = 0.0;
      geo_points.push_back(point);
    }

    RCLCPP_INFO(logger_, "Loaded %zu points from GeoJSON: %s",
      geo_points.size(), file_path.c_str());
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Failed to parse GeoJSON file %s: %s",
      file_path.c_str(), e.what());
    return false;
  }
}

bool GeoJsonLineNavigator::goalReceived(ActionT::Goal::ConstSharedPtr goal)
{
  auto bt_xml_filename = goal->behavior_tree;

  if (bt_xml_filename.empty()) {
    bt_xml_filename = bt_action_server_->getCurrentBTFilename();
  }

  if (!bt_action_server_->loadBehaviorTree(bt_xml_filename)) {
    RCLCPP_ERROR(
      logger_, "BT file not found: %s. Navigation canceled.",
      bt_xml_filename.c_str());
    return false;
  }

  if (goal->geojson_file.empty()) {
    RCLCPP_ERROR(logger_, "No geojson_file specified");
    return false;
  }

  std::vector<geographic_msgs::msg::GeoPoint> geo_points;
  if (!loadGeoJsonLine(goal->geojson_file, geo_points)) {
    RCLCPP_ERROR(logger_, "Failed to load GeoJSON line from: %s",
      goal->geojson_file.c_str());
    return false;
  }

  if (geo_points.size() < 2) {
    RCLCPP_ERROR(logger_, "GeoJSON LineString must have at least 2 points");
    return false;
  }

  // Reverse points if direction is backward
  if (!goal->forward) {
    std::reverse(geo_points.begin(), geo_points.end());
    RCLCPP_INFO(logger_, "Direction: backward (reversed %zu points)", geo_points.size());
  } else {
    RCLCPP_INFO(logger_, "Direction: forward (%zu points)", geo_points.size());
  }

  // Set blackboard variables for BT nodes
  auto blackboard = bt_action_server_->getBlackboard();
  blackboard->set<std::vector<geographic_msgs::msg::GeoPoint>>("geo_points", geo_points);

  RCLCPP_INFO(logger_, "GeoJsonLineNavigator received goal: %s", goal->geojson_file.c_str());
  RCLCPP_INFO(logger_, "First point: [lat: %lf, lon: %lf]",
    geo_points.front().latitude, geo_points.front().longitude);
  RCLCPP_INFO(logger_, "Last point: [lat: %lf, lon: %lf]",
    geo_points.back().latitude, geo_points.back().longitude);

  return true;
}

void GeoJsonLineNavigator::onLoop()
{
  auto feedback_msg = std::make_shared<ActionT::Feedback>();
  auto blackboard = bt_action_server_->getBlackboard();

  float distance_traveled = 0.0f;
  float total_distance = 0.0f;

  (void)blackboard->get("distance_traveled", distance_traveled);
  (void)blackboard->get("total_distance", total_distance);

  feedback_msg->distance_traveled = distance_traveled;
  feedback_msg->total_distance = total_distance;

  bt_action_server_->publishFeedback(feedback_msg);
}

void GeoJsonLineNavigator::onPreempt(ActionT::Goal::ConstSharedPtr goal)
{
  RCLCPP_INFO(logger_, "Received goal preemption request");

  if (!goal->geojson_file.empty()) {
    std::vector<geographic_msgs::msg::GeoPoint> geo_points;
    if (loadGeoJsonLine(goal->geojson_file, geo_points)) {
      if (!goal->forward) {
        std::reverse(geo_points.begin(), geo_points.end());
      }
      auto blackboard = bt_action_server_->getBlackboard();
      blackboard->set<std::vector<geographic_msgs::msg::GeoPoint>>("geo_points", geo_points);
    }
  }
}

void GeoJsonLineNavigator::goalCompleted(
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

  RCLCPP_INFO(
    logger_, "GeoJsonLineNavigator completed with status: %d",
    static_cast<int>(final_bt_status));
}

}  // namespace geojson_line_nav

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(geojson_line_nav::GeoJsonLineNavigator, nav2_core::NavigatorBase)
