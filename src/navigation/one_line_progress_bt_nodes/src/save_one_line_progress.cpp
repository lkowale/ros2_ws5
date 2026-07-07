// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#include <string>
#include <fstream>
#include <filesystem>
#include <vector>
#include <chrono>
#include <iomanip>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"
#include "geographic_msgs/msg/geo_point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <nlohmann/json.hpp>

class SaveOneLineProgress : public BT::SyncActionNode
{
public:
  SaveOneLineProgress(
    const std::string & name,
    const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
    node_ = rclcpp::Node::make_shared("save_one_line_progress_bt_node");
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("fields_directory", "/home/aa/ros2_ws4/src/fields",
        "Base directory for field files"),
      BT::InputPort<std::string>("field_name", "Field name for progress file location"),
      BT::InputPort<std::vector<geographic_msgs::msg::GeoPoint>>("geo_points",
        "Current geo_points to save"),
      BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("map_points",
        "Current map_points to save"),
      BT::InputPort<std::string>("stage", "approach", "Current stage: approach, swath, turn"),
      BT::InputPort<int>("pass_number", 0, "Current pass number"),
    };
  }

  BT::NodeStatus tick() override
  {
    std::string fields_directory;
    std::string field_name;
    std::vector<geographic_msgs::msg::GeoPoint> geo_points;
    std::vector<geometry_msgs::msg::PoseStamped> map_points;
    std::string stage;
    int pass_number = 0;

    if (!getInput("fields_directory", fields_directory)) {
      fields_directory = "/home/aa/ros2_ws4/src/fields";
    }

    if (!getInput("field_name", field_name) || field_name.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "Missing required input [field_name]");
      return BT::NodeStatus::FAILURE;
    }

    if (!getInput("geo_points", geo_points)) {
      RCLCPP_WARN(node_->get_logger(), "Missing input [geo_points]");
    }

    if (!getInput("map_points", map_points)) {
      RCLCPP_WARN(node_->get_logger(), "Missing input [map_points]");
    }

    getInput("stage", stage);
    getInput("pass_number", pass_number);

    // Build progress file path
    std::string progress_file = fields_directory + "/" + field_name + "/progress.json";

    // Ensure directory exists
    std::filesystem::path dir_path = std::filesystem::path(progress_file).parent_path();
    if (!std::filesystem::exists(dir_path)) {
      RCLCPP_ERROR(node_->get_logger(), "Field directory does not exist: %s",
        dir_path.c_str());
      return BT::NodeStatus::FAILURE;
    }

    try {
      nlohmann::json j;

      // Get current timestamp
      auto now = std::chrono::system_clock::now();
      auto time_t_now = std::chrono::system_clock::to_time_t(now);
      std::stringstream ss;
      ss << std::put_time(std::localtime(&time_t_now), "%Y-%m-%d %H:%M:%S");
      j["timestamp"] = ss.str();

      j["field_name"] = field_name;
      j["stage"] = stage;
      j["pass_number"] = pass_number;
      j["completed"] = false;

      // Save geo_points
      j["geo_points"] = nlohmann::json::array();
      for (const auto & pt : geo_points) {
        j["geo_points"].push_back({
          {"latitude", pt.latitude},
          {"longitude", pt.longitude},
          {"altitude", pt.altitude}
        });
      }

      // Save map_points (for direct resume without re-conversion)
      j["map_points"] = nlohmann::json::array();
      for (const auto & pose : map_points) {
        j["map_points"].push_back({
          {"frame_id", pose.header.frame_id},
          {"x", pose.pose.position.x},
          {"y", pose.pose.position.y},
          {"z", pose.pose.position.z},
          {"qx", pose.pose.orientation.x},
          {"qy", pose.pose.orientation.y},
          {"qz", pose.pose.orientation.z},
          {"qw", pose.pose.orientation.w}
        });
      }

      // Write to file
      std::ofstream file(progress_file);
      if (!file.is_open()) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to open progress file for writing: %s",
          progress_file.c_str());
        return BT::NodeStatus::FAILURE;
      }

      file << j.dump(2);
      file.close();

      RCLCPP_INFO(node_->get_logger(),
        "Saved progress to %s (stage: %s, pass: %d)",
        progress_file.c_str(), stage.c_str(), pass_number);

      return BT::NodeStatus::SUCCESS;

    } catch (const std::exception & e) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to save progress: %s", e.what());
      return BT::NodeStatus::FAILURE;
    }
  }

private:
  rclcpp::Node::SharedPtr node_;
};

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<SaveOneLineProgress>("SaveOneLineProgress");
}
