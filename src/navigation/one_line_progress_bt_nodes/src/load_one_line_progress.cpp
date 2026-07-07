// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#include <string>
#include <fstream>
#include <filesystem>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"
#include "geographic_msgs/msg/geo_point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <nlohmann/json.hpp>

class LoadOneLineProgress : public BT::SyncActionNode
{
public:
  LoadOneLineProgress(
    const std::string & name,
    const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
    node_ = rclcpp::Node::make_shared("load_one_line_progress_bt_node");
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("fields_directory", "/home/aa/ros2_ws4/src/fields",
        "Base directory for field files"),
      BT::InputPort<std::string>("field_name", "Field name for progress file location"),
      BT::OutputPort<std::vector<geographic_msgs::msg::GeoPoint>>("geo_points",
        "Loaded geo_points (if progress exists)"),
      BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>("map_points",
        "Loaded map_points (if progress exists)"),
      BT::OutputPort<std::string>("stage", "Loaded stage"),
      BT::OutputPort<int>("pass_number", "Loaded pass number"),
      BT::OutputPort<bool>("has_progress", "True if progress file was loaded"),
    };
  }

  BT::NodeStatus tick() override
  {
    std::string fields_directory;
    std::string field_name;

    if (!getInput("fields_directory", fields_directory)) {
      fields_directory = "/home/aa/ros2_ws4/src/fields";
    }

    if (!getInput("field_name", field_name) || field_name.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "Missing required input [field_name]");
      return BT::NodeStatus::FAILURE;
    }

    // Build progress file path
    std::string progress_file = fields_directory + "/" + field_name + "/progress.json";

    // Check if file exists
    if (!std::filesystem::exists(progress_file)) {
      RCLCPP_INFO(node_->get_logger(),
        "No progress file found at %s, starting fresh", progress_file.c_str());
      setOutput("has_progress", false);
      // Return FAILURE so Fallback can run ConvertGeoPoints
      return BT::NodeStatus::FAILURE;
    }

    try {
      std::ifstream file(progress_file);
      if (!file.is_open()) {
        RCLCPP_WARN(node_->get_logger(), "Failed to open progress file: %s",
          progress_file.c_str());
        setOutput("has_progress", false);
        return BT::NodeStatus::FAILURE;
      }

      nlohmann::json j;
      file >> j;
      file.close();

      // Check if this was a completed job (shouldn't resume from completed)
      if (j.contains("completed") && j["completed"].get<bool>()) {
        RCLCPP_INFO(node_->get_logger(),
          "Progress file indicates completed job, starting fresh");
        setOutput("has_progress", false);
        return BT::NodeStatus::FAILURE;
      }

      // Load geo_points
      std::vector<geographic_msgs::msg::GeoPoint> geo_points;
      if (j.contains("geo_points") && j["geo_points"].is_array()) {
        for (const auto & pt : j["geo_points"]) {
          geographic_msgs::msg::GeoPoint geo_pt;
          geo_pt.latitude = pt["latitude"].get<double>();
          geo_pt.longitude = pt["longitude"].get<double>();
          geo_pt.altitude = pt.value("altitude", 0.0);
          geo_points.push_back(geo_pt);
        }
      }

      // Load map_points
      std::vector<geometry_msgs::msg::PoseStamped> map_points;
      if (j.contains("map_points") && j["map_points"].is_array()) {
        for (const auto & mp : j["map_points"]) {
          geometry_msgs::msg::PoseStamped pose;
          pose.header.frame_id = mp.value("frame_id", "map");
          pose.pose.position.x = mp["x"].get<double>();
          pose.pose.position.y = mp["y"].get<double>();
          pose.pose.position.z = mp.value("z", 0.0);
          pose.pose.orientation.x = mp.value("qx", 0.0);
          pose.pose.orientation.y = mp.value("qy", 0.0);
          pose.pose.orientation.z = mp.value("qz", 0.0);
          pose.pose.orientation.w = mp.value("qw", 1.0);
          map_points.push_back(pose);
        }
      }

      // If no map_points in progress, need to do geo conversion
      if (map_points.empty()) {
        RCLCPP_INFO(node_->get_logger(),
          "Progress file has no map_points, need geo conversion");
        setOutput("has_progress", false);
        return BT::NodeStatus::FAILURE;
      }

      // Load stage and pass_number
      std::string stage = j.value("stage", "approach");
      int pass_number = j.value("pass_number", 0);

      // Set outputs
      if (!geo_points.empty()) {
        setOutput("geo_points", geo_points);
      }
      setOutput("map_points", map_points);
      setOutput("stage", stage);
      setOutput("pass_number", pass_number);
      setOutput("has_progress", true);

      RCLCPP_INFO(node_->get_logger(),
        "Loaded progress from %s (stage: %s, pass: %d, %zu geo_points, %zu map_points)",
        progress_file.c_str(), stage.c_str(), pass_number,
        geo_points.size(), map_points.size());

      return BT::NodeStatus::SUCCESS;

    } catch (const std::exception & e) {
      RCLCPP_WARN(node_->get_logger(), "Failed to parse progress file %s: %s",
        progress_file.c_str(), e.what());
      setOutput("has_progress", false);
      return BT::NodeStatus::FAILURE;
    }
  }

private:
  rclcpp::Node::SharedPtr node_;
};

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<LoadOneLineProgress>("LoadOneLineProgress");
}
