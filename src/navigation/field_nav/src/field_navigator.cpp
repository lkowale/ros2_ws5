// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#include <string>
#include <memory>
#include <vector>
#include <fstream>
#include <filesystem>

#include "field_nav/field_navigator.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <nlohmann/json.hpp>

namespace field_nav
{

bool FieldNavigator::configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
  std::shared_ptr<nav2_util::OdomSmoother> /*odom_smoother*/)
{
  auto node = parent_node.lock();
  if (!node) {
    return false;
  }

  nav2_util::declare_parameter_if_not_declared(
    node, getName() + ".fields_directory",
    rclcpp::ParameterValue("/home/aa/ros2_ws4/src/fields"));

  node->get_parameter(getName() + ".fields_directory", fields_directory_);

  return true;
}

std::string FieldNavigator::getDefaultBTFilepath(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node)
{
  std::string default_bt_xml_filename;
  auto node = parent_node.lock();

  if (!node) {
    return default_bt_xml_filename;
  }

  if (!node->has_parameter("default_field_bt_xml")) {
    std::string pkg_share_dir =
      ament_index_cpp::get_package_share_directory("field_nav");
    node->declare_parameter<std::string>(
      "default_field_bt_xml",
      pkg_share_dir + "/behavior_trees/field_nav.xml");
  }

  node->get_parameter("default_field_bt_xml", default_bt_xml_filename);

  return default_bt_xml_filename;
}

bool FieldNavigator::loadDirectedLinesFromFile(
  const std::string & field_name,
  std::vector<FieldLine> & lines)
{
  std::string field_dir = fields_directory_ + "/" + field_name;
  std::string file_path;

  const std::string suffix = "_directed_turns.geojson";
  try {
    for (const auto & entry : std::filesystem::directory_iterator(field_dir)) {
      auto filename = entry.path().filename().string();
      if (filename.size() > suffix.size() &&
        filename.compare(filename.size() - suffix.size(), suffix.size(), suffix) == 0)
      {
        file_path = entry.path().string();
        break;
      }
    }
  } catch (const std::filesystem::filesystem_error & e) {
    RCLCPP_ERROR(logger_, "Failed to scan field directory %s: %s", field_dir.c_str(), e.what());
    return false;
  }

  if (file_path.empty()) {
    RCLCPP_ERROR(logger_, "No *_directed_turns.geojson found in %s", field_dir.c_str());
    return false;
  }

  std::ifstream file(file_path);
  if (!file.is_open()) {
    RCLCPP_ERROR(logger_, "Failed to open field lines file: %s", file_path.c_str());
    return false;
  }

  try {
    nlohmann::json j;
    file >> j;
    lines.clear();

    for (const auto & feature : j["features"]) {
      FieldLine line;
      const auto & coords = feature["geometry"]["coordinates"];
      line.start.longitude = coords[0][0].get<double>();
      line.start.latitude = coords[0][1].get<double>();
      line.start.altitude = 0.0;
      line.end.longitude = coords[1][0].get<double>();
      line.end.latitude = coords[1][1].get<double>();
      line.end.altitude = 0.0;
      line.turn = feature["properties"]["turn"].get<std::string>();
      lines.push_back(line);
    }

    // Load headland boundaries from FeatureCollection properties (optional)
    headland_boundaries_.valid = false;
    if (j.contains("properties") && !j["properties"].is_null()) {
      const auto & props = j["properties"];
      if (props.contains("headland_sw") && props.contains("headland_ne")) {
        headland_boundaries_.sw.longitude = props["headland_sw"][0].get<double>();
        headland_boundaries_.sw.latitude  = props["headland_sw"][1].get<double>();
        headland_boundaries_.ne.longitude = props["headland_ne"][0].get<double>();
        headland_boundaries_.ne.latitude  = props["headland_ne"][1].get<double>();
        headland_boundaries_.valid = true;
        RCLCPP_INFO(logger_,
          "Headland boundaries: SW=[%.8f,%.8f] NE=[%.8f,%.8f]",
          headland_boundaries_.sw.longitude, headland_boundaries_.sw.latitude,
          headland_boundaries_.ne.longitude, headland_boundaries_.ne.latitude);
      }
    }

    RCLCPP_INFO(logger_, "Loaded %zu lines from %s", lines.size(), file_path.c_str());
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Failed to parse field lines file %s: %s", file_path.c_str(), e.what());
    return false;
  }
}

void FieldNavigator::loadSwathOffsets(
  const std::string & field_name,
  std::vector<FieldLine> & lines)
{
  std::string file_path = fields_directory_ + "/" + field_name + "/swath_offsets.json";
  std::ifstream file(file_path);
  if (!file.is_open()) {
    return;  // no offsets file — all offsets stay 0
  }

  try {
    nlohmann::json j;
    file >> j;
    int applied = 0;
    for (auto it = j.begin(); it != j.end(); ++it) {
      int idx;
      try { idx = std::stoi(it.key()); } catch (...) { continue; }
      double offset_m = it.value().get<double>() / 100.0;  // cm → m
      if (idx >= 0 && static_cast<size_t>(idx) < lines.size()) {
        lines[idx].lateral_offset_m = offset_m;
        ++applied;
      }
    }
    RCLCPP_INFO(logger_, "Loaded swath offsets from %s (%d entries)", file_path.c_str(), applied);
  } catch (const std::exception & e) {
    RCLCPP_WARN(logger_, "Failed to parse swath offsets %s: %s", file_path.c_str(), e.what());
  }
}

bool FieldNavigator::loadProgressFromFile(
  const std::string & field_name,
  int32_t & current_line_index,
  std::string & stage)
{
  std::string file_path = fields_directory_ + "/" + field_name + "/field_progress.json";

  std::ifstream file(file_path);
  if (!file.is_open()) {
    return false;
  }

  try {
    nlohmann::json j;
    file >> j;

    current_line_index = j["current_line_index"].get<int32_t>();
    stage = j["stage"].get<std::string>();

    RCLCPP_INFO(logger_, "Loaded progress: line_index=%d, stage=%s",
      current_line_index, stage.c_str());
    return true;
  } catch (const std::exception & e) {
    RCLCPP_WARN(logger_, "Failed to parse progress file: %s", e.what());
    return false;
  }
}

bool FieldNavigator::saveProgressToFile(
  const std::string & field_name,
  int32_t current_line_index,
  const std::string & stage)
{
  std::string file_path = fields_directory_ + "/" + field_name + "/field_progress.json";

  try {
    nlohmann::json j;
    j["current_line_index"] = current_line_index;
    j["stage"] = stage;

    std::ofstream file(file_path);
    if (!file.is_open()) {
      RCLCPP_ERROR(logger_, "Failed to open progress file for writing: %s", file_path.c_str());
      return false;
    }

    file << j.dump(2);
    RCLCPP_INFO(logger_, "Saved progress: line_index=%d, stage=%s",
      current_line_index, stage.c_str());
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Failed to save progress: %s", e.what());
    return false;
  }
}

bool FieldNavigator::goalReceived(ActionT::Goal::ConstSharedPtr goal)
{
  auto bt_xml_filename = goal->behavior_tree;

  // Use default BT if none specified
  if (bt_xml_filename.empty()) {
    bt_xml_filename = bt_action_server_->getCurrentBTFilename();
  }

  if (!bt_action_server_->loadBehaviorTree(bt_xml_filename)) {
    RCLCPP_ERROR(
      logger_, "BT file not found: %s. Navigation canceled.",
      bt_xml_filename.c_str());
    return false;
  }

  // Load directed lines from file
  if (goal->field_name.empty()) {
    RCLCPP_ERROR(logger_, "field_name is required for FieldNavigator");
    return false;
  }

  if (!loadDirectedLinesFromFile(goal->field_name, field_lines_)) {
    RCLCPP_ERROR(logger_, "Failed to load directed_lines from field: %s", goal->field_name.c_str());
    return false;
  }
  loadSwathOffsets(goal->field_name, field_lines_);

  total_lines_ = static_cast<int32_t>(field_lines_.size());

  if (total_lines_ == 0) {
    RCLCPP_ERROR(logger_, "No lines found in field: %s", goal->field_name.c_str());
    return false;
  }

  // Determine starting line index
  int32_t start_line_index = 0;
  std::string stage = "approach";

  if (goal->start_line_index == -1) {
    // Try to load from progress file
    if (loadProgressFromFile(goal->field_name, start_line_index, stage)) {
      RCLCPP_INFO(logger_, "Resuming from saved progress: line %d, stage %s",
        start_line_index, stage.c_str());
    }
  } else {
    // Fresh start — delete any saved progress so the BT doesn't reload it
    std::string progress_file = fields_directory_ + "/" + goal->field_name + "/field_progress.json";
    std::remove(progress_file.c_str());
    RCLCPP_INFO(logger_, "Fresh start — cleared saved progress for field '%s'",
      goal->field_name.c_str());
  }
  if (goal->start_line_index > 0) {
    start_line_index = goal->start_line_index;
    if (start_line_index >= total_lines_) {
      start_line_index = 0;
      RCLCPP_WARN(logger_, "start_line_index exceeds total lines, starting from 0");
    }
  }

  // Set blackboard variables for BT nodes
  auto blackboard = bt_action_server_->getBlackboard();

  // Current line's geo_points
  std::vector<geographic_msgs::msg::GeoPoint> geo_points = {
    field_lines_[start_line_index].start,
    field_lines_[start_line_index].end
  };
  blackboard->set<std::vector<geographic_msgs::msg::GeoPoint>>("geo_points", geo_points);

  // Field navigation specific variables
  blackboard->set<std::string>("field_name", goal->field_name);
  blackboard->set<std::string>("fields_directory", fields_directory_);
  blackboard->set<int32_t>("current_line_index", start_line_index);
  blackboard->set<int32_t>("total_lines", total_lines_);
  blackboard->set<std::string>("stage", stage);
  blackboard->set<std::string>("turn_direction", field_lines_[start_line_index].turn);

  // Store all lines for BT access
  blackboard->set<std::vector<FieldLine>>("field_lines", field_lines_);

  // Headland boundaries (if present in geojson) — published as individual GeoPoints
  if (headland_boundaries_.valid) {
    blackboard->set<geographic_msgs::msg::GeoPoint>("headland_sw", headland_boundaries_.sw);
    blackboard->set<geographic_msgs::msg::GeoPoint>("headland_ne", headland_boundaries_.ne);
  }

  RCLCPP_INFO(logger_, "FieldNavigator received goal:");
  RCLCPP_INFO(logger_, "Field: %s, Total lines: %d, Starting at line: %d",
    goal->field_name.c_str(), total_lines_, start_line_index);
  RCLCPP_INFO(logger_, "First line - Start: [lat: %lf, lon: %lf], End: [lat: %lf, lon: %lf], Turn: %s",
    geo_points[0].latitude, geo_points[0].longitude,
    geo_points[1].latitude, geo_points[1].longitude,
    field_lines_[start_line_index].turn.c_str());

  return true;
}

void FieldNavigator::onLoop()
{
  // Publish feedback
  auto feedback_msg = std::make_shared<ActionT::Feedback>();
  auto blackboard = bt_action_server_->getBlackboard();

  int32_t current_line_index = 0;
  int32_t total_lines = 0;
  std::string current_stage;
  float distance_traveled = 0.0f;
  float total_distance = 0.0f;

  (void)blackboard->get("current_line_index", current_line_index);
  (void)blackboard->get("total_lines", total_lines);
  (void)blackboard->get("stage", current_stage);
  (void)blackboard->get("distance_traveled", distance_traveled);
  (void)blackboard->get("total_distance", total_distance);

  feedback_msg->current_line_index = current_line_index;
  feedback_msg->total_lines = total_lines;
  feedback_msg->current_stage = current_stage;
  feedback_msg->distance_traveled = distance_traveled;
  feedback_msg->total_distance = total_distance;

  bt_action_server_->publishFeedback(feedback_msg);
}

void FieldNavigator::onPreempt(ActionT::Goal::ConstSharedPtr goal)
{
  RCLCPP_INFO(logger_, "Received goal preemption request");

  // Re-initialize with new goal
  if (!goal->field_name.empty()) {
    loadDirectedLinesFromFile(goal->field_name, field_lines_);
    loadSwathOffsets(goal->field_name, field_lines_);
    total_lines_ = static_cast<int32_t>(field_lines_.size());

    auto blackboard = bt_action_server_->getBlackboard();
    blackboard->set<std::vector<FieldLine>>("field_lines", field_lines_);
    blackboard->set<int32_t>("total_lines", total_lines_);
    blackboard->set<int32_t>("current_line_index", 0);

    if (total_lines_ > 0) {
      std::vector<geographic_msgs::msg::GeoPoint> geo_points = {
        field_lines_[0].start,
        field_lines_[0].end
      };
      blackboard->set<std::vector<geographic_msgs::msg::GeoPoint>>("geo_points", geo_points);
      blackboard->set<std::string>("turn_direction", field_lines_[0].turn);
    }
  }
}

void FieldNavigator::goalCompleted(
  typename ActionT::Result::SharedPtr result,
  const nav2_behavior_tree::BtStatus final_bt_status)
{
  auto blackboard = bt_action_server_->getBlackboard();
  int32_t current_line_index = 0;
  (void)blackboard->get("current_line_index", current_line_index);

  if (final_bt_status == nav2_behavior_tree::BtStatus::SUCCEEDED) {
    result->error_code = ActionT::Result::NONE;
    result->lines_completed = total_lines_;
  } else if (final_bt_status == nav2_behavior_tree::BtStatus::CANCELED) {
    result->error_code = ActionT::Result::CANCELLED;
    result->lines_completed = current_line_index;
  } else {
    result->error_code = ActionT::Result::FAILED;
    result->lines_completed = current_line_index;
  }

  RCLCPP_INFO(
    logger_, "FieldNavigator completed with status: %d, lines completed: %d",
    static_cast<int>(final_bt_status), result->lines_completed);
}

}  // namespace field_nav

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(field_nav::FieldNavigator, nav2_core::NavigatorBase)
