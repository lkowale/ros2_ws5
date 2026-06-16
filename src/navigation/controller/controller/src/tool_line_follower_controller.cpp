// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "tool_line_follower_controller/tool_line_follower_controller.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace tool_line_follower_controller
{

void ToolLineFollowerController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = node_.lock();
  plugin_name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  global_frame_ = costmap_ros_->getGlobalFrameID();
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".gps_frame_id", rclcpp::ParameterValue("gps_link"));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_linear_vel", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_linear_vel", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_vel", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".kp_near", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".kp_mid", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".kp_far", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".ki_cross_track", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".kd_cross_track", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".ki_clamp", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".ki_entry_ignore_s", rclcpp::ParameterValue(5.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".near_line_threshold", rclcpp::ParameterValue(0.15));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".mid_line_threshold", rclcpp::ParameterValue(0.60));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".tool_lateral_offset", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".approach_velocity_scaling_dist", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".approach_min_velocity", rclcpp::ParameterValue(0.3));

  node->get_parameter(plugin_name_ + ".gps_frame_id", tool_frame_id_);
  node->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
  node->get_parameter(plugin_name_ + ".max_linear_vel", max_linear_vel_);
  node->get_parameter(plugin_name_ + ".min_linear_vel", min_linear_vel_);
  node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance_);
  node->get_parameter(plugin_name_ + ".kp_near", kp_near_);
  node->get_parameter(plugin_name_ + ".kp_mid", kp_mid_);
  node->get_parameter(plugin_name_ + ".kp_far", kp_far_);
  node->get_parameter(plugin_name_ + ".ki_cross_track", ki_cross_track_);
  node->get_parameter(plugin_name_ + ".kd_cross_track", kd_cross_track_);
  node->get_parameter(plugin_name_ + ".ki_clamp", ki_clamp_);
  node->get_parameter(plugin_name_ + ".ki_entry_ignore_s", ki_entry_ignore_s_);
  node->get_parameter(plugin_name_ + ".near_line_threshold", near_line_threshold_);
  node->get_parameter(plugin_name_ + ".mid_line_threshold", mid_line_threshold_);
  node->get_parameter(plugin_name_ + ".tool_lateral_offset", tool_lateral_offset_);
  node->get_parameter(plugin_name_ + ".approach_velocity_scaling_dist", approach_velocity_scaling_dist_);
  node->get_parameter(plugin_name_ + ".approach_min_velocity", approach_min_velocity_);

  global_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
  cross_track_pub_ = node->create_publisher<std_msgs::msg::Float32>("swath_cross_track", 1);
  bearing_pub_ = node->create_publisher<std_msgs::msg::Float32>("swath_bearing", 1);

  RCLCPP_INFO(logger_,
    "ToolLineFollowerController configured. frame: %s, "
    "kp_near=%.2f kp_mid=%.2f kp_far=%.2f ki=%.2f kd=%.2f "
    "ki_clamp=%.3f ki_entry_ignore=%.1fs "
    "near=%.3fm mid=%.3fm tool_offset=%.3fm",
    tool_frame_id_.c_str(), kp_near_, kp_mid_, kp_far_,
    ki_cross_track_, kd_cross_track_, ki_clamp_, ki_entry_ignore_s_,
    near_line_threshold_, mid_line_threshold_, tool_lateral_offset_);
}

void ToolLineFollowerController::cleanup()
{
  global_path_pub_.reset();
  cross_track_pub_.reset();
  bearing_pub_.reset();
}

void ToolLineFollowerController::activate()
{
  global_path_pub_->on_activate();
  cross_track_pub_->on_activate();
  bearing_pub_->on_activate();
  pid_initialized_ = false;
  cross_track_error_integral_ = 0.0;
  prev_cross_track_error_ = 0.0;
  plan_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

void ToolLineFollowerController::deactivate()
{
  global_path_pub_->on_deactivate();
  cross_track_pub_->on_deactivate();
  bearing_pub_->on_deactivate();
}

void ToolLineFollowerController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  current_path_idx_ = 0;
  pid_initialized_ = false;
  cross_track_error_integral_ = 0.0;
  prev_cross_track_error_ = 0.0;
  plan_start_time_ = clock_->now();
  global_path_pub_->publish(path);
  RCLCPP_DEBUG(logger_, "Received new plan with %zu poses", path.poses.size());
}

void ToolLineFollowerController::setSpeedLimit(
  const double & speed_limit, const bool & percentage)
{
  speed_limit_ = speed_limit;
  speed_limit_is_percentage_ = percentage;
}

geometry_msgs::msg::TwistStamped ToolLineFollowerController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = pose.header.frame_id;
  cmd_vel.header.stamp = clock_->now();

  if (global_plan_.poses.empty()) {
    RCLCPP_WARN(logger_, "No path received");
    return cmd_vel;
  }

  geometry_msgs::msg::PoseStamped tool_pose;
  if (!getToolPose(tool_pose)) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000, "Could not get tool pose, stopping");
    return cmd_vel;
  }

  geometry_msgs::msg::Point closest_point;
  size_t closest_idx;
  findClosestPointOnPath(tool_pose.pose.position, closest_point, closest_idx);

  if (closest_idx > current_path_idx_) {
    current_path_idx_ = closest_idx;
  }

  double cross_track_error = calculateCrossTrackError(
    tool_pose.pose.position, closest_idx) - tool_lateral_offset_;

  rclcpp::Time current_time = clock_->now();
  if (!pid_initialized_) {
    prev_time_ = current_time;
    prev_cross_track_error_ = cross_track_error;
    pid_initialized_ = true;
  }

  double dt = (current_time - prev_time_).seconds();
  if (dt < 1e-6) {dt = 0.05;}

  double abs_ct = std::abs(cross_track_error);
  double kp_effective;
  if (abs_ct < near_line_threshold_) {
    kp_effective = kp_near_;
  } else if (abs_ct < mid_line_threshold_) {
    kp_effective = kp_mid_;
  } else {
    kp_effective = kp_far_;
  }

  double p_term = kp_effective * cross_track_error;

  double elapsed_since_plan = (current_time - plan_start_time_).seconds();
  if (elapsed_since_plan > ki_entry_ignore_s_) {
    cross_track_error_integral_ += cross_track_error * dt;
  }
  if (ki_cross_track_ > 1e-9) {
    cross_track_error_integral_ = std::clamp(
      cross_track_error_integral_,
      -ki_clamp_ / ki_cross_track_, ki_clamp_ / ki_cross_track_);
  }
  double i_term = ki_cross_track_ * cross_track_error_integral_;
  double d_term = kd_cross_track_ * (cross_track_error - prev_cross_track_error_) / dt;

  // Negate: positive error (left of path) → steer right (negative angular.z)
  double angular_vel = -(p_term + i_term + d_term);

  double max_vel = desired_linear_vel_;
  if (speed_limit_is_percentage_) {
    max_vel *= speed_limit_;
  } else if (speed_limit_ > 0.0) {
    max_vel = std::min(max_vel, speed_limit_);
  }

  if (approach_velocity_scaling_dist_ > 0.0) {
    double remaining_dist = 0.0;
    for (size_t i = closest_idx; i + 1 < global_plan_.poses.size(); ++i) {
      const auto & p1 = global_plan_.poses[i].pose.position;
      const auto & p2 = global_plan_.poses[i + 1].pose.position;
      remaining_dist += std::hypot(p2.x - p1.x, p2.y - p1.y);
    }
    if (remaining_dist < approach_velocity_scaling_dist_) {
      double t = remaining_dist / approach_velocity_scaling_dist_;
      max_vel = std::max(
        approach_min_velocity_,
        approach_min_velocity_ + (max_vel - approach_min_velocity_) * t);
    }
  }

  double error_factor = std::max(0.3, 1.0 - std::abs(cross_track_error));
  double linear_vel = std::clamp(max_vel * error_factor, min_linear_vel_, max_vel);

  cmd_vel.twist.linear.x = linear_vel;
  cmd_vel.twist.angular.z = std::clamp(angular_vel, -max_angular_vel_, max_angular_vel_);

  prev_time_ = current_time;
  prev_cross_track_error_ = cross_track_error;

  {
    std_msgs::msg::Float32 ct_msg;
    ct_msg.data = static_cast<float>(cross_track_error);
    cross_track_pub_->publish(ct_msg);

    if (closest_idx < global_plan_.poses.size() - 1) {
      const auto & p1 = global_plan_.poses[closest_idx].pose.position;
      const auto & p2 = global_plan_.poses[closest_idx + 1].pose.position;
      std_msgs::msg::Float32 brg_msg;
      brg_msg.data = static_cast<float>(std::atan2(p2.y - p1.y, p2.x - p1.x));
      bearing_pub_->publish(brg_msg);
    }
  }

  RCLCPP_DEBUG(logger_,
    "cross_track=%.3f  P=%.3f I=%.3f D=%.3f  cmd: v=%.2f w=%.3f",
    cross_track_error, p_term, i_term, d_term,
    cmd_vel.twist.linear.x, cmd_vel.twist.angular.z);

  return cmd_vel;
}

bool ToolLineFollowerController::getToolPose(geometry_msgs::msg::PoseStamped & tool_pose)
{
  geometry_msgs::msg::PoseStamped origin;
  origin.header.frame_id = tool_frame_id_;
  origin.header.stamp = rclcpp::Time(0);
  origin.pose.orientation.w = 1.0;
  try {
    tf_->transform(origin, tool_pose, global_frame_,
      tf2::durationFromSec(transform_tolerance_));
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_DEBUG(logger_, "TF error: %s", ex.what());
    return false;
  }
}

double ToolLineFollowerController::findClosestPointOnPath(
  const geometry_msgs::msg::Point & position,
  geometry_msgs::msg::Point & closest_point,
  size_t & closest_idx)
{
  double min_dist = std::numeric_limits<double>::max();
  closest_idx = 0;

  size_t search_start = current_path_idx_;
  if (search_start >= global_plan_.poses.size()) {search_start = 0;}

  for (size_t i = search_start; i < global_plan_.poses.size() - 1; ++i) {
    const auto & p1 = global_plan_.poses[i].pose.position;
    const auto & p2 = global_plan_.poses[i + 1].pose.position;
    double dx = p2.x - p1.x, dy = p2.y - p1.y;
    double seg_sq = dx * dx + dy * dy;
    if (seg_sq < 1e-6) {continue;}
    double t = std::clamp(
      ((position.x - p1.x) * dx + (position.y - p1.y) * dy) / seg_sq, 0.0, 1.0);
    geometry_msgs::msg::Point proj;
    proj.x = p1.x + t * dx;
    proj.y = p1.y + t * dy;
    proj.z = 0.0;
    double dist = std::hypot(position.x - proj.x, position.y - proj.y);
    if (dist < min_dist) {
      min_dist = dist;
      closest_point = proj;
      closest_idx = i;
    }
  }

  if (!global_plan_.poses.empty()) {
    const auto & last = global_plan_.poses.back().pose.position;
    double dist = std::hypot(position.x - last.x, position.y - last.y);
    if (dist < min_dist) {
      min_dist = dist;
      closest_point = last;
      closest_idx = global_plan_.poses.size() - 1;
    }
  }

  return min_dist;
}

double ToolLineFollowerController::calculateCrossTrackError(
  const geometry_msgs::msg::Point & position,
  size_t path_idx)
{
  if (path_idx >= global_plan_.poses.size() - 1) {
    path_idx = global_plan_.poses.size() - 2;
  }
  const auto & p1 = global_plan_.poses[path_idx].pose.position;
  const auto & p2 = global_plan_.poses[path_idx + 1].pose.position;
  double dx = p2.x - p1.x, dy = p2.y - p1.y;
  double len = std::hypot(dx, dy);
  if (len < 1e-6) {return 0.0;}
  // Unit left-normal
  double nx = -dy / len, ny = dx / len;
  return (position.x - p1.x) * nx + (position.y - p1.y) * ny;
}

}  // namespace tool_line_follower_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  tool_line_follower_controller::ToolLineFollowerController,
  nav2_core::Controller)
