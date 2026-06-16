#ifndef TOOL_LINE_FOLLOWER_CONTROLLER__TOOL_LINE_FOLLOWER_CONTROLLER_HPP_
#define TOOL_LINE_FOLLOWER_CONTROLLER__TOOL_LINE_FOLLOWER_CONTROLLER_HPP_

#include <string>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float32.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/buffer.h"

namespace tool_line_follower_controller
{

/**
 * Cross-track PID controller that steers the tool_link frame onto the path.
 *
 * Uses only the tool_link TF position — no heading/yaw.
 * angular.z = -PID(cross_track_error of tool_link vs path).
 * Forward speed is modulated proportionally to cross-track error.
 *
 * Ported from ros2_ws4 GpsLineFollowerController; gps_link replaced by
 * tool_link (rear axle / implement hitch point).
 */
class ToolLineFollowerController : public nav2_core::Controller
{
public:
  ToolLineFollowerController() = default;
  ~ToolLineFollowerController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  bool getToolPose(geometry_msgs::msg::PoseStamped & tool_pose);

  double findClosestPointOnPath(
    const geometry_msgs::msg::Point & position,
    geometry_msgs::msg::Point & closest_point,
    size_t & closest_idx);

  double calculateCrossTrackError(
    const geometry_msgs::msg::Point & position,
    size_t path_idx);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> global_path_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32>> cross_track_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32>> bearing_pub_;
  rclcpp::Logger logger_{rclcpp::get_logger("ToolLineFollowerController")};
  rclcpp::Clock::SharedPtr clock_;

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string global_frame_;

  nav_msgs::msg::Path global_plan_;
  size_t current_path_idx_{0};

  std::string tool_frame_id_;
  double desired_linear_vel_;
  double max_linear_vel_;
  double min_linear_vel_;
  double max_angular_vel_;
  double transform_tolerance_;

  double kp_near_;
  double kp_mid_;
  double kp_far_;
  double ki_cross_track_;
  double kd_cross_track_;
  double ki_clamp_;
  double ki_entry_ignore_s_;
  double near_line_threshold_;
  double mid_line_threshold_;
  double tool_lateral_offset_;

  double approach_velocity_scaling_dist_;
  double approach_min_velocity_;

  double cross_track_error_integral_{0.0};
  double prev_cross_track_error_{0.0};
  rclcpp::Time prev_time_;
  bool pid_initialized_{false};
  rclcpp::Time plan_start_time_;

  double speed_limit_{1.0};
  bool speed_limit_is_percentage_{false};
};

}  // namespace tool_line_follower_controller

#endif  // TOOL_LINE_FOLLOWER_CONTROLLER__TOOL_LINE_FOLLOWER_CONTROLLER_HPP_
