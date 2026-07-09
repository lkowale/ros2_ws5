#include "tool_line_follower_controller/pause_progress_checker.hpp"

#include <cmath>
#include <string>
#include "nav2_util/node_utils.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

namespace pause_progress_checker
{

void PauseProgressChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name)
{
  plugin_name_ = plugin_name;
  auto node = parent.lock();
  clock_ = node->get_clock();
  logger_ = node->get_logger();

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".required_movement_radius", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".movement_time_allowance", rclcpp::ParameterValue(10.0));

  radius_ = node->get_parameter(plugin_name + ".required_movement_radius").as_double();
  double time_allowance_param =
    node->get_parameter(plugin_name + ".movement_time_allowance").as_double();
  time_allowance_ = rclcpp::Duration::from_seconds(time_allowance_param);

  pause_pub_ = node->create_publisher<solbot5_msgs::msg::Pause>("/pause", rclcpp::QoS(10));

  RCLCPP_INFO(logger_, "%s: radius=%.2f time_allowance=%.1fs",
    plugin_name_.c_str(), radius_, time_allowance_param);
}

bool PauseProgressChecker::check(geometry_msgs::msg::PoseStamped & current_pose)
{
  geometry_msgs::msg::Pose2D pose2d;
  pose2d.x = current_pose.pose.position.x;
  pose2d.y = current_pose.pose.position.y;

  if (!baseline_pose_set_) {
    baseline_pose_ = pose2d;
    baseline_time_ = clock_->now();
    baseline_pose_set_ = true;
    return true;
  }

  if (poseDistance(baseline_pose_, pose2d) > radius_) {
    // Made enough progress — reset baseline and clear any existing pause
    baseline_pose_ = pose2d;
    baseline_time_ = clock_->now();
    if (paused_) {
      paused_ = false;
      publishPause(false);
      RCLCPP_INFO(logger_, "%s: progress resumed, pause cleared", plugin_name_.c_str());
    }
    return true;
  }

  if (clock_->now() - baseline_time_ > time_allowance_) {
    if (!paused_) {
      paused_ = true;
      publishPause(true);
      RCLCPP_WARN(logger_, "%s: no progress for %.1fs — publishing HIGH pause",
        plugin_name_.c_str(), time_allowance_.seconds());
    }
    // Return true so nav2 does NOT cancel the goal — pause handles the stop
    return true;
  }

  return true;
}

void PauseProgressChecker::reset()
{
  baseline_pose_set_ = false;
  if (paused_) {
    paused_ = false;
    publishPause(false);
  }
}

void PauseProgressChecker::publishPause(bool paused)
{
  solbot5_msgs::msg::Pause msg;
  msg.paused = paused;
  msg.source = plugin_name_;
  msg.reason = paused ? "No progress detected — robot may be stuck" : "Progress resumed";
  msg.severity = paused
    ? solbot5_msgs::msg::Pause::SEVERITY_HIGH
    : solbot5_msgs::msg::Pause::SEVERITY_NONE;
  pause_pub_->publish(msg);
}

double PauseProgressChecker::poseDistance(
  const geometry_msgs::msg::Pose2D & a,
  const geometry_msgs::msg::Pose2D & b)
{
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

}  // namespace pause_progress_checker

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  pause_progress_checker::PauseProgressChecker, nav2_core::ProgressChecker)
