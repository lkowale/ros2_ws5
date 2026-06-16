#include "drive_arc/drive_arc_behavior.hpp"

#include <cmath>
#include <memory>

#include "nav2_util/robot_utils.hpp"
#include "tf2/utils.h"

namespace drive_arc
{

DriveArcBehavior::DriveArcBehavior()
: nav2_behaviors::TimedBehavior<solbot5_msgs::action::DriveArc>(),
  feedback_(std::make_shared<ActionT::Feedback>())
{}

nav2_behaviors::ResultStatus DriveArcBehavior::onRun(
  const std::shared_ptr<const ActionT::Goal> command)
{
  if (command->radius <= 0.0) {
    RCLCPP_ERROR(logger_, "DriveArc: radius must be > 0");
    return {nav2_behaviors::Status::FAILED, ActionT::Result::TF_ERROR};
  }

  target_angle_  = command->angle;   // signed
  linear_speed_  = command->speed;   // signed

  // angular.z = v / r, sign from: forward+left → positive omega
  // omega = v * sin(angle_sign) / r  — but simpler:
  // left arc  (+angle): omega = +|v|/r  (forward) or -|v|/r (reverse)
  // right arc (-angle): omega = -|v|/r  (forward) or +|v|/r (reverse)
  // Unified: omega = speed / radius * sign(angle)
  double angle_sign = (command->angle >= 0.0) ? 1.0 : -1.0;
  angular_speed_ = linear_speed_ / command->radius * angle_sign;

  time_allowance_ = rclcpp::Duration::from_seconds(
    command->time_allowance > 0.0 ? command->time_allowance : 60.0);
  end_time_ = clock_->now() + time_allowance_;

  if (!nav2_util::getCurrentPose(
      initial_pose_, *tf_, local_frame_, robot_base_frame_, transform_tolerance_))
  {
    RCLCPP_ERROR(logger_, "DriveArc: cannot get initial pose");
    return {nav2_behaviors::Status::FAILED, ActionT::Result::TF_ERROR};
  }

  RCLCPP_INFO(logger_,
    "DriveArc: radius=%.2f angle=%.1f° speed=%.2f omega=%.3f",
    command->radius, command->angle * 180.0 / M_PI, linear_speed_, angular_speed_);

  return {nav2_behaviors::Status::SUCCEEDED, ActionT::Result::NONE};
}

nav2_behaviors::ResultStatus DriveArcBehavior::onCycleUpdate()
{
  if (clock_->now() > end_time_) {
    stopRobot();
    RCLCPP_WARN(logger_, "DriveArc: timeout");
    return {nav2_behaviors::Status::FAILED, ActionT::Result::TIMEOUT};
  }

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, local_frame_, robot_base_frame_, transform_tolerance_))
  {
    RCLCPP_ERROR(logger_, "DriveArc: cannot get current pose");
    return {nav2_behaviors::Status::FAILED, ActionT::Result::TF_ERROR};
  }

  // Measure swept angle as difference in yaw from initial pose.
  double yaw0 = tf2::getYaw(initial_pose_.pose.orientation);
  double yaw1 = tf2::getYaw(current_pose.pose.orientation);
  double swept = yaw1 - yaw0;
  // Normalise to [-π, π]
  while (swept >  M_PI) swept -= 2.0 * M_PI;
  while (swept < -M_PI) swept += 2.0 * M_PI;

  feedback_->angle_traveled = static_cast<float>(swept);
  action_server_->publish_feedback(feedback_);

  // Done when |swept| >= |target_angle|
  if (std::fabs(swept) >= std::fabs(target_angle_)) {
    stopRobot();
    RCLCPP_INFO(logger_, "DriveArc: done, swept=%.1f°", swept * 180.0 / M_PI);
    return {nav2_behaviors::Status::SUCCEEDED, ActionT::Result::NONE};
  }

  auto cmd = std::make_unique<geometry_msgs::msg::TwistStamped>();
  cmd->header.stamp = clock_->now();
  cmd->header.frame_id = robot_base_frame_;
  cmd->twist.linear.x  = linear_speed_;
  cmd->twist.angular.z = angular_speed_;
  vel_pub_->publish(std::move(cmd));

  return {nav2_behaviors::Status::RUNNING, ActionT::Result::NONE};
}

}  // namespace drive_arc

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(drive_arc::DriveArcBehavior, nav2_core::Behavior)
