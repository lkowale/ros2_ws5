#ifndef TOOL_LINE_FOLLOWER_CONTROLLER__TOOL_ARC_FOLLOWER_CONTROLLER_HPP_
#define TOOL_LINE_FOLLOWER_CONTROLLER__TOOL_ARC_FOLLOWER_CONTROLLER_HPP_

#include "tool_line_follower_controller/tool_line_follower_controller.hpp"

namespace tool_arc_follower_controller
{

/**
 * Extends ToolLineFollowerController with a curvature feed-forward term so
 * the robot maintains the correct turning rate when following arc paths even
 * when cross-track error is zero.
 *
 * angular.z = -PID(cross_track) + linear_vel * kappa * curvature_ff_gain
 */
class ToolArcFollowerController : public tool_line_follower_controller::ToolLineFollowerController
{
public:
  ToolArcFollowerController() = default;
  ~ToolArcFollowerController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

private:
  double computePathCurvature(size_t idx) const;
  double curvature_ff_gain_;
};

}  // namespace tool_arc_follower_controller

#endif  // TOOL_LINE_FOLLOWER_CONTROLLER__TOOL_ARC_FOLLOWER_CONTROLLER_HPP_
