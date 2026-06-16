#ifndef TOOL_LINE_FOLLOWER_CONTROLLER__TOOL_RPP_CONTROLLER_HPP_
#define TOOL_LINE_FOLLOWER_CONTROLLER__TOOL_RPP_CONTROLLER_HPP_

#include <string>
#include <memory>

#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace tool_rpp_controller
{

/**
 * RPP controller that substitutes a configurable TF frame pose (position +
 * orientation) for base_footprint before passing it to the pure-pursuit
 * geometry.  Set gps_frame_id to tool_link to place the lookahead circle at
 * the implement hitch point and use dual-antenna heading for arc following.
 */
class ToolAnchoredRPPController
  : public nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
{
public:
  ToolAnchoredRPPController() = default;
  ~ToolAnchoredRPPController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & speed,
    nav2_core::GoalChecker * goal_checker) override;

private:
  std::string gps_frame_id_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Logger logger_{rclcpp::get_logger("ToolAnchoredRPPController")};
};

}  // namespace tool_rpp_controller

#endif  // TOOL_LINE_FOLLOWER_CONTROLLER__TOOL_RPP_CONTROLLER_HPP_
