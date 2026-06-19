#ifndef TOOL_LINE_FOLLOWER_CONTROLLER__RS_PATH_CONTROLLER_HPP_
#define TOOL_LINE_FOLLOWER_CONTROLLER__RS_PATH_CONTROLLER_HPP_

#include <string>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"

namespace rs_path_controller
{

// Stanley-style path-tracking controller for Ackermann robots on RS paths.
//
// steering = heading_err + atan2(k_cross * cross_track_err, v)
//
// - heading_err: angle between robot yaw and path tangent at closest point.
// - cross_track_err: signed lateral distance from closest path point.
// - Speed is constant (desired_linear_vel) scaled near the goal and for
//   high-curvature segments.
// - Reverse segments are detected from waypoint yaw (flipped by π in RS paths)
//   and handled by negating linear velocity and adjusting error signs.
// - Tracks tool_link (rear axle) position to match the RS planner frame.
class RsPathController : public nav2_core::Controller
{
public:
  RsPathController() = default;
  ~RsPathController() override = default;

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

private:
  // Find index of closest waypoint to position, searching forward from current_idx_.
  size_t closestIndex(double px, double py) const;

  // Signed cross-track error at waypoint idx: positive = robot is left of path.
  double crossTrackError(double px, double py, size_t idx) const;

  // Path tangent yaw at index idx (from idx→idx+1 segment direction).
  double tangentYaw(size_t idx) const;

  // True if path segment at idx is a reverse segment (waypoint yaw ≈ tangent+π).
  bool isReverse(size_t idx) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  rclcpp::Logger logger_{rclcpp::get_logger("RsPathController")};
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string global_frame_;

  nav_msgs::msg::Path global_plan_;
  size_t current_idx_{0};

  // Parameters
  double desired_linear_vel_{1.0};
  double max_angular_vel_{1.0};
  double k_heading_{2.0};    // heading error gain (rad/rad)
  double k_cross_{1.0};      // Stanley cross-track gain (m/s normalised)
  double approach_dist_{3.0};  // distance from goal to start slowing down
  double min_approach_vel_{0.3};
  double transform_tolerance_{0.1};

  double speed_limit_{1.0};
  bool speed_limit_is_percentage_{false};

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;
};

}  // namespace rs_path_controller

#endif  // TOOL_LINE_FOLLOWER_CONTROLLER__RS_PATH_CONTROLLER_HPP_
