#ifndef TOOL_LINE_FOLLOWER_CONTROLLER__RS_PATH_CONTROLLER_HPP_
#define TOOL_LINE_FOLLOWER_CONTROLLER__RS_PATH_CONTROLLER_HPP_

#include <array>
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

// MPC-style path controller for Ackermann robots on RS paths.
//
// Each cycle:
//   1. Find closest path waypoint to rear axle (forward-only scan).
//   2. Compute adaptive lookahead L in [min_look, max_look], proportional
//      to distance from robot to current_idx_.
//   3. Simulate robot arc at candidate steering angles; find where it
//      intersects the path near the lookahead point (cross point).
//   4. Binary-search the steering angle that places the cross point at L:
//        cross < L  → less steering   (arc converges too fast)
//        cross > L or no cross → more steering  (arc misses path)
//   5. Output: w = v * tan(delta) / wheelbase.
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
  // Closest path waypoint to (px,py), scanning forward from current_idx_.
  size_t closestIndex(double px, double py) const;

  // First waypoint >= dist of path arc ahead of from_idx.
  size_t lookaheadIndex(size_t from_idx, double dist) const;

  // Last waypoint index in the same forward/reverse segment as from_idx.
  size_t segmentEndIndex(size_t from_idx) const;

  // Path tangent yaw at idx (from idx→idx+1 direction).
  double tangentYaw(size_t idx) const;

  // True if path segment at idx is a reverse segment.
  bool isReverse(size_t idx) const;

  // Simulate Ackermann bicycle model: returns arc as (x,y) points.
  std::vector<std::array<double, 2>> simulateArc(
    double x, double y, double yaw, double delta, bool reverse) const;

  // Distance from robot to first intersection of arc with path segments
  // [from_idx, to_idx]. Returns -1 if no intersection.
  double arcPathIntersection(
    const std::vector<std::array<double, 2>> & arc,
    double robot_x, double robot_y,
    size_t from_idx, size_t to_idx) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  rclcpp::Logger logger_{rclcpp::get_logger("RsPathController")};
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string global_frame_;

  nav_msgs::msg::Path global_plan_;
  size_t current_idx_{0};
  bool prev_rev_{false};
  double current_steering_angle_{0.0};

  // Parameters
  double desired_linear_vel_{1.0};
  double max_angular_vel_{1.0};
  double min_lookahead_dist_{0.5};
  double max_lookahead_dist_{2.0};
  double wheelbase_{1.2};
  double max_steering_angle_{0.7};
  int    sim_steps_{20};
  double sim_step_len_{0.1};
  double approach_dist_{3.0};
  double min_approach_vel_{0.3};
  double transform_tolerance_{0.1};

  double speed_limit_{1.0};
  bool speed_limit_is_percentage_{false};

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;
};

}  // namespace rs_path_controller

#endif  // TOOL_LINE_FOLLOWER_CONTROLLER__RS_PATH_CONTROLLER_HPP_
