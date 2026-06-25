#ifndef PLANNER__REEDS_SHEPP_PLANNER_HPP_
#define PLANNER__REEDS_SHEPP_PLANNER_HPP_

#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"

namespace planner
{

// Nav2 global planner plugin: Reeds-Shepp path from start pose to goal pose.
//
// Uses OMPL's ReedsSheppStateSpace (all 48 word families) for correct path
// selection, then samples waypoints at `interpolation_resolution` metre spacing.
// Reverse segments have yaw flipped by π so RPP drives backward.
//
// Parameters (set under the plugin name in nav2_params.yaml):
//   min_turning_radius   [m]   default 1.5  — Ackermann minimum radius
//   interpolation_resolution [m] default 0.05 — waypoint spacing
class ReedsSheppPlanner : public nav2_core::GlobalPlanner
{
public:
  ReedsSheppPlanner() = default;
  ~ReedsSheppPlanner() = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

private:
  nav2_util::LifecycleNode::SharedPtr node_;
  std::string global_frame_;
  std::string name_;
  double rho_;   // min turning radius
  double step_;  // interpolation resolution
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr fwd_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr rev_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr constraint_sub_;
  std::mutex constraint_mutex_;
  std::string constraint_raw_;
  std::string turn_side_constraint_;  // "left", "right", or ""
  double      swath_yaw_constraint_{0.0};
  bool        force_forward_first_{false};
};

}  // namespace planner

#endif  // PLANNER__REEDS_SHEPP_PLANNER_HPP_
