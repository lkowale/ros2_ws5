#ifndef UNIVERSAL_RS_PLANNER__UNIVERSAL_RS_PLANNER_HPP_
#define UNIVERSAL_RS_PLANNER__UNIVERSAL_RS_PLANNER_HPP_

#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace universal_rs_planner
{

// Nav2 global planner plugin: shortest Reeds-Shepp path, no constraints.
//
// Uses OMPL's ReedsSheppStateSpace (all 48 word families).
// Reverse segments have yaw flipped by π so the controller drives backward.
//
// Parameters:
//   min_turning_radius        [m]    default 1.5
//   interpolation_resolution  [m]    default 0.05
class UniversalReedsSheppPlanner : public nav2_core::GlobalPlanner
{
public:
  UniversalReedsSheppPlanner() = default;
  ~UniversalReedsSheppPlanner() = default;

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
};

}  // namespace universal_rs_planner

#endif  // UNIVERSAL_RS_PLANNER__UNIVERSAL_RS_PLANNER_HPP_
