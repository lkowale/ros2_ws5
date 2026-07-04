// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#ifndef APPROACH_SWATH_NAV__APPROACH_SWATH_NAVIGATOR_HPP_
#define APPROACH_SWATH_NAV__APPROACH_SWATH_NAVIGATOR_HPP_

#include <string>
#include <memory>
#include <vector>

#include "nav2_core/behavior_tree_navigator.hpp"
#include "solbot5_msgs/action/run_approach_swath.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace approach_swath_nav
{

class ApproachSwathNavigator
  : public nav2_core::BehaviorTreeNavigator<solbot5_msgs::action::RunApproachSwath>
{
public:
  using ActionT = solbot5_msgs::action::RunApproachSwath;

  ApproachSwathNavigator() = default;
  ~ApproachSwathNavigator() override = default;

  bool configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
    std::shared_ptr<nav2_util::OdomSmoother> odom_smoother) override;

  std::string getName() override {return std::string("approach_swath");}

  std::string getDefaultBTFilepath(
    rclcpp_lifecycle::LifecycleNode::WeakPtr node) override;

protected:
  bool goalReceived(ActionT::Goal::ConstSharedPtr goal) override;

  void onLoop() override;

  void onPreempt(ActionT::Goal::ConstSharedPtr goal) override;

  void goalCompleted(
    typename ActionT::Result::SharedPtr result,
    const nav2_behavior_tree::BtStatus final_bt_status) override;

  std::string default_planner_id_;
  std::string default_controller_id_;
  std::string default_path_pub_topic_;
};

}  // namespace approach_swath_nav

#endif  // APPROACH_SWATH_NAV__APPROACH_SWATH_NAVIGATOR_HPP_
