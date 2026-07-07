// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#ifndef FIELD_NAV__FIELD_NAVIGATOR_HPP_
#define FIELD_NAV__FIELD_NAVIGATOR_HPP_

#include <string>
#include <memory>
#include <vector>

#include "nav2_core/behavior_tree_navigator.hpp"
#include "solbot5_msgs/action/run_field.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "field_nav/field_line.hpp"

namespace field_nav
{

class FieldNavigator
  : public nav2_core::BehaviorTreeNavigator<solbot5_msgs::action::RunField>
{
public:
  using ActionT = solbot5_msgs::action::RunField;

  FieldNavigator() = default;
  ~FieldNavigator() override = default;

  bool configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
    std::shared_ptr<nav2_util::OdomSmoother> odom_smoother) override;

  std::string getName() override {return std::string("run_field");}

  std::string getDefaultBTFilepath(
    rclcpp_lifecycle::LifecycleNode::WeakPtr node) override;

protected:
  bool goalReceived(ActionT::Goal::ConstSharedPtr goal) override;

  void onLoop() override;

  void onPreempt(ActionT::Goal::ConstSharedPtr goal) override;

  void goalCompleted(
    typename ActionT::Result::SharedPtr result,
    const nav2_behavior_tree::BtStatus final_bt_status) override;

  bool loadDirectedLinesFromFile(
    const std::string & field_name,
    std::vector<FieldLine> & lines);

  bool loadProgressFromFile(
    const std::string & field_name,
    int32_t & current_line_index,
    std::string & stage);

  bool saveProgressToFile(
    const std::string & field_name,
    int32_t current_line_index,
    const std::string & stage);

  void loadSwathOffsets(
    const std::string & field_name,
    std::vector<FieldLine> & lines);

  std::string fields_directory_;
  std::vector<FieldLine> field_lines_;
  int32_t total_lines_;
};

}  // namespace field_nav

#endif  // FIELD_NAV__FIELD_NAVIGATOR_HPP_
