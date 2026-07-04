// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#ifndef GEOJSON_LINE_NAV__GEOJSON_LINE_NAVIGATOR_HPP_
#define GEOJSON_LINE_NAV__GEOJSON_LINE_NAVIGATOR_HPP_

#include <string>
#include <memory>
#include <vector>

#include "nav2_core/behavior_tree_navigator.hpp"
#include "solbot5_msgs/action/run_geo_json_line.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "geographic_msgs/msg/geo_point.hpp"

namespace geojson_line_nav
{

class GeoJsonLineNavigator
  : public nav2_core::BehaviorTreeNavigator<solbot5_msgs::action::RunGeoJsonLine>
{
public:
  using ActionT = solbot5_msgs::action::RunGeoJsonLine;

  GeoJsonLineNavigator() = default;
  ~GeoJsonLineNavigator() override = default;

  bool configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
    std::shared_ptr<nav2_util::OdomSmoother> odom_smoother) override;

  std::string getName() override {return std::string("run_geojson_line");}

  std::string getDefaultBTFilepath(
    rclcpp_lifecycle::LifecycleNode::WeakPtr node) override;

protected:
  bool goalReceived(ActionT::Goal::ConstSharedPtr goal) override;

  void onLoop() override;

  void onPreempt(ActionT::Goal::ConstSharedPtr goal) override;

  void goalCompleted(
    typename ActionT::Result::SharedPtr result,
    const nav2_behavior_tree::BtStatus final_bt_status) override;

  bool loadGeoJsonLine(
    const std::string & file_path,
    std::vector<geographic_msgs::msg::GeoPoint> & geo_points);
};

}  // namespace geojson_line_nav

#endif  // GEOJSON_LINE_NAV__GEOJSON_LINE_NAVIGATOR_HPP_
