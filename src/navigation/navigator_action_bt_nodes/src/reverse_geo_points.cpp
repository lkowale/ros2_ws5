// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#include <memory>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"
#include "geographic_msgs/msg/geo_point.hpp"

class ReverseGeoPoints : public BT::SyncActionNode
{
public:
  ReverseGeoPoints(
    const std::string & name,
    const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::BidirectionalPort<std::vector<geographic_msgs::msg::GeoPoint>>("geo_points"),
    };
  }

  BT::NodeStatus tick() override
  {
    std::vector<geographic_msgs::msg::GeoPoint> geo_points;

    if (!getInput("geo_points", geo_points)) {
      return BT::NodeStatus::FAILURE;
    }

    std::reverse(geo_points.begin(), geo_points.end());
    setOutput("geo_points", geo_points);

    return BT::NodeStatus::SUCCESS;
  }
};

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<ReverseGeoPoints>("ReverseGeoPoints");
}
