// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"
#include "geographic_msgs/msg/geo_point.hpp"

class GetGeoPointFromVector : public BT::SyncActionNode
{
public:
  GetGeoPointFromVector(
    const std::string & name,
    const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::vector<geographic_msgs::msg::GeoPoint>>("geo_points"),
      BT::InputPort<int>("index", 0, "Index of geo_point to extract"),
      BT::OutputPort<geographic_msgs::msg::GeoPoint>("geo_point"),
    };
  }

  BT::NodeStatus tick() override
  {
    std::vector<geographic_msgs::msg::GeoPoint> geo_points;
    int index = 0;

    if (!getInput("geo_points", geo_points)) {
      return BT::NodeStatus::FAILURE;
    }

    if (!getInput("index", index)) {
      return BT::NodeStatus::FAILURE;
    }

    if (index < 0 || index >= static_cast<int>(geo_points.size())) {
      return BT::NodeStatus::FAILURE;
    }

    setOutput("geo_point", geo_points[index]);
    return BT::NodeStatus::SUCCESS;
  }
};

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<GetGeoPointFromVector>("GetGeoPointFromVector");
}
