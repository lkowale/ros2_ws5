// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#include <chrono>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "field_nav/field_line.hpp"

using field_nav::FieldLine;

class SetSwathLateralOffset : public BT::SyncActionNode
{
public:
  SetSwathLateralOffset(
    const std::string & name,
    const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
    auto node = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    logger_ = node->get_logger();

    // Use a dedicated callback group + executor (Nav2 BtServiceNode pattern)
    // so spin_until_future_complete doesn't conflict with the BT executor.
    callback_group_ = node->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
    callback_group_executor_.add_callback_group(
      callback_group_, node->get_node_base_interface());

    client_ = node->create_client<rcl_interfaces::srv::SetParameters>(
      "/controller_server/set_parameters",
      rmw_qos_profile_services_default,
      callback_group_);
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::vector<FieldLine>>("field_lines", "All field lines with lateral offsets"),
      BT::InputPort<int32_t>("current_line_index", "Current swath index"),
      BT::InputPort<std::string>("controller_plugin", "Controller plugin name",
        "GpsLineFollower"),
    };
  }

  BT::NodeStatus tick() override
  {
    std::vector<FieldLine> field_lines;
    int32_t line_index = 0;
    std::string plugin;

    if (!getInput("field_lines", field_lines) ||
        !getInput("current_line_index", line_index))
    {
      RCLCPP_ERROR(logger_, "SetSwathLateralOffset: missing inputs");
      return BT::NodeStatus::FAILURE;
    }
    getInput("controller_plugin", plugin);

    double offset_m = 0.0;
    if (line_index >= 0 && static_cast<size_t>(line_index) < field_lines.size()) {
      offset_m = field_lines[line_index].lateral_offset_m;
    }

    if (!client_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_ERROR(logger_,
        "SetSwathLateralOffset: /controller_server/set_parameters not available");
      return BT::NodeStatus::FAILURE;
    }

    auto req = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    rcl_interfaces::msg::Parameter param;
    param.name = plugin + ".antenna_lateral_offset";
    param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    param.value.double_value = offset_m;
    req->parameters.push_back(param);

    auto future = client_->async_send_request(req);
    if (callback_group_executor_.spin_until_future_complete(
        future, std::chrono::seconds(2)) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(logger_, "SetSwathLateralOffset: set_parameters call timed out");
      return BT::NodeStatus::FAILURE;
    }

    if (!future.get()->results.empty() && !future.get()->results[0].successful) {
      RCLCPP_ERROR(logger_, "SetSwathLateralOffset: parameter rejected: %s",
        future.get()->results[0].reason.c_str());
      return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(logger_,
      "Swath %d lateral offset set to %.4f m", line_index, offset_m);
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr client_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Logger logger_{rclcpp::get_logger("SetSwathLateralOffset")};
};

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<SetSwathLateralOffset>("SetSwathLateralOffset");
}
