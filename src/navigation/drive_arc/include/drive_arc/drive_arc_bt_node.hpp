#ifndef DRIVE_ARC__DRIVE_ARC_BT_NODE_HPP_
#define DRIVE_ARC__DRIVE_ARC_BT_NODE_HPP_

#include <string>
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "solbot5_msgs/action/drive_arc.hpp"

namespace drive_arc
{

class DriveArcAction
  : public nav2_behavior_tree::BtActionNode<solbot5_msgs::action::DriveArc>
{
  using Action = solbot5_msgs::action::DriveArc;

public:
  DriveArcAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BtActionNode<Action>(xml_tag_name, action_name, conf) {}

  void on_tick() override
  {
    getInput("radius",          goal_.radius);
    getInput("angle",           goal_.angle);
    getInput("speed",           goal_.speed);
    getInput("time_allowance",  goal_.time_allowance);
  }

  BT::NodeStatus on_success()   override { return BT::NodeStatus::SUCCESS; }
  BT::NodeStatus on_aborted()   override { return BT::NodeStatus::FAILURE; }
  BT::NodeStatus on_cancelled() override { return BT::NodeStatus::SUCCESS; }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<double>("radius",         1.5,  "Turning radius [m]"),
      BT::InputPort<double>("angle",          1.57, "Arc angle [rad] (+ left, - right)"),
      BT::InputPort<double>("speed",          0.4,  "Linear speed [m/s] (- = reverse)"),
      BT::InputPort<double>("time_allowance", 30.0, "Max time [s]"),
      BT::OutputPort<Action::Result::_error_code_type>("error_code_id", "Error code"),
    });
  }
};

}  // namespace drive_arc

#endif  // DRIVE_ARC__DRIVE_ARC_BT_NODE_HPP_
