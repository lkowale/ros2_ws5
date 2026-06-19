// PopNextGoal — BT action node for rs_test_nav.
//
// Reads from blackboard:
//   rs_test_goals  : vector<PoseStamped>
//   rs_test_labels : vector<string>
//   rs_test_index  : int               — next goal index (0-based)
//   rs_robot_pose  : PoseStamped       — current robot pose from /odom
//
// Returns FAILURE when list is exhausted (signals KeepRunningUntilFailure to stop).
// Otherwise sets {goal} and {rs_test_label} output ports and returns SUCCESS.

#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

using geometry_msgs::msg::PoseStamped;

class PopNextGoal : public BT::SyncActionNode
{
public:
  PopNextGoal(const std::string & name, const BT::NodeConfiguration & cfg)
  : BT::SyncActionNode(name, cfg)
  {
    node_ = rclcpp::Node::make_shared("pop_next_goal_bt");
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<PoseStamped>("goal"),
      BT::OutputPort<std::string>("rs_test_label"),
    };
  }

  BT::NodeStatus tick() override
  {
    std::vector<PoseStamped> goals;
    int idx = 0;

    if (!config().blackboard->get("rs_test_goals", goals) ||
        !config().blackboard->get("rs_test_index", idx))
    {
      RCLCPP_ERROR(node_->get_logger(), "PopNextGoal: blackboard missing rs_test_goals/index");
      return BT::NodeStatus::FAILURE;
    }

    if (idx < 0 || static_cast<size_t>(idx) >= goals.size()) {
      RCLCPP_INFO(node_->get_logger(), "PopNextGoal: all %zu goals done", goals.size());
      return BT::NodeStatus::FAILURE;
    }

    std::vector<std::string> labels;
    config().blackboard->get("rs_test_labels", labels);

    const PoseStamped & tgt = goals[static_cast<size_t>(idx)];
    std::string label = static_cast<size_t>(idx) < labels.size()
      ? labels[static_cast<size_t>(idx)]
      : ("goal_" + std::to_string(idx));

    config().blackboard->set("rs_test_index", idx + 1);

    setOutput("goal", tgt);
    setOutput("rs_test_label", label);

    PoseStamped start;
    start.header.frame_id = "map";
    start.pose.orientation.w = 1.0;
    config().blackboard->get("rs_robot_pose", start);

    RCLCPP_INFO(node_->get_logger(),
      "PopNextGoal [%d/%zu] '%s'  start=(%.2f,%.2f) → goal=(%.2f,%.2f)",
      idx + 1, goals.size(), label.c_str(),
      start.pose.position.x, start.pose.position.y,
      tgt.pose.position.x,   tgt.pose.position.y);

    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
};

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<PopNextGoal>("PopNextGoal");
}
