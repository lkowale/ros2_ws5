// BT nodes for one_line_nav:
//   ReversePoses     — reverses a vector<PoseStamped> in place on the blackboard
//   GetPoseFromPoses — extracts one PoseStamped from a vector by index

#include <algorithm>
#include <memory>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

// ── ReversePoses ─────────────────────────────────────────────────────────────

class ReversePoses : public BT::SyncActionNode
{
public:
  ReversePoses(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::BidirectionalPort<std::vector<geometry_msgs::msg::PoseStamped>>("poses"),
    };
  }

  BT::NodeStatus tick() override
  {
    std::vector<geometry_msgs::msg::PoseStamped> poses;
    if (!getInput("poses", poses)) {return BT::NodeStatus::FAILURE;}
    std::reverse(poses.begin(), poses.end());
    // Flip orientations: each pose now points in the reverse swath direction
    for (auto & p : poses) {
      // Rotate yaw by π: q_new = q * [0,0,1,0] (multiply by 180° z rotation)
      double qz = p.pose.orientation.z;
      double qw = p.pose.orientation.w;
      p.pose.orientation.z =  qw;   // sin((yaw+π)/2) = cos(yaw/2)
      p.pose.orientation.w = -qz;   // cos((yaw+π)/2) = -sin(yaw/2)
    }
    setOutput("poses", poses);
    return BT::NodeStatus::SUCCESS;
  }
};

// ── GetPoseFromPoses ──────────────────────────────────────────────────────────

class GetPoseFromPoses : public BT::SyncActionNode
{
public:
  GetPoseFromPoses(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("poses"),
      BT::InputPort<int>("index", 0, "Index to extract"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose"),
    };
  }

  BT::NodeStatus tick() override
  {
    std::vector<geometry_msgs::msg::PoseStamped> poses;
    int index = 0;
    if (!getInput("poses", poses) || !getInput("index", index)) {
      return BT::NodeStatus::FAILURE;
    }
    if (index < 0 || index >= static_cast<int>(poses.size())) {
      return BT::NodeStatus::FAILURE;
    }
    setOutput("pose", poses[index]);
    return BT::NodeStatus::SUCCESS;
  }
};

// ── Registration ─────────────────────────────────────────────────────────────

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<ReversePoses>("ReversePoses");
  factory.registerNodeType<GetPoseFromPoses>("GetPoseFromPoses");
}
