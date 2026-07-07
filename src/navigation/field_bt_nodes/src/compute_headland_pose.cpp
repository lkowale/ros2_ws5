// ComputeHeadlandPose BT node.
//
// Given the current swath end pose (map_points[1]) and turn_direction ("left"/"right"),
// computes a headland intermediate pose reached by driving a 90° arc of radius `rho`
// away from the swath into the headland.
//
// Output pose: position = arc endpoint, yaw = tangent at arc end (perpendicular to swath).
// The RS planner then takes over for the second leg to the next swath start.

#include <cmath>
#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

static double wrap(double a)
{
  while (a >  M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

class ComputeHeadlandPose : public BT::SyncActionNode
{
public:
  ComputeHeadlandPose(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("map_points",
        "Current swath start/end poses; [1] is the swath end (turn start)"),
      BT::InputPort<std::string>("turn_direction", "left or right"),
      BT::InputPort<double>("rho", 3.0, "Turning radius [m]"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("headland_pose",
        "Intermediate pose in headland after 90° arc"),
    };
  }

  BT::NodeStatus tick() override
  {
    std::vector<geometry_msgs::msg::PoseStamped> map_points;
    std::string turn_direction;
    double rho = 3.0;

    if (!getInput("map_points", map_points) || map_points.size() < 2) {
      RCLCPP_ERROR(rclcpp::get_logger("ComputeHeadlandPose"),
        "map_points missing or too short");
      return BT::NodeStatus::FAILURE;
    }
    if (!getInput("turn_direction", turn_direction)) {
      return BT::NodeStatus::FAILURE;
    }
    getInput("rho", rho);

    if (turn_direction != "left" && turn_direction != "right") {
      RCLCPP_WARN(rclcpp::get_logger("ComputeHeadlandPose"),
        "turn_direction '%s' is not left/right — skipping", turn_direction.c_str());
      return BT::NodeStatus::FAILURE;
    }

    // Swath end pose
    const auto & end = map_points[1];
    const double ex   = end.pose.position.x;
    const double ey   = end.pose.position.y;
    const double eyaw = 2.0 * std::atan2(end.pose.orientation.z, end.pose.orientation.w);

    // Arc: 90° in the turn direction from swath end heading.
    // left  → arc centre is to the left  of heading (+90° cross), arc sweeps CCW
    // right → arc centre is to the right of heading (-90° cross), arc sweeps CW
    const double sign = (turn_direction == "left") ? 1.0 : -1.0;

    // Centre of turning circle
    const double cx = ex - sign * rho * std::sin(eyaw);
    const double cy = ey + sign * rho * std::cos(eyaw);

    // Arc endpoint after 90° sweep
    const double arc_angle = sign * M_PI / 2.0;
    const double px = cx + sign * rho * std::sin(eyaw + arc_angle);
    const double py = cy - sign * rho * std::cos(eyaw + arc_angle);
    const double pyaw = wrap(eyaw + arc_angle);

    geometry_msgs::msg::PoseStamped out;
    out.header = end.header;
    out.pose.position.x  = px;
    out.pose.position.y  = py;
    out.pose.position.z  = 0.0;
    out.pose.orientation.x = 0.0;
    out.pose.orientation.y = 0.0;
    out.pose.orientation.z = std::sin(pyaw / 2.0);
    out.pose.orientation.w = std::cos(pyaw / 2.0);

    setOutput("headland_pose", out);

    RCLCPP_INFO(rclcpp::get_logger("ComputeHeadlandPose"),
      "turn=%s rho=%.2f  swath_end=(%.2f,%.2f,%.1f°) → headland=(%.2f,%.2f,%.1f°)",
      turn_direction.c_str(), rho,
      ex, ey, eyaw * 180.0 / M_PI,
      px, py, pyaw * 180.0 / M_PI);

    return BT::NodeStatus::SUCCESS;
  }
};

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<ComputeHeadlandPose>("ComputeHeadlandPose");
}
