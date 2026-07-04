// BT nodes for one_line_nav:
//   ConvertGeoPoints            — calls fromLL to convert geo_points → map_points (PoseStamped)
//   ReverseGeoPoints            — reverses geo_points in place on the blackboard
//   GetGeoPointFromVector       — extracts one GeoPoint from geo_points by index
//   ReversePoses                — reverses map_points vector, flipping each orientation 180°
//   GetPoseFromPoses            — extracts one PoseStamped from map_points by index
//   SetRsPlannerConstraints     — publishes turn_side to /rs_planner_constraints (latched)
//   ClearRsPlannerConstraints   — clears the constraint (publishes empty string)

#include <cmath>
#include <algorithm>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "robot_localization/srv/from_ll.hpp"
#include "geographic_msgs/msg/geo_point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"

// ── ConvertGeoPoints ──────────────────────────────────────────────────────────

class ConvertGeoPoints : public BT::SyncActionNode
{
public:
  ConvertGeoPoints(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
    node_ = rclcpp::Node::make_shared("convert_geo_points_bt_node");
    client_ = node_->create_client<robot_localization::srv::FromLL>("fromLL");
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::vector<geographic_msgs::msg::GeoPoint>>("geo_points"),
      BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>("map_points"),
    };
  }

  BT::NodeStatus tick() override
  {
    std::vector<geographic_msgs::msg::GeoPoint> geo_points;
    if (!getInput("geo_points", geo_points)) {
      RCLCPP_ERROR(node_->get_logger(), "Missing input [geo_points]");
      return BT::NodeStatus::FAILURE;
    }

    if (!client_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_ERROR(node_->get_logger(), "Service fromLL not available");
      return BT::NodeStatus::FAILURE;
    }

    std::vector<geometry_msgs::msg::PoseStamped> map_points;

    for (auto & gp : geo_points) {
      auto req = std::make_shared<robot_localization::srv::FromLL::Request>();
      req->ll_point = gp;

      auto future = client_->async_send_request(req);
      auto result = rclcpp::spin_until_future_complete(
        node_, future, std::chrono::seconds(2));

      if (result != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call fromLL");
        return BT::NodeStatus::FAILURE;
      }

      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = node_->now();
      pose.header.frame_id = "map";
      pose.pose.position = future.get()->map_point;
      pose.pose.orientation.w = 1.0;
      map_points.push_back(pose);

      RCLCPP_INFO(node_->get_logger(),
        "Converted (%.6f, %.6f) -> map (%.3f, %.3f)",
        gp.latitude, gp.longitude,
        pose.pose.position.x, pose.pose.position.y);
    }

    // Set orientations to the swath heading (start → end)
    if (map_points.size() >= 2) {
      double dx = map_points[1].pose.position.x - map_points[0].pose.position.x;
      double dy = map_points[1].pose.position.y - map_points[0].pose.position.y;
      double yaw = std::atan2(dy, dx);
      geometry_msgs::msg::Quaternion q;
      q.x = 0.0; q.y = 0.0;
      q.z = std::sin(yaw / 2.0);
      q.w = std::cos(yaw / 2.0);
      for (auto & p : map_points) {p.pose.orientation = q;}
    }

    setOutput("map_points", map_points);
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<robot_localization::srv::FromLL>::SharedPtr client_;
};

// ── ReverseGeoPoints ──────────────────────────────────────────────────────────

class ReverseGeoPoints : public BT::SyncActionNode
{
public:
  ReverseGeoPoints(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::BidirectionalPort<std::vector<geographic_msgs::msg::GeoPoint>>("geo_points"),
    };
  }

  BT::NodeStatus tick() override
  {
    std::vector<geographic_msgs::msg::GeoPoint> geo_points;
    if (!getInput("geo_points", geo_points)) {return BT::NodeStatus::FAILURE;}
    std::reverse(geo_points.begin(), geo_points.end());
    setOutput("geo_points", geo_points);
    return BT::NodeStatus::SUCCESS;
  }
};

// ── GetGeoPointFromVector ─────────────────────────────────────────────────────

class GetGeoPointFromVector : public BT::SyncActionNode
{
public:
  GetGeoPointFromVector(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::vector<geographic_msgs::msg::GeoPoint>>("geo_points"),
      BT::InputPort<int>("index", 0, "Index to extract"),
      BT::OutputPort<geographic_msgs::msg::GeoPoint>("geo_point"),
    };
  }

  BT::NodeStatus tick() override
  {
    std::vector<geographic_msgs::msg::GeoPoint> geo_points;
    int index = 0;
    if (!getInput("geo_points", geo_points) || !getInput("index", index)) {
      return BT::NodeStatus::FAILURE;
    }
    if (index < 0 || index >= static_cast<int>(geo_points.size())) {
      return BT::NodeStatus::FAILURE;
    }
    setOutput("geo_point", geo_points[index]);
    return BT::NodeStatus::SUCCESS;
  }
};

// ── ReversePoses ──────────────────────────────────────────────────────────────

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
    for (auto & p : poses) {
      double qz = p.pose.orientation.z;
      double qw = p.pose.orientation.w;
      p.pose.orientation.z =  qw;
      p.pose.orientation.w = -qz;
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

// ── SetRsPlannerConstraints ───────────────────────────────────────────────────
// Reads turn_side and map_points from the blackboard. Publishes a constraint
// message to /rs_planner_constraints (latched) containing:
//   "<turn_side>,<swath_yaw_rad>,forward[,<bnd_x>,<bnd_y>]"
// The optional bnd_x/bnd_y are the map-frame coordinates of the headland
// boundary point on the active (turn) side. The planner projects all path
// waypoints onto the swath axis and retries with a deeper goal if any waypoint
// crosses the boundary back into the inland zone.

class SetRsPlannerConstraints : public BT::SyncActionNode
{
public:
  SetRsPlannerConstraints(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
    node_ = rclcpp::Node::make_shared("set_rs_constraints_bt_node");
    auto qos = rclcpp::QoS(1).transient_local();
    pub_ = node_->create_publisher<std_msgs::msg::String>("/rs_planner_constraints", qos);
    fromll_client_ = node_->create_client<robot_localization::srv::FromLL>("/fromLL");
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("turn_side", "{turn_side}", "left or right"),
      BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("map_points", "{map_points}",
        "swath start/end poses to derive swath heading"),
      BT::InputPort<geographic_msgs::msg::GeoPoint>("headland_sw", "{headland_sw}",
        "SW headland boundary point (optional)"),
      BT::InputPort<geographic_msgs::msg::GeoPoint>("headland_ne", "{headland_ne}",
        "NE headland boundary point (optional)"),
    };
  }

  BT::NodeStatus tick() override
  {
    std::string turn_side;
    getInput("turn_side", turn_side);
    if (turn_side != "left" && turn_side != "right") {
      RCLCPP_WARN(node_->get_logger(),
        "SetRsPlannerConstraints: invalid turn_side '%s'", turn_side.c_str());
      return BT::NodeStatus::SUCCESS;
    }

    // Derive swath axis from map_points (start→end direction), normalize to [0,π)
    double swath_yaw = 0.0;
    std::vector<geometry_msgs::msg::PoseStamped> map_points;
    if (getInput("map_points", map_points) && map_points.size() >= 2) {
      const auto & p0 = map_points[0].pose.position;
      const auto & p1 = map_points[1].pose.position;
      double yaw = std::atan2(p1.y - p0.y, p1.x - p0.x);
      if (yaw < 0.0) yaw += M_PI;
      if (yaw >= M_PI) yaw -= M_PI;
      swath_yaw = yaw;
    } else {
      RCLCPP_WARN(node_->get_logger(),
        "SetRsPlannerConstraints: map_points unavailable, using swath_yaw=0");
    }

    // Try to convert the active headland boundary GeoPoint to map frame
    std::string bnd_str;
    geographic_msgs::msg::GeoPoint bnd_geo;
    bool have_bnd = false;
    // "right" turn → NE headland (even swath end), "left" → SW headland (odd swath end)
    if (turn_side == "right") {
      have_bnd = getInput("headland_ne", bnd_geo).has_value();
    } else {
      have_bnd = getInput("headland_sw", bnd_geo).has_value();
    }

    if (have_bnd && fromll_client_->wait_for_service(std::chrono::milliseconds(100))) {
      auto req = std::make_shared<robot_localization::srv::FromLL::Request>();
      req->ll_point.latitude  = bnd_geo.latitude;
      req->ll_point.longitude = bnd_geo.longitude;
      req->ll_point.altitude  = 0.0;
      auto future = fromll_client_->async_send_request(req);
      if (rclcpp::spin_until_future_complete(
            node_, future, std::chrono::milliseconds(500)) ==
          rclcpp::FutureReturnCode::SUCCESS)
      {
        const auto & mp = future.get()->map_point;
        char tmp[64];
        std::snprintf(tmp, sizeof(tmp), ",%.4f,%.4f", mp.x, mp.y);
        bnd_str = tmp;
      } else {
        RCLCPP_WARN(node_->get_logger(), "SetRsPlannerConstraints: fromLL timeout for boundary");
      }
    }

    char buf[128];
    std::snprintf(buf, sizeof(buf), "%s,%.6f,forward%s",
      turn_side.c_str(), swath_yaw, bnd_str.c_str());
    std_msgs::msg::String msg;
    msg.data = buf;
    pub_->publish(msg);
    RCLCPP_INFO(node_->get_logger(),
      "RS constraint: turn_side=%s swath_yaw=%.1f° boundary=%s",
      turn_side.c_str(), swath_yaw * 180.0 / M_PI,
      bnd_str.empty() ? "none" : bnd_str.c_str());
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Client<robot_localization::srv::FromLL>::SharedPtr fromll_client_;
};

// ── ClearRsPlannerConstraints ─────────────────────────────────────────────────
// Publishes empty string — RS planner reverts to unconstrained OMPL path.

class ClearRsPlannerConstraints : public BT::SyncActionNode
{
public:
  ClearRsPlannerConstraints(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
    node_ = rclcpp::Node::make_shared("clear_rs_constraints_bt_node");
    auto qos = rclcpp::QoS(1).transient_local();
    pub_ = node_->create_publisher<std_msgs::msg::String>("/rs_planner_constraints", qos);
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    std_msgs::msg::String msg;
    msg.data = "";
    pub_->publish(msg);
    RCLCPP_INFO(node_->get_logger(), "RS constraints cleared");
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

// ── Registration ─────────────────────────────────────────────────────────────

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<ConvertGeoPoints>("ConvertGeoPoints");
  factory.registerNodeType<ReverseGeoPoints>("ReverseGeoPoints");
  factory.registerNodeType<GetGeoPointFromVector>("GetGeoPointFromVector");
  factory.registerNodeType<ReversePoses>("ReversePoses");
  factory.registerNodeType<GetPoseFromPoses>("GetPoseFromPoses");
  factory.registerNodeType<SetRsPlannerConstraints>("SetRsPlannerConstraints");
  factory.registerNodeType<ClearRsPlannerConstraints>("ClearRsPlannerConstraints");
}
