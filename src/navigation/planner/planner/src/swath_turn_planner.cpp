// Swath headland turn planner for solbot5.
//
// Configured via the `segment` parameter (1, 2, or 3).  Each instance plans
// only its own segment; Nav2 passes the robot's actual current pose as `start`
// so segments chain correctly after the previous one is followed.
//
// Turn geometry (swath bearing α, radius ρ):
//   Seg 1: forward left  arc 90°  from current pose
//   Seg 2: reverse right arc 60°  from current pose
//   Seg 3: forward connecting arc from current pose to lead-in start,
//           then straight lead-in to swath start
//
// The `goal` pose always encodes the NEXT swath start: position=swath_start, yaw=α.

#include "planner/swath_turn_planner.hpp"

#include <cmath>
#include <string>
#include <vector>

#include "nav2_util/node_utils.hpp"

namespace planner
{

// ── helpers ──────────────────────────────────────────────────────────────────

double SwathTurnPlanner::wrap(double a)
{
  while (a >  M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

geometry_msgs::msg::PoseStamped SwathTurnPlanner::toPoseStamped(Pose2D p, bool reverse) const
{
  geometry_msgs::msg::PoseStamped ps;
  ps.header.frame_id = global_frame_;
  double yaw = reverse ? wrap(p.yaw + M_PI) : p.yaw;
  ps.pose.position.x = p.x;
  ps.pose.position.y = p.y;
  ps.pose.orientation.z = std::sin(yaw / 2.0);
  ps.pose.orientation.w = std::cos(yaw / 2.0);
  return ps;
}

SwathTurnPlanner::Pose2D SwathTurnPlanner::arcEnd(
  Pose2D p, double radius, double angle_rad, bool left, bool forward) const
{
  double sign = left ? 1.0 : -1.0;
  double cx = p.x - sign * radius * std::sin(p.yaw);
  double cy = p.y + sign * radius * std::cos(p.yaw);

  double fwd = forward ? 1.0 : -1.0;
  double delta_yaw = fwd * sign * angle_rad;
  double new_yaw = wrap(p.yaw + delta_yaw);

  double sweep = delta_yaw;
  double new_x = cx + sign * radius * std::sin(p.yaw + sweep);
  double new_y = cy - sign * radius * std::cos(p.yaw + sweep);

  return {new_x, new_y, new_yaw};
}

SwathTurnPlanner::Pose2D SwathTurnPlanner::straightEnd(
  Pose2D p, double length, bool forward) const
{
  double fwd = forward ? 1.0 : -1.0;
  return {
    p.x + fwd * length * std::cos(p.yaw),
    p.y + fwd * length * std::sin(p.yaw),
    p.yaw
  };
}

void SwathTurnPlanner::appendArc(
  std::vector<geometry_msgs::msg::PoseStamped> & poses,
  Pose2D start, double radius, double angle_rad, bool left, bool forward) const
{
  double sign = left ? 1.0 : -1.0;
  double fwd  = forward ? 1.0 : -1.0;
  double cx   = start.x - sign * radius * std::sin(start.yaw);
  double cy   = start.y + sign * radius * std::cos(start.yaw);

  int n = std::max(2, static_cast<int>(std::ceil(radius * angle_rad / step_)));
  double d_angle = fwd * sign * angle_rad / n;

  for (int i = 0; i <= n; ++i) {
    double swept = d_angle * i;
    double yaw = wrap(start.yaw + swept);
    double x   = cx + sign * radius * std::sin(start.yaw + swept);
    double y   = cy - sign * radius * std::cos(start.yaw + swept);
    poses.push_back(toPoseStamped({x, y, yaw}, !forward));
  }
}

void SwathTurnPlanner::appendStraight(
  std::vector<geometry_msgs::msg::PoseStamped> & poses,
  Pose2D start, double length, bool forward) const
{
  int n = std::max(2, static_cast<int>(std::ceil(length / step_)));
  double fwd = forward ? 1.0 : -1.0;
  for (int i = 0; i <= n; ++i) {
    double t = static_cast<double>(i) / n;
    double x = start.x + fwd * t * length * std::cos(start.yaw);
    double y = start.y + fwd * t * length * std::sin(start.yaw);
    poses.push_back(toPoseStamped({x, y, start.yaw}, !forward));
  }
}

// ── plugin lifecycle ──────────────────────────────────────────────────────────

void SwathTurnPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> /*tf*/,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_         = parent.lock();
  name_         = name;
  global_frame_ = costmap_ros->getGlobalFrameID();

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".min_turning_radius",       rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".lead_in_length",           rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".lead_out_length",          rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".segment",                  rclcpp::ParameterValue(1));

  node_->get_parameter(name_ + ".min_turning_radius",       rho_);
  node_->get_parameter(name_ + ".interpolation_resolution", step_);
  node_->get_parameter(name_ + ".lead_in_length",           lead_in_);
  node_->get_parameter(name_ + ".lead_out_length",          lead_out_);
  node_->get_parameter(name_ + ".segment",                  segment_);

  path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("/plan_turn", 1);

  RCLCPP_INFO(node_->get_logger(),
    "SwathTurnPlanner[seg%d] configured: rho=%.2f step=%.3f lead_in=%.2f lead_out=%.2f",
    segment_, rho_, step_, lead_in_, lead_out_);
}

void SwathTurnPlanner::cleanup()    {}
void SwathTurnPlanner::activate()   {}
void SwathTurnPlanner::deactivate() {}

// ── createPlan ────────────────────────────────────────────────────────────────

nav_msgs::msg::Path SwathTurnPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> /*cancel_checker*/)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = global_frame_;
  path.header.stamp    = node_->now();

  auto & sq = start.pose.orientation;
  double s_yaw = 2.0 * std::atan2(sq.z, sq.w);
  Pose2D S = {start.pose.position.x, start.pose.position.y, s_yaw};

  auto & gq = goal.pose.orientation;
  double alpha = 2.0 * std::atan2(gq.z, gq.w);
  double gx = goal.pose.position.x;
  double gy = goal.pose.position.y;

  Pose2D P_lead = {
    gx - lead_in_ * std::cos(alpha),
    gy - lead_in_ * std::sin(alpha),
    alpha
  };

  std::vector<geometry_msgs::msg::PoseStamped> poses;

  if (segment_ == 1) {
    // Optional leadout: straight forward along swath before the arc
    Pose2D arc_start = S;
    if (lead_out_ > 0.0) {
      appendStraight(poses, S, lead_out_, true);
      arc_start = straightEnd(S, lead_out_, true);
    }
    // Forward left arc 90°
    appendArc(poses, arc_start, rho_, M_PI / 2.0, true, true);
    RCLCPP_INFO(node_->get_logger(),
      "SwathTurnPlanner seg1: leadout=%.2fm fwd-left 90° from (%.2f,%.2f,%.1f°)",
      lead_out_, S.x, S.y, s_yaw * 180.0 / M_PI);

  } else if (segment_ == 2) {
    // Reverse right arc 60°
    appendArc(poses, S, rho_, M_PI / 3.0, false, false);
    RCLCPP_INFO(node_->get_logger(),
      "SwathTurnPlanner seg2: rev-right 60° from (%.2f,%.2f,%.1f°)",
      S.x, S.y, s_yaw * 180.0 / M_PI);

  } else {
    // Seg 3: connecting arc from current pose to P_lead, then straight lead-in.
    // Find a circle of radius ρ tangent at S (heading s_yaw) that also arrives
    // at P_lead with heading alpha.  Try all 4 entry/exit left-right combos.
    bool found = false;
    bool conn_left = true;
    double conn_angle = 0.0;

    for (int el = 0; el <= 1 && !found; ++el) {
      double es = el ? 1.0 : -1.0;
      double c_entry_x = S.x + es * rho_ * (-std::sin(s_yaw));
      double c_entry_y = S.y + es * rho_ * ( std::cos(s_yaw));

      for (int xl = 0; xl <= 1 && !found; ++xl) {
        double xs = xl ? 1.0 : -1.0;
        double c_exit_x = P_lead.x + xs * rho_ * (-std::sin(alpha));
        double c_exit_y = P_lead.y + xs * rho_ * ( std::cos(alpha));

        double dist = std::hypot(c_entry_x - c_exit_x, c_entry_y - c_exit_y);
        if (dist > 1e-3) continue;

        double cx = c_entry_x, cy = c_entry_y;
        double ang_start = std::atan2(S.y - cy,      S.x - cx);
        double ang_end   = std::atan2(P_lead.y - cy, P_lead.x - cx);

        double sweep;
        if (el) {
          sweep = wrap(ang_end - ang_start);
          if (sweep < 0) sweep += 2.0 * M_PI;
        } else {
          sweep = wrap(ang_start - ang_end);
          if (sweep < 0) sweep += 2.0 * M_PI;
        }

        if (sweep < 1e-6 || sweep > 1.5 * M_PI) continue;

        conn_left  = (el == 1);
        conn_angle = sweep;
        found = true;

        RCLCPP_INFO(node_->get_logger(),
          "SwathTurnPlanner seg3: connecting arc %s %.1f° from (%.2f,%.2f,%.1f°)",
          conn_left ? "left" : "right", sweep * 180.0 / M_PI,
          S.x, S.y, s_yaw * 180.0 / M_PI);
      }
    }

    if (found) {
      appendArc(poses, S, rho_, conn_angle, conn_left, true);
    } else {
      RCLCPP_WARN(node_->get_logger(),
        "SwathTurnPlanner seg3: no connecting arc found, straight fallback");
      double dist = std::hypot(P_lead.x - S.x, P_lead.y - S.y);
      if (dist > step_) {
        double fb_yaw = std::atan2(P_lead.y - S.y, P_lead.x - S.x);
        appendStraight(poses, {S.x, S.y, fb_yaw}, dist, true);
      }
    }

    // Lead-in straight to swath start
    appendStraight(poses, P_lead, lead_in_, true);
  }

  path.poses = poses;
  path_pub_->publish(path);
  return path;
}

}  // namespace planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(planner::SwathTurnPlanner, nav2_core::GlobalPlanner)
