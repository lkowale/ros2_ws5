// Swath headland turn planner for solbot5.
//
// Turn geometry (robot finishes swath at pose S, heading α; next swath starts at G, heading α):
//
//   Seg 1: forward left arc  90°, radius ρ
//          → robot moves to S1, heading α+90°
//
//   Seg 2: reverse right arc 60°, radius ρ
//          (reversing right = in map frame curves the robot's rear to the right,
//           net yaw change = -60° from S1 heading)
//          → robot at S2, heading α+30°
//
//   Seg 3: forward connecting arc (left or right, variable angle θ)
//          connecting S2 to P_lead (lead-in start), so that at P_lead heading = α
//          P_lead = G - lead_in * (cos α, sin α)
//
//   Seg 4: forward straight lead_in_ metres, ending at G heading α
//
// Connecting arc geometry (Seg 3):
//   We need to find a circle of radius ρ tangent to the robot at S2 (heading α+30°)
//   that also passes through P_lead with exit heading α.
//   The centre of the connecting arc lies on the perpendicular to S2's heading,
//   at distance ρ (left = +90°, right = -90°).
//   We try both left/right for the connecting arc and pick the one whose centre
//   is consistent with arriving at P_lead with heading α.
//   If neither analytical solution works (geometry doesn't close), we fall back
//   to outputting only segs 1+2+straight-to-lead-in.

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
  ps.pose.position.x = p.x;
  ps.pose.position.y = p.y;
  // Reverse segments: flip yaw by π so RPP drives backward.
  double yaw = reverse ? wrap(p.yaw + M_PI) : p.yaw;
  ps.pose.orientation.z = std::sin(yaw / 2.0);
  ps.pose.orientation.w = std::cos(yaw / 2.0);
  return ps;
}

SwathTurnPlanner::Pose2D SwathTurnPlanner::arcEnd(
  Pose2D p, double radius, double angle_rad, bool left, bool forward) const
{
  // Centre of curvature: perpendicular to heading, left=+90°, right=-90°.
  double sign = left ? 1.0 : -1.0;
  double cx = p.x - sign * radius * std::sin(p.yaw);
  double cy = p.y + sign * radius * std::cos(p.yaw);

  // Arc sweeps angle_rad in the turning direction.
  // Forward + left  → yaw increases (+angle_rad)
  // Forward + right → yaw decreases (-angle_rad)
  // Reverse + right → robot reverses, rear goes right → yaw increases in map frame
  // In all cases the effective yaw delta = forward*(left?+1:-1)*angle_rad
  double fwd = forward ? 1.0 : -1.0;
  double delta_yaw = fwd * sign * angle_rad;

  // New heading.
  double new_yaw = wrap(p.yaw + delta_yaw);

  // New position: robot rotates around (cx,cy) by delta_yaw.
  // When reversing the robot moves backward so the arc angle in the world is
  // opposite to the heading change — same formula works because we track
  // where the robot centre goes, not the heading.
  double sweep = delta_yaw;  // signed world angle swept around centre
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
    node_, name_ + ".min_turning_radius",     rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".lead_in_length",         rclcpp::ParameterValue(0.5));

  node_->get_parameter(name_ + ".min_turning_radius",      rho_);
  node_->get_parameter(name_ + ".interpolation_resolution", step_);
  node_->get_parameter(name_ + ".lead_in_length",          lead_in_);
  path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("/plan_turn", 1);

  RCLCPP_INFO(node_->get_logger(),
    "SwathTurnPlanner configured: rho=%.2f m  step=%.3f m  lead_in=%.2f m",
    rho_, step_, lead_in_);
}

void SwathTurnPlanner::cleanup()   {}
void SwathTurnPlanner::activate()  {}
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

  // Extract start pose.
  auto & sq = start.pose.orientation;
  double s_yaw = 2.0 * std::atan2(sq.z, sq.w);

  // Extract goal (swath start of next pass, heading = swath bearing α).
  auto & gq = goal.pose.orientation;
  double alpha = 2.0 * std::atan2(gq.z, gq.w);  // swath bearing
  double gx = goal.pose.position.x;
  double gy = goal.pose.position.y;

  Pose2D S = {start.pose.position.x, start.pose.position.y, s_yaw};

  // Lead-in start: 0.5m behind swath start along swath bearing.
  Pose2D P_lead = {
    gx - lead_in_ * std::cos(alpha),
    gy - lead_in_ * std::sin(alpha),
    alpha
  };

  // ── Segment 1: forward left arc 90° ──────────────────────────────────────
  Pose2D S1 = arcEnd(S, rho_, M_PI / 2.0, true, true);

  // ── Segment 2: reverse right arc 60° ─────────────────────────────────────
  // Reversing right: robot backs up, rear swings right → yaw decreases by 60°.
  Pose2D S2 = arcEnd(S1, rho_, M_PI / 3.0, false, false);

  // ── Segment 3: connecting arc S2 → P_lead ────────────────────────────────
  // We need a circle of radius ρ through S2 (heading S2.yaw) to P_lead (heading alpha).
  // Try left-turn arc, then right-turn arc.
  // For a left-turn arc from S2: centre C = S2 + ρ * perp_left(S2.yaw)
  //   perp_left(yaw) = (-sin yaw, cos yaw)
  // For the arc to exit at P_lead with heading alpha, the centre must also equal
  //   P_lead + ρ * perp_left(alpha)  (if left arc at exit too)
  //   P_lead + ρ * perp_right(alpha) (if right arc at exit)
  // We try all 4 combos (entry_left×exit_left, entry_left×exit_right,
  // entry_right×exit_left, entry_right×exit_right) and pick the one whose
  // centre is consistent (distance between the two candidate centres < 1mm).

  bool found = false;
  bool conn_left = true;
  double conn_angle = 0.0;
  Pose2D S3 = P_lead;

  for (int el = 0; el <= 1 && !found; ++el) {   // entry left/right
    double es = el ? 1.0 : -1.0;  // +1=left, -1=right
    double c_entry_x = S2.x + es * rho_ * (-std::sin(S2.yaw));
    double c_entry_y = S2.y + es * rho_ * ( std::cos(S2.yaw));

    for (int xl = 0; xl <= 1 && !found; ++xl) {  // exit left/right
      double xs = xl ? 1.0 : -1.0;
      double c_exit_x = P_lead.x + xs * rho_ * (-std::sin(alpha));
      double c_exit_y = P_lead.y + xs * rho_ * ( std::cos(alpha));

      double dist = std::hypot(c_entry_x - c_exit_x, c_entry_y - c_exit_y);
      if (dist > 1e-3) continue;   // centres don't agree → this combo doesn't work

      // Determine the arc angle swept around the centre.
      // Vector from centre to S2: angle = atan2(S2.y-cy, S2.x-cx)
      double cx = c_entry_x, cy = c_entry_y;
      double ang_start = std::atan2(S2.y - cy, S2.x - cx);
      double ang_end   = std::atan2(P_lead.y - cy, P_lead.x - cx);

      // For a left arc (CCW), angle increases; for right (CW), decreases.
      double sweep;
      if (el) {  // left arc
        sweep = wrap(ang_end - ang_start);
        if (sweep < 0) sweep += 2.0 * M_PI;
      } else {   // right arc
        sweep = wrap(ang_start - ang_end);
        if (sweep < 0) sweep += 2.0 * M_PI;
      }

      if (sweep < 1e-6 || sweep > 1.5 * M_PI) continue;  // skip degenerate or >270°

      conn_left  = (el == 1);
      conn_angle = sweep;
      S3 = P_lead;
      found = true;

      RCLCPP_INFO(node_->get_logger(),
        "SwathTurnPlanner: connecting arc %s %.1f°",
        conn_left ? "left" : "right", sweep * 180.0 / M_PI);
    }
  }

  if (!found) {
    // Fallback: straight line from S2 to P_lead, then lead-in.
    RCLCPP_WARN(node_->get_logger(),
      "SwathTurnPlanner: no connecting arc found, using straight fallback");
  }

  // ── Build path ────────────────────────────────────────────────────────────
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  poses.push_back(toPoseStamped(S, false));

  // Seg 1: forward left 90°
  appendArc(poses, S, rho_, M_PI / 2.0, true, true);

  // Seg 2: reverse right 60°
  appendArc(poses, S1, rho_, M_PI / 3.0, false, false);

  if (found) {
    // Seg 3: connecting arc
    appendArc(poses, S2, rho_, conn_angle, conn_left, true);
  } else {
    // Seg 3 fallback: straight to P_lead
    double dist = std::hypot(P_lead.x - S2.x, P_lead.y - S2.y);
    if (dist > step_) {
      double fb_yaw = std::atan2(P_lead.y - S2.y, P_lead.x - S2.x);
      appendStraight(poses, {S2.x, S2.y, fb_yaw}, dist, true);
    }
  }

  // Seg 4: lead-in straight
  appendStraight(poses, P_lead, lead_in_, true);

  path.poses = poses;
  path_pub_->publish(path);

  RCLCPP_INFO(node_->get_logger(),
    "SwathTurnPlanner: start=(%.2f,%.2f,%.1f°) goal=(%.2f,%.2f,%.1f°) "
    "waypts=%zu",
    S.x, S.y, s_yaw * 180.0 / M_PI,
    gx, gy, alpha * 180.0 / M_PI,
    poses.size());

  return path;
}

}  // namespace planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(planner::SwathTurnPlanner, nav2_core::GlobalPlanner)
