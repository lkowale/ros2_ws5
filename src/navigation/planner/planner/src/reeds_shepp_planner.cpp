// Reeds-Shepp planner plugin for Nav2.
//
// Uses OMPL's ReedsSheppStateSpace to find the shortest valid RS path (all 48
// word families checked, correct turning-radius constraint enforced).
// The path is sampled at `step_` metre intervals into PoseStamped waypoints.
// On reverse segments the yaw is rotated by π so that RPP drives backward.
//
// Constraint support: subscribes to /rs_planner_constraints (std_msgs/String,
// latched). When the message contains "left" or "right", the planner enforces
// that the turn arcs are on that side (relative to the robot's forward heading
// at the start). Used for headland turns to prevent the RS path from cutting
// into the crop side of the field.
//
// Implementation: OMPL only returns the globally shortest path. To enforce a
// turn-side constraint we use RS path lateral symmetry: mirror the goal across
// the robot's forward axis (negate the lateral component in robot frame),
// plan the mirrored problem, then mirror the resulting waypoints back. The
// mirrored problem produces a path that curves to the opposite lateral side —
// so if OMPL's unconstrained best violates the constraint, we plan the mirror
// and the mirrored result will curve to the correct side.
//
// Mirror transform for a point (x,y) in world frame, given robot at (rx,ry,ryaw):
//   local  = R(-ryaw) * (world - robot)
//   mirror = (local.x, -local.y)          ← negate lateral
//   world' = R(+ryaw) * mirror + robot
// Yaw is mirrored as:  yaw' = 2*ryaw - yaw  (reflect across robot heading axis)

#include "planner/reeds_shepp_planner.hpp"

#include <chrono>
#include <cmath>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "nav2_util/node_utils.hpp"
#include "ompl/base/spaces/ReedsSheppStateSpace.h"
#include "ompl/base/spaces/SE2StateSpace.h"
#include "std_msgs/msg/string.hpp"

namespace planner
{

// ─── helpers ─────────────────────────────────────────────────────────────────

static double wrap(double a)
{
  while (a >  M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

static geometry_msgs::msg::Quaternion yawToQuat(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0; q.y = 0.0;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
  return q;
}

static double quatToYaw(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

// Mirror a world point across the robot's forward (heading) axis.
static void mirrorPoint(
  double rx, double ry, double ryaw,
  double wx, double wy,
  double & mx, double & my)
{
  double lx =  std::cos(ryaw) * (wx - rx) + std::sin(ryaw) * (wy - ry);
  double ly = -std::sin(ryaw) * (wx - rx) + std::cos(ryaw) * (wy - ry);
  ly = -ly;  // negate lateral
  mx = std::cos(ryaw) * lx - std::sin(ryaw) * ly + rx;
  my = std::sin(ryaw) * lx + std::cos(ryaw) * ly + ry;
}

// Mirror a yaw angle across the robot's forward axis.
static double mirrorYaw(double ryaw, double yaw)
{
  return wrap(2.0 * ryaw - yaw);
}

// ─── Pose propagation ────────────────────────────────────────────────────────

static void stepPose(
  ompl::base::ReedsSheppStateSpace::ReedsSheppPathSegmentType type,
  double ds,
  double rho,
  double & cx, double & cy, double & cyaw)
{
  using T = ompl::base::ReedsSheppStateSpace::ReedsSheppPathSegmentType;
  switch (type) {
    case T::RS_STRAIGHT:
      cx   += ds * std::cos(cyaw);
      cy   += ds * std::sin(cyaw);
      break;
    case T::RS_LEFT: {
      const double dphi = ds / rho;
      cx   += rho * (std::sin(cyaw + dphi) - std::sin(cyaw));
      cy   += rho * (-std::cos(cyaw + dphi) + std::cos(cyaw));
      cyaw  = wrap(cyaw + dphi);
      break;
    }
    case T::RS_RIGHT: {
      const double dphi = ds / rho;
      cx   += rho * (-std::sin(cyaw - dphi) + std::sin(cyaw));
      cy   += rho * ( std::cos(cyaw - dphi) - std::cos(cyaw));
      cyaw  = wrap(cyaw - dphi);
      break;
    }
    default: break;
  }
}

// ─── Plugin implementation ────────────────────────────────────────────────────

void ReedsSheppPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> /*tf*/,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  global_frame_ = costmap_ros->getGlobalFrameID();

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".min_turning_radius",
    rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".interpolation_resolution",
    rclcpp::ParameterValue(0.05));

  node_->get_parameter(name_ + ".min_turning_radius", rho_);
  node_->get_parameter(name_ + ".interpolation_resolution", step_);

  auto pub_qos = rclcpp::QoS(1).transient_local();
  fwd_pub_ = node_->create_publisher<nav_msgs::msg::Path>("/plan_forward", pub_qos);
  rev_pub_ = node_->create_publisher<nav_msgs::msg::Path>("/plan_reverse", pub_qos);

  // Subscribe to constraint topic (latched).
  // Message format: "<turn_side>,<swath_yaw_rad>,forward"  or "" to clear.
  auto sub_qos = rclcpp::QoS(1).transient_local();
  constraint_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/rs_planner_constraints", sub_qos,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      std::lock_guard<std::mutex> lk(constraint_mutex_);
      constraint_raw_ = msg->data;
      // Parse: "left,<yaw_rad>,forward" or ""
      turn_side_constraint_ = "";
      swath_yaw_constraint_ = 0.0;
      force_forward_first_  = false;
      if (!msg->data.empty()) {
        std::istringstream ss(msg->data);
        std::string tok;
        if (std::getline(ss, tok, ',')) turn_side_constraint_ = tok;
        if (std::getline(ss, tok, ',')) swath_yaw_constraint_ = std::stod(tok);
        if (std::getline(ss, tok, ',')) force_forward_first_ = (tok == "forward");
      }
      RCLCPP_INFO(node_->get_logger(),
        "RS constraint: side=%s swath_yaw=%.1f° fwd_first=%d",
        turn_side_constraint_.c_str(),
        swath_yaw_constraint_ * 180.0 / M_PI,
        force_forward_first_);
    });

  RCLCPP_INFO(node_->get_logger(),
    "ReedsSheppPlanner configured (OMPL backend): rho=%.2f m  step=%.3f m", rho_, step_);
}

void ReedsSheppPlanner::cleanup() {}
void ReedsSheppPlanner::activate() {}
void ReedsSheppPlanner::deactivate() {}

// Sample one RS path (already computed by OMPL) into path.poses.
// Propagates from (sx,sy,syaw) in OMPL space.
// If mirror_result is true, reflects each waypoint across the line through
// (mx,my) at angle mirror_yaw back into world frame.
static void sampleRsPath(
  const ompl::base::ReedsSheppStateSpace::ReedsSheppPath & rs_path,
  double rho, double step,
  double sx, double sy, double syaw,
  bool mirror_result,
  double mx, double my, double mirror_yaw,
  const std_msgs::msg::Header & hdr,
  nav_msgs::msg::Path & path,
  nav_msgs::msg::Path & fwd_path,
  nav_msgs::msg::Path & rev_path,
  std::string & seg_str)
{
  using T = ompl::base::ReedsSheppStateSpace::ReedsSheppPathSegmentType;

  double cx = sx, cy = sy, cyaw = syaw;

  for (int i = 0; i < 5; ++i) {
    const T      type    = rs_path.type_[i];
    const double seg_len = rs_path.length_[i];

    if (type == T::RS_NOP || std::abs(seg_len) < 1e-9) continue;

    const double seg_m = seg_len * rho;
    const bool   rev   = (seg_m < 0.0);
    const double dist  = std::abs(seg_m);

    seg_str += (rev ? '-' : '+');
    switch (type) {
      case T::RS_LEFT:     seg_str += 'L'; break;
      case T::RS_RIGHT:    seg_str += 'R'; break;
      case T::RS_STRAIGHT: seg_str += 'S'; break;
      default: break;
    }
    char sbuf[16];
    std::snprintf(sbuf, sizeof(sbuf), "(%.2fm) ", dist);
    seg_str += sbuf;

    const std::size_t before = path.poses.size();
    double travelled = 0.0;
    while (travelled + step < dist - 1e-9) {
      stepPose(type, rev ? -step : step, rho, cx, cy, cyaw);
      travelled += step;

      double wx = cx, wy = cy, wyaw = cyaw;
      if (mirror_result) {
        mirrorPoint(mx, my, mirror_yaw, cx, cy, wx, wy);
        wyaw = mirrorYaw(mirror_yaw, cyaw);
      }

      geometry_msgs::msg::PoseStamped p;
      p.header = hdr;
      p.pose.position.x = wx; p.pose.position.y = wy; p.pose.position.z = 0.0;
      p.pose.orientation = yawToQuat(wyaw);
      path.poses.push_back(p);
    }
    const double remaining = dist - travelled;
    if (remaining > 1e-9) {
      stepPose(type, rev ? -remaining : remaining, rho, cx, cy, cyaw);

      double wx = cx, wy = cy, wyaw = cyaw;
      if (mirror_result) {
        mirrorPoint(mx, my, mirror_yaw, cx, cy, wx, wy);
        wyaw = mirrorYaw(mirror_yaw, cyaw);
      }

      geometry_msgs::msg::PoseStamped p;
      p.header = hdr;
      p.pose.position.x = wx; p.pose.position.y = wy; p.pose.position.z = 0.0;
      p.pose.orientation = yawToQuat(wyaw);
      path.poses.push_back(p);
    }

    for (std::size_t k = before; k < path.poses.size(); ++k) {
      if (rev) rev_path.poses.push_back(path.poses[k]);
      else     fwd_path.poses.push_back(path.poses[k]);
    }
  }
}

nav_msgs::msg::Path ReedsSheppPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  nav_msgs::msg::Path path;
  path.header.stamp = node_->now();
  path.header.frame_id = global_frame_;

  if (start.header.frame_id != global_frame_ ||
      goal.header.frame_id  != global_frame_)
  {
    RCLCPP_ERROR(node_->get_logger(),
      "ReedsSheppPlanner: start/goal must be in frame '%s'", global_frame_.c_str());
    return path;
  }

  const double sx   = start.pose.position.x;
  const double sy   = start.pose.position.y;
  const double syaw = quatToYaw(start.pose.orientation);
  const double gx   = goal.pose.position.x;
  const double gy   = goal.pose.position.y;
  const double gyaw = quatToYaw(goal.pose.orientation);

  if (std::hypot(gx - sx, gy - sy) < 1e-4 && std::abs(wrap(gyaw - syaw)) < 1e-3) {
    return path;
  }

  // Read constraint. The BT node publishes the constraint just before calling
  // ComputePathToPose; the latched message is delivered asynchronously by the
  // executor's subscription callback. Wait up to 50 ms for it to arrive so we
  // don't race past an empty constraint_raw_ on the first planning call.
  {
    const auto deadline = node_->now() + rclcpp::Duration::from_seconds(0.05);
    while (node_->now() < deadline) {
      {
        std::lock_guard<std::mutex> lk(constraint_mutex_);
        if (!constraint_raw_.empty()) break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }
  std::string turn_side;
  double swath_yaw = 0.0;
  bool force_fwd_first = false;
  {
    std::lock_guard<std::mutex> lk(constraint_mutex_);
    turn_side       = turn_side_constraint_;
    swath_yaw       = swath_yaw_constraint_;
    force_fwd_first = force_forward_first_;
  }

  // ── Plan with OMPL ────────────────────────────────────────────────────────
  ompl::base::ReedsSheppStateSpace rs(rho_);
  auto * s_from = rs.allocState()->as<ompl::base::SE2StateSpace::StateType>();
  auto * s_to   = rs.allocState()->as<ompl::base::SE2StateSpace::StateType>();
  s_from->setX(sx); s_from->setY(sy); s_from->setYaw(syaw);
  s_to->setX(gx);   s_to->setY(gy);   s_to->setYaw(gyaw);

  const auto rs_path = rs.reedsShepp(s_from, s_to);
  rs.freeState(s_from);
  rs.freeState(s_to);

  using T = ompl::base::ReedsSheppStateSpace::ReedsSheppPathSegmentType;

  // ── Forward-first constraint ──────────────────────────────────────────────
  // A reverse-first path drives back into the infield immediately after the
  // swath end. We must ensure the first RS segment is driven forward.
  // Check by looking at the sign of the first non-NOP segment length.
  bool use_mirror = false;
  bool first_seg_reverse = false;
  if (force_fwd_first) {
    for (int i = 0; i < 5; ++i) {
      if (rs_path.type_[i] == T::RS_NOP) continue;
      if (std::abs(rs_path.length_[i]) < 1e-9) continue;
      first_seg_reverse = (rs_path.length_[i] < 0.0);
      break;
    }
    if (first_seg_reverse) {
      use_mirror = true;
      RCLCPP_INFO(node_->get_logger(),
        "RS: direct path starts reverse — will search for forward-first candidate");
    }
  }

  // ── Sample ────────────────────────────────────────────────────────────────
  nav_msgs::msg::Path fwd_path, rev_path;
  fwd_path.header = rev_path.header = path.header;
  std::string seg_str;

  // Sample the direct path first.
  sampleRsPath(rs_path, rho_, step_, sx, sy, syaw, false,
    sx, sy, syaw,
    path.header, path, fwd_path, rev_path, seg_str);

  if (use_mirror) {
    // Check if the sampled path actually starts forward by examining the
    // direction from start to second waypoint.
    bool sampled_fwd = false;
    if (path.poses.size() >= 2) {
      const double dx0 = path.poses[1].pose.position.x - path.poses[0].pose.position.x;
      const double dy0 = path.poses[1].pose.position.y - path.poses[0].pose.position.y;
      sampled_fwd = (std::cos(syaw) * dx0 + std::sin(syaw) * dy0) > 0.0;
    }

    if (!sampled_fwd) {
      // Negate all segment lengths: -L+R-L becomes +L-R+L.
      // RS paths are reversible — negating lengths produces a path that
      // traverses the identical arcs forward-first, also connecting start→goal.
      ompl::base::ReedsSheppStateSpace::ReedsSheppPath rs_fwd = rs_path;
      for (int i = 0; i < 5; ++i) rs_fwd.length_[i] = -rs_fwd.length_[i];

      path.poses.clear(); fwd_path.poses.clear(); rev_path.poses.clear();
      std::string seg_fwd;
      sampleRsPath(rs_fwd, rho_, step_, sx, sy, syaw, false,
        sx, sy, syaw,
        path.header, path, fwd_path, rev_path, seg_fwd);
      seg_str = "[fwd] " + seg_fwd;
      RCLCPP_INFO(node_->get_logger(), "RS fwd-first: negated to %s", seg_str.c_str());
    }
  }

  // Close end-point gap
  // Track where the propagation ended by re-examining last pose vs goal
  const double end_gap = path.poses.empty() ? 0.0 :
    std::hypot(path.poses.back().pose.position.x - gx,
               path.poses.back().pose.position.y - gy);

  // Compute path bounding box to detect infield excursion.
  // Swath runs east-west (X axis). Infield is ±Y from swath line.
  // First waypoint direction tells us fwd (same sign as cos/sin of syaw) or rev.
  double bb_xmin = sx, bb_xmax = sx, bb_ymin = sy, bb_ymax = sy;
  bool path_starts_fwd = false;
  if (path.poses.size() >= 2) {
    const double dx0 = path.poses.front().pose.position.x - sx;
    const double dy0 = path.poses.front().pose.position.y - sy;
    path_starts_fwd = (std::cos(syaw) * dx0 + std::sin(syaw) * dy0) > 0.0;
    for (const auto & p : path.poses) {
      bb_xmin = std::min(bb_xmin, p.pose.position.x);
      bb_xmax = std::max(bb_xmax, p.pose.position.x);
      bb_ymin = std::min(bb_ymin, p.pose.position.y);
      bb_ymax = std::max(bb_ymax, p.pose.position.y);
    }
  }
  const double peak_y_excursion = std::max(std::abs(bb_ymax - sy), std::abs(bb_ymin - sy));
  // Flag if path went backward (into infield) or had large lateral excursion.
  const char * fwd_flag = path_starts_fwd ? "FWD" : "REV(INFIELD!)";

  RCLCPP_INFO(node_->get_logger(),
    "RS: (%.2f,%.2f,%.1f°)→(%.2f,%.2f,%.1f°) %s pts=%zu gap=%.4fm "
    "[%s bbox x=%.2f..%.2f y=%.2f..%.2f peak_lateral=%.2fm]",
    sx, sy, syaw * 180.0 / M_PI,
    gx, gy, gyaw * 180.0 / M_PI,
    seg_str.c_str(), path.poses.size(), end_gap,
    fwd_flag, bb_xmin, bb_xmax, bb_ymin, bb_ymax, peak_y_excursion);

  if (!path.poses.empty() && end_gap > 1e-3) {
    RCLCPP_WARN(node_->get_logger(), "RS path end gap %.4fm — appending exact goal", end_gap);
    geometry_msgs::msg::PoseStamped gp;
    gp.header = path.header;
    gp.pose.position.x = gx; gp.pose.position.y = gy; gp.pose.position.z = 0.0;
    gp.pose.orientation = goal.pose.orientation;
    path.poses.push_back(gp);
    // Append to whichever of fwd/rev had the last waypoint
    const auto & last = path.poses[path.poses.size() - 2];
    bool last_in_rev = !rev_path.poses.empty() &&
      std::abs(rev_path.poses.back().pose.position.x - last.pose.position.x) < 1e-6 &&
      std::abs(rev_path.poses.back().pose.position.y - last.pose.position.y) < 1e-6;
    if (last_in_rev) rev_path.poses.push_back(gp);
    else             fwd_path.poses.push_back(gp);
  }

  fwd_pub_->publish(fwd_path);
  rev_pub_->publish(rev_path);

  return path;
}

}  // namespace planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(planner::ReedsSheppPlanner, nav2_core::GlobalPlanner)
