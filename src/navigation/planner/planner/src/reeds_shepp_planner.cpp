// Reeds-Shepp planner plugin for Nav2.
//
// Implements all 48 Reeds-Shepp words from the original 1990 paper
// (Reeds & Shepp, "Optimal paths for a car that goes both forwards and backwards").
// The minimum-length path is found by checking all word families and picking the
// shortest feasible path that fits within the robot's turning radius.
//
// Segment encoding
// ----------------
//   Each Reeds-Shepp segment is (type, length, direction):
//     type:      'S' = straight, 'L' = left arc, 'R' = right arc
//     length:    path length in normalised units (multiply by rho for metres)
//     direction: +1 = forward, -1 = reverse
//
// Path output
// -----------
//   Each segment is sampled at `step_` metre intervals into PoseStamped points.
//   On reverse segments the yaw is rotated by π so that RPP drives backward.

#include "planner/reeds_shepp_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "nav2_util/node_utils.hpp"

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

// ─── Reeds-Shepp internals ────────────────────────────────────────────────────

struct Segment
{
  char  type;    // 'S', 'L', 'R'
  double len;   // normalised length (can be negative for reverse)
  // direction is encoded in the sign of len:
  //   len > 0  → forward   (drive in the arc / straight direction)
  //   len < 0  → reverse   (drive opposite direction)
};

struct RSPath
{
  std::vector<Segment> segs;
  double total_length() const
  {
    double s = 0.0;
    for (auto & sg : segs) s += std::abs(sg.len);
    return s;
  }
};

// Normalise goal into Reeds-Shepp frame (start = origin, heading = 0).
// Returns (x, y, phi) all normalised by rho.
struct Goal2D { double x, y, phi; };

static Goal2D normalise(
  double sx, double sy, double syaw,
  double gx, double gy, double gyaw,
  double rho)
{
  const double dx  = gx - sx;
  const double dy  = gy - sy;
  const double c   = std::cos(syaw);
  const double s   = std::sin(syaw);
  const double lx  = ( c * dx + s * dy) / rho;
  const double ly  = (-s * dx + c * dy) / rho;
  const double phi = wrap(gyaw - syaw);
  return {lx, ly, phi};
}

// ─── RS word families ────────────────────────────────────────────────────────
// All 12 base families from Reeds & Shepp (1990), Table 1.
// Naming: segment type L/R/S, sign + (forward) / - (reverse).
// Each function operates in the normalised frame (start=origin, rho=1).
// The four symmetries (timeflip, reflect, both, none) are applied by collect().

using Opt = std::optional<RSPath>;

// ── Family 1: CSC  L+S+L+  ──────────────────────────────────────────────────
static Opt LpSpLp(double x, double y, double phi)
{
  const double xi  = x - std::sin(phi);
  const double eta = y - 1.0 + std::cos(phi);
  const double rho = std::hypot(xi, eta);
  if (rho < 1e-9) return {};
  const double t = std::atan2(eta, xi);
  const double u = rho;
  const double v = wrap(phi - t);
  if (t < -1e-6 || v < -1e-6) return {};
  return RSPath{{{'L', t}, {'S', u}, {'L', v}}};
}

// ── Family 2: CSC  L+S+R+  ──────────────────────────────────────────────────
static Opt LpSpRp(double x, double y, double phi)
{
  const double xi  = x + std::sin(phi);
  const double eta = y - 1.0 - std::cos(phi);
  const double r2  = xi * xi + eta * eta;
  if (r2 < 4.0) return {};
  const double u = std::sqrt(r2 - 4.0);
  const double t = std::atan2(eta, xi) - std::atan2(2.0, u);
  const double v = wrap(t - phi);
  if (t < -1e-6 || v < -1e-6) return {};
  return RSPath{{{'L', t}, {'S', u}, {'R', v}}};
}

// ── Family 3: CCC  L+R-L+  ──────────────────────────────────────────────────
static Opt LpRmLp(double x, double y, double phi)
{
  const double xi  = x - std::sin(phi);
  const double eta = y - 1.0 + std::cos(phi);
  const double rho = std::hypot(xi, eta);
  if (rho > 4.0) return {};
  const double u = std::acos(1.0 - rho * rho / 8.0);
  const double A = std::atan2(eta, xi);
  const double t = wrap(A + 0.5 * u + M_PI);
  const double v = wrap(phi - t + u);
  if (t < -1e-6 || v < -1e-6) return {};
  return RSPath{{{'L', t}, {'R', -u}, {'L', v}}};
}

// ── Family 4: CCC  L+R-L-  ──────────────────────────────────────────────────
static Opt LpRmLm(double x, double y, double phi)
{
  const double xi  = x - std::sin(phi);
  const double eta = y - 1.0 + std::cos(phi);
  const double rho = std::hypot(xi, eta);
  if (rho > 4.0) return {};
  const double u = std::acos(1.0 - rho * rho / 8.0);
  const double A = std::atan2(eta, xi);
  const double t = wrap(A + 0.5 * u + M_PI);
  const double v = wrap(t + u - phi);
  if (t < -1e-6 || v < -1e-6) return {};
  return RSPath{{{'L', t}, {'R', -u}, {'L', -v}}};
}

// ── Family 5: CCC  L+R+L-  ──────────────────────────────────────────────────
static Opt LpRpLm(double x, double y, double phi)
{
  const double xi  = x + std::sin(phi);
  const double eta = y - 1.0 - std::cos(phi);
  const double rho = std::hypot(xi, eta);
  if (rho > 4.0) return {};
  const double u = std::acos(1.0 - rho * rho / 8.0);
  const double A = std::atan2(eta, xi);
  const double t = wrap(A - 0.5 * u + M_PI);
  const double v = wrap(t - u - phi);
  if (t < -1e-6 || v < -1e-6) return {};
  return RSPath{{{'L', t}, {'R', u}, {'L', -v}}};
}

// ── Family 6: CCSC  L+R-S-L-  ───────────────────────────────────────────────
static Opt LpRmSmLm(double x, double y, double phi)
{
  const double xi  = x + std::sin(phi);
  const double eta = y - 1.0 - std::cos(phi);
  const double rho = std::hypot(xi, eta);
  if (rho < 2.0) return {};
  const double u = std::sqrt(rho * rho - 4.0) - 2.0;
  if (u < -1e-6) return {};
  const double A = std::atan2(eta, xi);
  const double t = wrap(A + std::atan2(2.0, rho - 2.0) + M_PI / 2.0);
  const double v = wrap(t - phi + M_PI / 2.0);
  if (t < -1e-6 || v < -1e-6) return {};
  return RSPath{{{'L', t}, {'R', -M_PI / 2.0}, {'S', -u}, {'L', -v}}};
}

// ── Family 7: CCSC  L+R-S-R-  ───────────────────────────────────────────────
static Opt LpRmSmRm(double x, double y, double phi)
{
  const double xi  = x + std::sin(phi);
  const double eta = y - 1.0 - std::cos(phi);
  const double rho = std::hypot(xi, eta);
  if (rho < 2.0) return {};
  const double u = std::sqrt(rho * rho - 4.0) - 2.0;
  if (u < -1e-6) return {};
  const double A = std::atan2(eta, xi);
  const double t = wrap(A + std::atan2(2.0, rho - 2.0) + M_PI / 2.0);
  const double v = wrap(t - phi);
  if (t < -1e-6 || v < -1e-6) return {};
  return RSPath{{{'L', t}, {'R', -M_PI / 2.0}, {'S', -u}, {'R', -v}}};
}

// ── Family 8: CCSCC  L+R-S-L-R+  ────────────────────────────────────────────
static Opt LpRmSmLmRp(double x, double y, double phi)
{
  const double xi  = x + std::sin(phi);
  const double eta = y - 1.0 - std::cos(phi);
  const double rho = std::hypot(xi, eta);
  if (rho < 4.0) return {};
  const double u = std::sqrt(rho * rho - 4.0) - 4.0;
  if (u < -1e-6) return {};
  const double A = std::atan2(eta, xi);
  const double t = wrap(A + std::atan2(2.0, rho - 4.0) + M_PI / 2.0);
  const double v = wrap(t - phi);
  if (t < -1e-6 || v < -1e-6) return {};
  return RSPath{{{'L', t}, {'R', -M_PI / 2.0}, {'S', -u}, {'L', -M_PI / 2.0}, {'R', v}}};
}

// ── Family 9: CCCC  L+R+L-R-  ────────────────────────────────────────────────
static Opt LpRpLmRm(double x, double y, double phi)
{
  const double xi  = x - std::sin(phi);
  const double eta = y - 1.0 + std::cos(phi);
  const double r2  = xi * xi + eta * eta;
  const double p = (2.0 + r2) / 4.0;
  if (p < 0.0 || p > 1.0) return {};
  const double u = std::acos(std::sqrt(p));
  const double A = std::atan2(eta, xi);
  const double t = wrap(A - std::atan2(-std::sin(2.0 * u), 1.0 - 2.0 * std::cos(2.0 * u)) + M_PI);
  const double v = wrap(phi - t + 2.0 * u);
  if (t < -1e-6 || v < -1e-6) return {};
  return RSPath{{{'L', t}, {'R', u}, {'L', -u}, {'R', -v}}};
}

// ── Family 10: CCCC  L+R-L-R+  ────────────────────────────────────────────────
static Opt LpRmLmRp(double x, double y, double phi)
{
  const double xi  = x + std::sin(phi);
  const double eta = y - 1.0 - std::cos(phi);
  const double r2  = xi * xi + eta * eta;
  const double p   = (2.0 + r2) / 4.0;
  if (p < 0.0 || p > 1.0) return {};
  const double u = std::acos(std::sqrt(p));
  const double A = std::atan2(eta, xi);
  const double t = wrap(A + std::atan2(-std::sin(2.0 * u), -1.0 + 2.0 * std::cos(2.0 * u)));
  const double v = wrap(t - phi);
  if (t < -1e-6 || v < -1e-6) return {};
  return RSPath{{{'L', t}, {'R', -u}, {'L', -u}, {'R', v}}};
}

// ── Family 11: CSCC  L+S+R+L-  ───────────────────────────────────────────────
static Opt LpSpRpLm(double x, double y, double phi)
{
  const double xi  = x - std::sin(phi);
  const double eta = y - 1.0 + std::cos(phi);
  const double rho = std::hypot(xi, eta);
  if (rho < 2.0) return {};
  const double u   = std::sqrt(rho * rho - 4.0) - 2.0;
  if (u < -1e-6) return {};
  const double A   = std::atan2(eta, xi);
  const double t   = wrap(A - std::atan2(2.0, rho - 2.0));
  const double v   = wrap(t - phi - M_PI / 2.0);
  if (t < -1e-6 || v < -1e-6) return {};
  return RSPath{{{'L', t}, {'S', u}, {'R', M_PI / 2.0}, {'L', -v}}};
}

// ── Family 12: CSCC  L+S+L+R-  ───────────────────────────────────────────────
static Opt LpSpLpRm(double x, double y, double phi)
{
  const double xi  = x + std::sin(phi);
  const double eta = y + 1.0 - std::cos(phi);
  const double rho = std::hypot(xi, eta);
  if (rho < 2.0) return {};
  const double u   = std::sqrt(rho * rho - 4.0) - 2.0;
  if (u < -1e-6) return {};
  const double A   = std::atan2(eta, xi);
  const double t   = wrap(A + std::atan2(2.0, rho - 2.0));
  const double v   = wrap(phi - t + M_PI / 2.0);
  if (t < -1e-6 || v < -1e-6) return {};
  return RSPath{{{'L', t}, {'S', u}, {'L', M_PI / 2.0}, {'R', -v}}};
}

// ─── Symmetry transforms ─────────────────────────────────────────────────────
// Reeds & Shepp exploit four symmetries to reduce to a minimal set of formulas:
//   time-flip:   (x, y, phi) → (-x, y, -phi)   (reverse all directions)
//   reflection:  (x, y, phi) → (x, -y, -phi)    (left ↔ right)

static RSPath timeflip(RSPath p)
{
  for (auto & s : p.segs) s.len = -s.len;
  return p;
}

static RSPath reflect(RSPath p)
{
  for (auto & s : p.segs) {
    if (s.type == 'L') s.type = 'R';
    else if (s.type == 'R') s.type = 'L';
  }
  return p;
}

// Collect all symmetry variants of a path formula applied to (x,y,phi).
static void collect(
  std::function<Opt(double, double, double)> fn,
  double x, double y, double phi,
  std::vector<RSPath> & candidates)
{
  double xf = -x, yf =  y, pf = -phi;  // time-flip coords
  double xr =  x, yr = -y, pr = -phi;  // reflect coords
  double xfr = -x, yfr = -y, pfr = phi; // both

  if (auto p = fn( x,  y,  phi)) candidates.push_back(*p);
  if (auto p = fn(xf, yf, pf))   candidates.push_back(timeflip(*p));
  if (auto p = fn(xr, yr, pr))   candidates.push_back(reflect(*p));
  if (auto p = fn(xfr, yfr, pfr)) candidates.push_back(reflect(timeflip(*p)));
}

// Cost of a path: total length plus a penalty per reverse segment so that
// forward-only paths always beat equal-length all-reverse alternatives.
// The penalty (5% of length per reverse segment) only affects tie-breaking;
// it does not prevent genuinely shorter reverse paths from winning.
static double pathCost(const RSPath & p)
{
  double total = 0.0;
  double rev_len = 0.0;
  for (auto & s : p.segs) {
    total += std::abs(s.len);
    if (s.len < 0.0) rev_len += std::abs(s.len);
  }
  return total + 0.05 * rev_len;
}

static RSPath bestPath(double x, double y, double phi)
{
  std::vector<RSPath> cands;

  // CSC (families 1-2)
  collect(LpSpLp,      x, y, phi, cands);
  collect(LpSpRp,      x, y, phi, cands);

  // CCC (families 3-5)
  collect(LpRmLp,      x, y, phi, cands);
  collect(LpRmLm,      x, y, phi, cands);
  collect(LpRpLm,      x, y, phi, cands);

  // CCSC (families 6-7)
  collect(LpRmSmLm,    x, y, phi, cands);
  collect(LpRmSmRm,    x, y, phi, cands);

  // CCSCC (family 8)
  collect(LpRmSmLmRp,  x, y, phi, cands);

  // CCCC (families 9-10)
  collect(LpRpLmRm,    x, y, phi, cands);
  collect(LpRmLmRp,    x, y, phi, cands);

  // CSCC (families 11-12)
  collect(LpSpRpLm,    x, y, phi, cands);
  collect(LpSpLpRm,    x, y, phi, cands);

  // Pick lowest-cost path (length + small reverse penalty for tie-breaking).
  RSPath best;
  double bestCost = std::numeric_limits<double>::infinity();
  for (auto & c : cands) {
    double cost = pathCost(c);
    if (cost < bestCost) { bestCost = cost; best = c; }
  }
  return best;
}

// ─── Path sampling ────────────────────────────────────────────────────────────

// Advance pose by one arc/straight step of ds metres (signed by rev).
static void stepPose(char type, double ds, bool rev, double rho,
                     double & cx, double & cy, double & cyaw)
{
  const double d = rev ? -ds : ds;
  switch (type) {
    case 'S':
      cx   += d * std::cos(cyaw);
      cy   += d * std::sin(cyaw);
      break;
    case 'L': {
      const double dphi = d / rho;
      cx   += rho * (std::sin(cyaw + dphi) - std::sin(cyaw));
      cy   += rho * (-std::cos(cyaw + dphi) + std::cos(cyaw));
      cyaw  = wrap(cyaw + dphi);
      break;
    }
    case 'R': {
      const double dphi = d / rho;
      cx   += rho * (-std::sin(cyaw - dphi) + std::sin(cyaw));
      cy   += rho * ( std::cos(cyaw - dphi) - std::cos(cyaw));
      cyaw  = wrap(cyaw - dphi);
      break;
    }
  }
}

// Propagate a pose along one segment, emitting waypoints at `step` metre intervals.
// cx/cy/cyaw are updated to the exact geometric end of the segment.
static void sampleSegment(
  const Segment & seg,
  double rho,
  double step,
  double & cx, double & cy, double & cyaw,
  const std_msgs::msg::Header & header,
  std::vector<geometry_msgs::msg::PoseStamped> & out)
{
  const double len_m = seg.len * rho;         // signed metres
  const bool   rev   = (len_m < 0.0);
  const double dist  = std::abs(len_m);

  // Emit intermediate waypoints at `step` intervals, then one final point at
  // the exact geometric segment end. This avoids any integer-truncation gap.
  double travelled = 0.0;
  while (travelled + step < dist - 1e-9) {
    stepPose(seg.type, step, rev, rho, cx, cy, cyaw);
    travelled += step;

    geometry_msgs::msg::PoseStamped p;
    p.header = header;
    p.pose.position.x = cx;
    p.pose.position.y = cy;
    p.pose.position.z = 0.0;
    p.pose.orientation = yawToQuat(rev ? wrap(cyaw + M_PI) : cyaw);
    out.push_back(p);
  }

  // Final step: advance exactly to the end of the segment.
  const double remaining = dist - travelled;
  if (remaining > 1e-9) {
    stepPose(seg.type, remaining, rev, rho, cx, cy, cyaw);

    geometry_msgs::msg::PoseStamped p;
    p.header = header;
    p.pose.position.x = cx;
    p.pose.position.y = cy;
    p.pose.position.z = 0.0;
    p.pose.orientation = yawToQuat(rev ? wrap(cyaw + M_PI) : cyaw);
    out.push_back(p);
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

  auto qos = rclcpp::QoS(1).transient_local();
  fwd_pub_ = node_->create_publisher<nav_msgs::msg::Path>("/plan_forward", qos);
  rev_pub_ = node_->create_publisher<nav_msgs::msg::Path>("/plan_reverse", qos);

  RCLCPP_INFO(node_->get_logger(),
    "ReedsSheppPlanner configured: rho=%.2f m  step=%.3f m", rho_, step_);
}

void ReedsSheppPlanner::cleanup() {}
void ReedsSheppPlanner::activate() {}
void ReedsSheppPlanner::deactivate() {}

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

  // Trivially at goal?
  if (std::hypot(gx - sx, gy - sy) < 1e-4 && std::abs(wrap(gyaw - syaw)) < 1e-3) {
    return path;
  }

  // Transform goal into robot's start frame, normalised by rho
  const Goal2D g = normalise(sx, sy, syaw, gx, gy, gyaw, rho_);

  // Find the best Reeds-Shepp path
  const RSPath rs = bestPath(g.x, g.y, g.phi);

  if (rs.segs.empty()) {
    // RS word families returned no valid path (can happen for near-collinear
    // start/goal where all t/v sign checks fail). Fall back to a straight-line
    // path so the controller can at least make progress.
    RCLCPP_WARN(node_->get_logger(),
      "ReedsSheppPlanner: no RS path found (%.2f,%.2f → %.2f,%.2f), using straight line",
      sx, sy, gx, gy);
    const double dist = std::hypot(gx - sx, gy - sy);
    const int n = std::max(2, static_cast<int>(dist / step_));
    for (int i = 0; i <= n; ++i) {
      const double t = static_cast<double>(i) / n;
      geometry_msgs::msg::PoseStamped p;
      p.header = path.header;
      p.pose.position.x = sx + t * (gx - sx);
      p.pose.position.y = sy + t * (gy - sy);
      p.pose.position.z = 0.0;
      p.pose.orientation = (i == n) ? goal.pose.orientation : yawToQuat(syaw);
      path.poses.push_back(p);
    }
    return path;
  }

  // Build segment description string and split fwd/rev paths for visualisation.
  std::string seg_str;
  nav_msgs::msg::Path fwd_path, rev_path;
  fwd_path.header = rev_path.header = path.header;

  double cx = sx, cy = sy, cyaw = syaw;

  RCLCPP_INFO(node_->get_logger(),
    "RS sample: start=(%.4f,%.4f,%.4f°) goal=(%.4f,%.4f,%.4f°) rho=%.3f step=%.4f",
    sx, sy, syaw * 180.0 / M_PI,
    gx, gy, gyaw * 180.0 / M_PI, rho_, step_);

  for (const auto & seg : rs.segs) {
    if (cancel_checker && cancel_checker()) return path;
    if (std::abs(seg.len) < 1e-9) continue;

    const bool rev = (seg.len < 0.0);
    const double seg_m = std::abs(seg.len) * rho_;

    seg_str += (rev ? '-' : '+');
    seg_str += seg.type;
    seg_str += '(';
    seg_str += std::to_string(static_cast<int>(std::round(std::abs(seg.len) * rho_ * 10.0) / 10.0));
    seg_str += "dm) ";

    const double pre_x = cx, pre_y = cy, pre_yaw = cyaw;
    std::size_t before = path.poses.size();
    sampleSegment(seg, rho_, step_, cx, cy, cyaw, path.header, path.poses);
    const std::size_t n_added = path.poses.size() - before;

    RCLCPP_INFO(node_->get_logger(),
      "  seg %c%c len=%.4fm  in=(%.4f,%.4f,%.3f°)  out=(%.4f,%.4f,%.3f°)  poses=%zu",
      rev ? '-' : '+', seg.type, seg_m,
      pre_x, pre_y, pre_yaw * 180.0 / M_PI,
      cx, cy, cyaw * 180.0 / M_PI, n_added);

    for (std::size_t i = before; i < path.poses.size(); ++i) {
      if (rev) rev_path.poses.push_back(path.poses[i]);
      else     fwd_path.poses.push_back(path.poses[i]);
    }
  }

  // After sampling all segments, cx/cy/cyaw should equal gx/gy/gyaw.
  // Log the residual gap so we can detect RS formula errors.
  const double end_gap = std::hypot(cx - gx, cy - gy);
  const double yaw_err = std::abs(wrap(cyaw - gyaw)) * 180.0 / M_PI;
  RCLCPP_INFO(node_->get_logger(),
    "RS end: sampled=(%.4f,%.4f,%.3f°) goal=(%.4f,%.4f,%.3f°)  gap=%.4fm  yaw_err=%.3f°",
    cx, cy, cyaw * 180.0 / M_PI,
    gx, gy, gyaw * 180.0 / M_PI, end_gap, yaw_err);

  // Append exact goal pose if the last sampled point is more than one step away.
  // Never overwrite (snap) the last point — that creates a teleport discontinuity.
  if (!path.poses.empty()) {
    const auto & last = path.poses.back().pose.position;
    const double gap = std::hypot(gx - last.x, gy - last.y);
    if (gap > 1e-3) {
      RCLCPP_WARN(node_->get_logger(),
        "RS path end gap %.4fm > 1mm — appending exact goal point", gap);
      geometry_msgs::msg::PoseStamped gp;
      gp.header = path.header;
      gp.pose.position.x  = gx;
      gp.pose.position.y  = gy;
      gp.pose.position.z  = 0.0;
      const bool last_rev = (rs.segs.back().len < 0.0);
      gp.pose.orientation = last_rev
        ? yawToQuat(wrap(gyaw + M_PI))
        : goal.pose.orientation;
      path.poses.push_back(gp);
      if (last_rev) rev_path.poses.push_back(gp);
      else          fwd_path.poses.push_back(gp);
    }
  }

  fwd_pub_->publish(fwd_path);
  rev_pub_->publish(rev_path);

  RCLCPP_INFO(node_->get_logger(),
    "ReedsSheppPlanner: (%.2f,%.2f,%.1f°) → (%.2f,%.2f,%.1f°)  %s waypts=%zu",
    sx, sy, syaw * 180.0 / M_PI,
    gx, gy, gyaw * 180.0 / M_PI,
    seg_str.c_str(), path.poses.size());

  return path;
}

}  // namespace planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(planner::ReedsSheppPlanner, nav2_core::GlobalPlanner)
