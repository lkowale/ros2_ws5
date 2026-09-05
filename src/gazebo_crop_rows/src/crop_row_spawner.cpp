#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <robot_localization/srv/from_ll.hpp>

#include <gz/transport/Node.hh>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/boolean.pb.h>

#include <nlohmann/json.hpp>

#include <atomic>
#include <cmath>
#include <fstream>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

struct Swath {
  double cx, cy;
  double ux, uy;
  double px, py;
  double yaw;
  double length;
  double offset;
};

struct Segment {
  double along;
};

static double quat_yaw(double x, double y, double z, double w) {
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

// All segments render green.
static const float kSegColors[1][3] = {
  {0.13f, 0.55f, 0.13f},  // green
};

// Single-box SDF used by the rolling spawner (non-spawn_all mode).
static std::string make_sdf(const std::string & name,
                             double x, double y, double yaw,
                             double length, double width,
                             int index)
{
  const float * c = kSegColors[0];
  char buf[2048];
  std::snprintf(buf, sizeof(buf),
    "<sdf version=\"1.7\">"
    "<model name=\"%s\">"
    "<static>true</static>"
    "<pose>%.4f %.4f 0.005 0 0 %.6f</pose>"
    "<link name=\"link\">"
    "<visual name=\"visual\">"
    "<geometry><box><size>%.4f %.4f 0.01</size></box></geometry>"
    "<material>"
    "<ambient>%.2f %.2f %.2f 1</ambient>"
    "<diffuse>%.2f %.2f %.2f 1</diffuse>"
    "<specular>0 0 0 1</specular>"
    "</material>"
    "</visual>"
    "</link>"
    "</model>"
    "</sdf>",
    name.c_str(), x, y, yaw, length, width,
    c[0], c[1], c[2], c[0], c[1], c[2]);
  return buf;
}

// Multi-link swath SDF: one model containing all L/R segment boxes for one swath.
// Each segment link is positioned relative to the model origin (gz world frame origin).
// The model pose is identity (0,0,0) so link poses are absolute gz-world coordinates.
static std::string make_swath_sdf(
  const std::string & model_name,
  const std::vector<std::tuple<std::string, double, double, double, double, double, int>> & boxes,
  double length, double width)
{
  // boxes: vector of (link_name, gz_x, gz_y, gz_yaw, length, width, seg_index)
  std::string sdf;
  sdf.reserve(boxes.size() * 512 + 256);
  sdf += "<sdf version=\"1.7\"><model name=\"";
  sdf += model_name;
  sdf += "\"><static>true</static><pose>0 0 0 0 0 0</pose>";

  for (auto & [lname, lx, ly, lyaw, llen, lwid, lidx] : boxes) {
    const float * c = kSegColors[0];
    char buf[768];
    std::snprintf(buf, sizeof(buf),
      "<link name=\"%s\">"
      "<pose>%.4f %.4f 0.005 0 0 %.6f</pose>"
      "<visual name=\"visual\">"
      "<geometry><box><size>%.4f %.4f 0.01</size></box></geometry>"
      "<material>"
      "<ambient>%.2f %.2f %.2f 1</ambient>"
      "<diffuse>%.2f %.2f %.2f 1</diffuse>"
      "<specular>0 0 0 1</specular>"
      "</material>"
      "</visual>"
      "</link>",
      lname.c_str(), lx, ly, lyaw, llen, lwid,
      c[0], c[1], c[2], c[0], c[1], c[2]);
    sdf += buf;
  }
  sdf += "</model></sdf>";
  return sdf;
  (void)length; (void)width;
}

class CropRowSpawner : public rclcpp::Node {
public:
  CropRowSpawner() : Node("crop_row_spawner") {
    declare_parameter("field_file", "");
    declare_parameter("world_name", "house_short_crop_rows");
    declare_parameter("robot_model", "solbot5");
    declare_parameter("gz_odom_topic", "/gz/robot_world_odom");
    declare_parameter("spawn_ahead_m", 4.0);
    declare_parameter("remove_behind_m", 3.0);
    declare_parameter("row_offset_m", 0.18);
    declare_parameter("row_width_m", 0.06);
    declare_parameter("segment_length_m", 2.0);
    declare_parameter("min_move_m", 1.5);
    declare_parameter("spawn_all", false);

    field_file_   = get_parameter("field_file").as_string();
    world_        = get_parameter("world_name").as_string();
    ahead_        = get_parameter("spawn_ahead_m").as_double();
    behind_       = get_parameter("remove_behind_m").as_double();
    row_offset_   = get_parameter("row_offset_m").as_double();
    width_        = get_parameter("row_width_m").as_double();
    seg_len_      = get_parameter("segment_length_m").as_double();
    min_move_     = get_parameter("min_move_m").as_double();
    spawn_all_    = get_parameter("spawn_all").as_bool();

    if (field_file_.empty()) {
      RCLCPP_ERROR(get_logger(), "field_file parameter not set");
      return;
    }

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    from_ll_cli_ = create_client<robot_localization::srv::FromLL>("/fromLL");

    spawn_srv_  = "/world/" + world_ + "/create";
    remove_srv_ = "/world/" + world_ + "/remove";

    event_pub_ = create_publisher<std_msgs::msg::String>(
      "crop_row_spawner/events", rclcpp::QoS(50));

    // Subscribe to ground-truth world-frame odometry (NOT the drifting Ackermann
    // wheel odom on odometry/gazebo) so the map<->gz offset stays correct.
    gz_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      get_parameter("gz_odom_topic").as_string(), 10,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        const auto & p = msg->pose.pose.position;
        const auto & q = msg->pose.pose.orientation;
        double yaw = quat_yaw(q.x, q.y, q.z, q.w);
        gz_pose_x_.store(p.x);
        gz_pose_y_.store(p.y);
        gz_pose_yaw_.store(yaw);
        gz_pose_valid_.store(true);
      });

    // Wait for /fromLL then load swaths using it for coordinate conversion
    RCLCPP_INFO(get_logger(), "Waiting for /fromLL service...");
    init_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&CropRowSpawner::try_init, this));
  }

private:
  void try_init() {
    if (!from_ll_cli_->service_is_ready()) return;
    init_timer_->cancel();

    RCLCPP_INFO(get_logger(), "/fromLL ready — loading swaths from %s",
                field_file_.c_str());
    load_swath_points();
  }

  void load_swath_points() {
    std::ifstream f(field_file_);
    auto d = nlohmann::json::parse(f);

    // Collect all WGS84 endpoints
    pending_raw_.clear();
    for (auto & feat : d["features"]) {
      auto & c = feat["geometry"]["coordinates"];
      pending_raw_.push_back({c[0][0], c[0][1], c[1][0], c[1][1]});
    }

    pending_idx_ = 0;
    converted_.clear();
    converted_.resize(pending_raw_.size() * 2);
    convert_next_point();
  }

  void convert_next_point() {
    size_t total = pending_raw_.size() * 2;
    if (pending_idx_ >= total) {
      finish_swath_load();
      return;
    }

    size_t si = pending_idx_ / 2;
    bool second = (pending_idx_ % 2) == 1;
    auto & r = pending_raw_[si];

    auto req = std::make_shared<robot_localization::srv::FromLL::Request>();
    req->ll_point.latitude  = second ? r.lat1 : r.lat0;
    req->ll_point.longitude = second ? r.lon1 : r.lon0;
    req->ll_point.altitude  = 100.0;

    from_ll_cli_->async_send_request(req,
      [this](rclcpp::Client<robot_localization::srv::FromLL>::SharedFuture fut) {
        auto & p = fut.get()->map_point;
        converted_[pending_idx_] = {p.x, p.y};
        ++pending_idx_;
        convert_next_point();
      });
  }

  void finish_swath_load() {
    // /fromLL serves as soon as navsat_transform is up, but returns (0,0) for
    // every point until wait_for_datum's datum is actually set — silently
    // poisoning every swath with len=0/NaN forever (this function only runs
    // once). Detect that degenerate case and retry the whole load instead of
    // committing to it.
    bool degenerate = true;
    for (size_t i = 0; i < pending_raw_.size() && degenerate; ++i) {
      double dx = converted_[2*i+1].x - converted_[2*i].x;
      double dy = converted_[2*i+1].y - converted_[2*i].y;
      if (std::hypot(dx, dy) > 1e-3) degenerate = false;
    }
    if (degenerate) {
      RCLCPP_WARN(get_logger(),
        "/fromLL returned degenerate points for every swath (datum not set "
        "yet?) — retrying load in 2s");
      swath_load_retry_timer_ = create_wall_timer(
        std::chrono::milliseconds(2000),
        [this]() {
          swath_load_retry_timer_->cancel();
          load_swath_points();
        });
      return;
    }

    for (size_t i = 0; i < pending_raw_.size(); ++i) {
      double x0 = converted_[2*i].x,   y0 = converted_[2*i].y;
      double x1 = converted_[2*i+1].x, y1 = converted_[2*i+1].y;
      double dx = x1-x0, dy = y1-y0;
      double len = std::hypot(dx, dy);
      Swath sw;
      sw.ux = dx/len; sw.uy = dy/len;
      // Perpendicular always points to the same physical side regardless of
      // which direction the swath was stored. Ensure py > 0 (points North in ENU).
      sw.px = -sw.uy; sw.py = sw.ux;
      if (sw.py < 0) { sw.px = -sw.px; sw.py = -sw.py; }
      sw.yaw = std::atan2(dy, dx);
      sw.cx = (x0+x1)/2; sw.cy = (y0+y1)/2;
      sw.length = len; sw.offset = row_offset_;
      swaths_.push_back(sw);
      RCLCPP_INFO(get_logger(),
        "  swath[%zu]: (%.2f,%.2f)->(%.2f,%.2f) len=%.2f yaw=%.1f° ux=%.3f uy=%.3f",
        i, x0, y0, x1, y1, len, sw.yaw * 180.0 / M_PI, sw.ux, sw.uy);
    }
    RCLCPP_INFO(get_logger(), "Loaded %zu swaths (map-frame coords via /fromLL)",
                swaths_.size());

    if (spawn_all_) {
      // Wait for gz odom so we can compute the map→gz offset, then spawn everything.
      spawn_all_pending_timer_ = create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&CropRowSpawner::try_spawn_all, this));
    } else {
      timer_ = create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&CropRowSpawner::update, this));
    }
  }

  void try_spawn_all() {
    if (!gz_pose_valid_.load()) return;
    spawn_all_pending_timer_->cancel();
    spawn_all_swaths();
  }

  void spawn_all_swaths() {
    // Compute map→gz offset from current odom vs TF.
    double rx, ry, ryaw;
    if (!get_transforms(rx, ry, ryaw)) {
      RCLCPP_WARN(get_logger(), "spawn_all: TF not ready, retrying...");
      spawn_all_pending_timer_ = create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&CropRowSpawner::try_spawn_all, this));
      return;
    }
    double gz_x = gz_pose_x_.load();
    double gz_y = gz_pose_y_.load();
    double gz_yaw = gz_pose_yaw_.load();
    gz_offset_x_   = rx - gz_x;
    gz_offset_y_   = ry - gz_y;
    gz_yaw_offset_ = gz_yaw - ryaw;

    RCLCPP_INFO(get_logger(),
      "spawn_all: gz_offset=(%.3f, %.3f) gz_yaw_off=%.2f°",
      gz_offset_x_, gz_offset_y_, gz_yaw_offset_ * 180.0 / M_PI);

    // Spawn one multi-link model per swath (16 spawns instead of 448).
    // All segment links live inside one SDF model — far fewer scene graph entries.
    int sw_idx = 0;
    int total_models = 0;
    int total_links = 0;
    int ctr = 0;
    for (auto & sw : swaths_) {
      double gz_row_yaw = sw.yaw + gz_yaw_offset_;
      double start = -sw.length / 2.0 + seg_len_ / 2.0;
      double end   =  sw.length / 2.0;

      using BoxTuple = std::tuple<std::string, double, double, double, double, double, int>;
      std::vector<BoxTuple> boxes;
      int seg_ctr = ctr;
      for (double a = start; a < end; a += seg_len_, ++seg_ctr) {
        double seg_cx = sw.cx + a * sw.ux;
        double seg_cy = sw.cy + a * sw.uy;
        for (int sign : {+1, -1}) {
          double wx = (seg_cx + sign * sw.offset * sw.px) - gz_offset_x_;
          double wy = (seg_cy + sign * sw.offset * sw.py) - gz_offset_y_;
          std::string lname = "seg" + std::to_string(seg_ctr) + (sign > 0 ? "_L" : "_R");
          boxes.emplace_back(lname, wx, wy, gz_row_yaw, seg_len_, width_, seg_ctr);
        }
      }

      std::string model_name = "gcr_sw" + std::to_string(sw_idx);
      gz::msgs::EntityFactory req;
      req.set_name(model_name);
      req.set_sdf(make_swath_sdf(model_name, boxes, seg_len_, width_));
      gz::msgs::Boolean rep;
      bool result = false;
      if (gz_node_.Request(spawn_srv_, req, 2000, rep, result) && rep.data()) {
        // Record the model name so remove_all() can clean it up if ever needed.
        std::lock_guard<std::mutex> lk(mtx_);
        spawned_[model_name] = {0.0};
        ++total_models;
        total_links += static_cast<int>(boxes.size());
      } else {
        RCLCPP_WARN(get_logger(), "spawn_all: failed to spawn swath model %s", model_name.c_str());
      }

      // Advance the global counter past all segments in this swath.
      for (double a = start; a < end; a += seg_len_) { ++ctr; }
      ++sw_idx;
    }
    RCLCPP_INFO(get_logger(),
      "spawn_all: spawned %d swath models (%d links) across %zu swaths",
      total_models, total_links, swaths_.size());
    // No rolling timer needed — all boxes stay until node shuts down.
  }

  bool get_transforms(double & rx, double & ry, double & ryaw) {
    try {
      auto tr = tf_buffer_->lookupTransform(
        "map", "base_footprint", tf2::TimePointZero,
        tf2::durationFromSec(0.1));
      rx   = tr.transform.translation.x;
      ry   = tr.transform.translation.y;
      ryaw = quat_yaw(tr.transform.rotation.x, tr.transform.rotation.y,
                      tr.transform.rotation.z, tr.transform.rotation.w);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "TF lookup map->base_footprint failed: %s", e.what());
      return false;
    }
    return true;
  }

  const Swath * nearest_swath(double rx, double ry, double ryaw) {
    const Swath * best = nullptr;
    double best_perp = 1e9;
    for (auto & sw : swaths_) {
      double perp = std::abs((rx - sw.cx) * (-sw.uy) + (ry - sw.cy) * sw.ux);
      double dot  = std::cos(ryaw) * sw.ux + std::sin(ryaw) * sw.uy;
      if (std::abs(dot) < 0.5) continue;
      if (perp < best_perp) { best_perp = perp; best = &sw; }
    }
    return best;
  }

  static double along(const Swath & sw, double rx, double ry) {
    return (rx - sw.cx) * sw.ux + (ry - sw.cy) * sw.uy;
  }

  void remove_all() {
    std::lock_guard<std::mutex> lk(mtx_);
    for (auto & [name, _] : spawned_) {
      gz::msgs::Entity req;
      req.set_name(name);
      req.set_type(gz::msgs::Entity::MODEL);
      gz::msgs::Boolean rep;
      bool result = false;
      gz_node_.Request(remove_srv_, req, 500, rep, result);
      char buf[128];
      std::snprintf(buf, sizeof(buf), "{\"ev\":\"remove\",\"name\":\"%s\"}", name.c_str());
      std_msgs::msg::String emsg; emsg.data = buf;
      event_pub_->publish(emsg);
    }
    spawned_.clear();
    last_spawn_along_.reset();
  }

  void update() {
    double rx, ry, ryaw;
    if (!get_transforms(rx, ry, ryaw)) return;

    const Swath * sw = nearest_swath(rx, ry, ryaw);
    if (!sw) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "No swath matches robot pose (%.3f,%.3f,%.1f°) — heading not aligned "
        "with any swath axis (need |dot|>=0.5)",
        rx, ry, ryaw * 180.0 / M_PI);
      return;
    }

    // gz_offset = map_pose − gz_pose, constant world-frame translation between frames.
    // gz_yaw_offset = gz_yaw − map_yaw, refreshed every tick (typically ~0).
    if (!gz_pose_valid_.load()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "No Gazebo odometry yet on '%s' — skipping spawn",
        get_parameter("gz_odom_topic").as_string().c_str());
      return;
    }
    double gz_x, gz_y;
    {
      gz_x           = gz_pose_x_.load();
      gz_y           = gz_pose_y_.load();
      gz_yaw_offset_ = gz_pose_yaw_.load() - ryaw;
      gz_offset_x_   = rx - gz_x;
      gz_offset_y_   = ry - gz_y;
    }

    if (sw != current_swath_) {
      remove_all();
      current_swath_ = sw;
    }

    double a = along(*sw, rx, ry);

    // Remove segments that have fallen behind (signed by travel direction)
    double dot_for_removal = std::cos(ryaw) * sw->ux + std::sin(ryaw) * sw->uy;
    double ts = (dot_for_removal >= 0) ? 1.0 : -1.0;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      for (auto it = spawned_.begin(); it != spawned_.end(); ) {
        double lag = ts * (a - it->second.along);
        if (lag > behind_) {
          gz::msgs::Entity req;
          req.set_name(it->first);
          req.set_type(gz::msgs::Entity::MODEL);
          gz::msgs::Boolean rep;
          bool result = false;
          gz_node_.Request(remove_srv_, req, 500, rep, result);
          {
            char buf[128];
            std::snprintf(buf, sizeof(buf), "{\"ev\":\"remove\",\"name\":\"%s\"}", it->first.c_str());
            std_msgs::msg::String emsg; emsg.data = buf;
            event_pub_->publish(emsg);
          }
          it = spawned_.erase(it);
        } else { ++it; }
      }
    }

    // dot>0: robot travels in same direction as swath ux,uy → spawn ahead = a+ahead
    // dot<0: robot travels opposite                         → spawn ahead = a-ahead
    double dot = std::cos(ryaw) * sw->ux + std::sin(ryaw) * sw->uy;
    double travel_sign = (dot >= 0) ? 1.0 : -1.0;
    double spawn_a = a + travel_sign * ahead_;

    if (std::abs(spawn_a) > sw->length / 2 + seg_len_) return;
    if (last_spawn_along_ && std::abs(spawn_a - *last_spawn_along_) < min_move_) return;
    last_spawn_along_ = spawn_a;

    // Foot of perpendicular on swath at spawn_a, in map frame
    double seg_cx = sw->cx + spawn_a * sw->ux;
    double seg_cy = sw->cy + spawn_a * sw->uy;

    int ctr;
    { std::lock_guard<std::mutex> lk(mtx_); ctr = counter_++; }

    // gz_offset is the constant world-frame translation: map_pos − gz_pos.
    // Subtract it from the map-frame spawn position to get the gz world position.
    // gz_yaw_offset is applied to the row box orientation (typically ~0).
    double gz_row_yaw = sw->yaw + gz_yaw_offset_;

    for (int sign : {+1, -1}) {
      double wx = (seg_cx + sign * sw->offset * sw->px) - gz_offset_x_;
      double wy = (seg_cy + sign * sw->offset * sw->py) - gz_offset_y_;
      std::string name = "gcr_" + std::to_string(ctr) + (sign > 0 ? "_L" : "_R");

      gz::msgs::EntityFactory req;
      req.set_name(name);
      req.set_sdf(make_sdf(name, wx, wy, gz_row_yaw, seg_len_, width_, ctr));
      gz::msgs::Boolean rep;
      bool result = false;
      if (gz_node_.Request(spawn_srv_, req, 500, rep, result) && rep.data()) {
        std::lock_guard<std::mutex> lk(mtx_);
        spawned_[name] = {spawn_a};
        RCLCPP_DEBUG(get_logger(), "Spawned %s at gz=(%.2f,%.2f)", name.c_str(), wx, wy);
        // Publish spawn event for recorder
        double map_x = seg_cx + sign * sw->offset * sw->px;
        double map_y = seg_cy + sign * sw->offset * sw->py;
        char buf[512];
        std::snprintf(buf, sizeof(buf),
          "{\"ev\":\"spawn\",\"name\":\"%s\","
          "\"map_x\":%.4f,\"map_y\":%.4f,"
          "\"gz_x\":%.4f,\"gz_y\":%.4f,"
          "\"yaw\":%.4f,\"len\":%.3f,\"wid\":%.3f}",
          name.c_str(), map_x, map_y, wx, wy,
          gz_row_yaw, seg_len_, width_);
        std_msgs::msg::String emsg;
        emsg.data = buf;
        event_pub_->publish(emsg);
      } else {
        RCLCPP_WARN(get_logger(), "Failed to spawn %s", name.c_str());
      }
    }

    // Diagnostic
    int si = (int)(sw - swaths_.data());
    double perp = (rx - sw->cx) * (-sw->uy) + (ry - sw->cy) * sw->ux;
    double sw_yaw_deg = sw->yaw * 180.0 / M_PI;
    double robot_yaw_deg = ryaw * 180.0 / M_PI;
    RCLCPP_INFO(get_logger(),
      "[DIAG] swath=%d along=%.2f perp=%.3f travel_sign=%.0f "
      "robot_map=(%.3f,%.3f,%.1f°) robot_gz=(%.3f,%.3f) "
      "sw_yaw=%.1f° gz_row_yaw=%.1f° dot=%.3f "
      "gz_off=(%.3f,%.3f) gz_yaw_off=%.1f°",
      si, a, perp, travel_sign,
      rx, ry, robot_yaw_deg,
      gz_x, gz_y,
      sw_yaw_deg, gz_row_yaw * 180.0 / M_PI, dot,
      gz_offset_x_, gz_offset_y_,
      gz_yaw_offset_ * 180.0 / M_PI);
  }

  std::string field_file_, world_, spawn_srv_, remove_srv_;
  double ahead_, behind_, row_offset_, width_, seg_len_, min_move_;
  bool spawn_all_ = false;
  double gz_offset_x_ = 0.0, gz_offset_y_ = 0.0, gz_yaw_offset_ = 0.0;
  // Cached Gazebo world pose from bridged odometry topic (lock-free)
  std::atomic<double> gz_pose_x_{0.0}, gz_pose_y_{0.0}, gz_pose_yaw_{0.0};
  std::atomic<bool>   gz_pose_valid_{false};
  std::vector<Swath> swaths_;
  std::unordered_map<std::string, Segment> spawned_;
  std::mutex mtx_;
  int counter_ = 0;
  std::optional<double> last_spawn_along_;
  const Swath * current_swath_ = nullptr;

  struct Point2d { double x, y; };
  struct RawSwath { double lon0, lat0, lon1, lat1; };
  std::vector<RawSwath> pending_raw_;
  std::vector<Point2d> converted_;
  size_t pending_idx_ = 0;

  rclcpp::Client<robot_localization::srv::FromLL>::SharedPtr from_ll_cli_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gz_odom_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_pub_;
  gz::transport::Node gz_node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr spawn_all_pending_timer_;
  rclcpp::TimerBase::SharedPtr swath_load_retry_timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CropRowSpawner>());
  rclcpp::shutdown();
  return 0;
}
