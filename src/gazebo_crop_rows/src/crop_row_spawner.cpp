#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <robot_localization/srv/from_ll.hpp>

#include <gz/transport/Node.hh>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/boolean.pb.h>

#include <nlohmann/json.hpp>

#include <cmath>
#include <cstdio>
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

static std::string make_sdf(const std::string & name,
                             double x, double y, double yaw,
                             double length, double width)
{
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
    "<ambient>0.13 0.55 0.13 1</ambient>"
    "<diffuse>0.13 0.55 0.13 1</diffuse>"
    "<specular>0 0 0 1</specular>"
    "</material>"
    "</visual>"
    "</link>"
    "</model>"
    "</sdf>",
    name.c_str(), x, y, yaw, length, width);
  return buf;
}

class CropRowSpawner : public rclcpp::Node {
public:
  CropRowSpawner() : Node("crop_row_spawner") {
    declare_parameter("field_file", "");
    declare_parameter("world_name", "house_short_crop_rows");
    declare_parameter("robot_model", "solbot5");
    declare_parameter("spawn_ahead_m", 4.0);
    declare_parameter("remove_behind_m", 3.0);
    declare_parameter("row_offset_m", 0.18);
    declare_parameter("row_width_m", 0.06);
    declare_parameter("segment_length_m", 2.0);
    declare_parameter("min_move_m", 1.5);

    field_file_   = get_parameter("field_file").as_string();
    world_        = get_parameter("world_name").as_string();
    robot_model_  = get_parameter("robot_model").as_string();
    ahead_        = get_parameter("spawn_ahead_m").as_double();
    behind_       = get_parameter("remove_behind_m").as_double();
    row_offset_   = get_parameter("row_offset_m").as_double();
    width_        = get_parameter("row_width_m").as_double();
    seg_len_      = get_parameter("segment_length_m").as_double();
    min_move_     = get_parameter("min_move_m").as_double();

    if (field_file_.empty()) {
      RCLCPP_ERROR(get_logger(), "field_file parameter not set");
      return;
    }

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    from_ll_cli_ = create_client<robot_localization::srv::FromLL>("/fromLL");

    spawn_srv_  = "/world/" + world_ + "/create";
    remove_srv_ = "/world/" + world_ + "/remove";

    // Wait for /fromLL then load swaths using it for coordinate conversion
    RCLCPP_INFO(get_logger(), "Waiting for /fromLL service...");
    init_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&CropRowSpawner::try_init, this));
  }

private:
  // Returns (x, y, yaw_rad) of a Gazebo model via gz CLI, or false on failure.
  bool gz_model_pose(const std::string & model, double & x, double & y, double & yaw) {
    std::string cmd = "gz model -m " + model + " -p 2>/dev/null";
    FILE * fp = popen(cmd.c_str(), "r");
    if (!fp) return false;
    char buf[256];
    bool found_xyz_header = false;
    bool got_xyz = false;
    double lx = 0, ly = 0, lyaw = 0;
    while (fgets(buf, sizeof(buf), fp)) {
      std::string s(buf);
      // trim leading whitespace
      auto pos = s.find_first_not_of(" \t\r\n");
      if (pos != std::string::npos) s = s.substr(pos);
      if (s.find("XYZ") != std::string::npos) { found_xyz_header = true; continue; }
      if (found_xyz_header && !got_xyz && s.size() > 1 && s[0] == '[') {
        if (std::sscanf(s.c_str(), "[%lf %lf", &lx, &ly) == 2) got_xyz = true;
        continue;
      }
      if (got_xyz && s.find("RPY") != std::string::npos && s.find("rad") != std::string::npos) continue;
      if (got_xyz && s.size() > 1 && s[0] == '[') {
        double r = 0, p = 0, yw = 0;
        if (std::sscanf(s.c_str(), "[%lf %lf %lf]", &r, &p, &yw) == 3) {
          lyaw = yw;
          x = lx; y = ly; yaw = lyaw;
          pclose(fp);
          return true;
        }
      }
    }
    pclose(fp);
    return false;
  }

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
    }
    RCLCPP_INFO(get_logger(), "Loaded %zu swaths (map-frame coords via /fromLL)",
                swaths_.size());
    timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&CropRowSpawner::update, this));
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
    } catch (...) {
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
    }
    spawned_.clear();
    last_spawn_along_.reset();
  }

  void update() {
    double rx, ry, ryaw;
    if (!get_transforms(rx, ry, ryaw)) return;

    const Swath * sw = nearest_swath(rx, ry, ryaw);
    if (!sw) return;

    // Refresh gz_offset and gz_yaw_offset every tick from live Gazebo pose.
    // gz_offset = map_pose - gz_world_pose  (XY translation)
    // gz_yaw_offset = gz_yaw - map_yaw      (yaw rotation to apply to spawned box)
    {
      double gx, gy, gyaw_gz;
      if (!gz_model_pose(robot_model_, gx, gy, gyaw_gz)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "Cannot get Gazebo pose of '%s' — skipping spawn", robot_model_.c_str());
        return;
      }
      gz_offset_x_   = rx - gx;
      gz_offset_y_   = ry - gy;
      gz_yaw_offset_ = gyaw_gz - ryaw;   // radians; wrap not needed, used in trig
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

    // Row yaw in Gazebo world frame = map-frame swath yaw + gz_yaw_offset
    double gz_row_yaw = sw->yaw + gz_yaw_offset_;

    for (int sign : {+1, -1}) {
      // Convert map-frame spawn position to Gazebo world frame
      double wx = (seg_cx + sign * sw->offset * sw->px) - gz_offset_x_;
      double wy = (seg_cy + sign * sw->offset * sw->py) - gz_offset_y_;
      std::string name = "gcr_" + std::to_string(ctr) + (sign > 0 ? "_L" : "_R");

      gz::msgs::EntityFactory req;
      req.set_name(name);
      req.set_sdf(make_sdf(name, wx, wy, gz_row_yaw, seg_len_, width_));
      gz::msgs::Boolean rep;
      bool result = false;
      if (gz_node_.Request(spawn_srv_, req, 500, rep, result) && rep.data()) {
        std::lock_guard<std::mutex> lk(mtx_);
        spawned_[name] = {spawn_a};
        RCLCPP_DEBUG(get_logger(), "Spawned %s at gz=(%.2f,%.2f)", name.c_str(), wx, wy);
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
      "robot_map=(%.3f,%.3f,%.1f°) sw_yaw=%.1f° gz_row_yaw=%.1f° dot=%.3f "
      "spawn_gz=(%.3f,%.3f) gz_offset=(%.3f,%.3f) gz_yaw_off=%.1f°",
      si, a, perp, travel_sign,
      rx, ry, robot_yaw_deg, sw_yaw_deg,
      gz_row_yaw * 180.0 / M_PI, dot,
      seg_cx - gz_offset_x_, seg_cy - gz_offset_y_,
      gz_offset_x_, gz_offset_y_,
      gz_yaw_offset_ * 180.0 / M_PI);
  }

  std::string field_file_, world_, robot_model_, spawn_srv_, remove_srv_;
  double ahead_, behind_, row_offset_, width_, seg_len_, min_move_;
  double gz_offset_x_ = 0.0, gz_offset_y_ = 0.0, gz_yaw_offset_ = 0.0;
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
  gz::transport::Node gz_node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CropRowSpawner>());
  rclcpp::shutdown();
  return 0;
}
