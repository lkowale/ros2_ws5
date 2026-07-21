#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <gz/transport/Node.hh>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/boolean.pb.h>

#include <nlohmann/json.hpp>

#include <cmath>
#include <fstream>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

static constexpr double WGS84_A = 6378137.0;

struct Swath {
  double cx, cy;
  double ux, uy;   // unit along-swath
  double px, py;   // unit perpendicular (left)
  double yaw;
  double length;
  double offset;
};

struct Segment {
  double along;  // along-swath position when spawned
};

static double wgs84_x(double lon, double lat, double dlon, double dlat) {
  return (lon - dlon) * M_PI / 180.0 * WGS84_A * std::cos(dlat * M_PI / 180.0);
}
static double wgs84_y(double lat, double dlat) {
  return (lat - dlat) * M_PI / 180.0 * WGS84_A;
}

static double norm_angle(double a) {
  while (a >  M_PI / 2) a -= M_PI;
  while (a <= -M_PI / 2) a += M_PI;
  return a;
}

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
    declare_parameter("spawn_ahead_m", 4.0);
    declare_parameter("remove_behind_m", 3.0);
    declare_parameter("row_offset_m", 0.18);
    declare_parameter("row_width_m", 0.06);
    declare_parameter("segment_length_m", 2.0);
    declare_parameter("min_move_m", 1.5);
    declare_parameter("datum_lat", 53.5204991);
    declare_parameter("datum_lon", 17.8258532);

    auto field_file   = get_parameter("field_file").as_string();
    world_            = get_parameter("world_name").as_string();
    ahead_            = get_parameter("spawn_ahead_m").as_double();
    behind_           = get_parameter("remove_behind_m").as_double();
    width_            = get_parameter("row_width_m").as_double();
    seg_len_          = get_parameter("segment_length_m").as_double();
    min_move_         = get_parameter("min_move_m").as_double();
    double datum_lat  = get_parameter("datum_lat").as_double();
    double datum_lon  = get_parameter("datum_lon").as_double();

    if (field_file.empty()) {
      RCLCPP_ERROR(get_logger(), "field_file parameter not set");
      return;
    }

    load_swaths(field_file, datum_lon, datum_lat,
                get_parameter("row_offset_m").as_double());
    RCLCPP_INFO(get_logger(), "Loaded %zu swaths from %s",
                swaths_.size(), field_file.c_str());

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    spawn_srv_  = "/world/" + world_ + "/create";
    remove_srv_ = "/world/" + world_ + "/remove";

    timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&CropRowSpawner::update, this));
  }

private:
  void load_swaths(const std::string & path, double dlon, double dlat, double offset) {
    std::ifstream f(path);
    auto d = nlohmann::json::parse(f);
    for (auto & feat : d["features"]) {
      auto & coords = feat["geometry"]["coordinates"];
      double x0 = wgs84_x(coords[0][0], coords[0][1], dlon, dlat);
      double y0 = wgs84_y(coords[0][1], dlat);
      double x1 = wgs84_x(coords[1][0], coords[1][1], dlon, dlat);
      double y1 = wgs84_y(coords[1][1], dlat);
      double dx = x1 - x0, dy = y1 - y0;
      double len = std::hypot(dx, dy);
      Swath sw;
      sw.ux = dx / len; sw.uy = dy / len;
      sw.px = -sw.uy;   sw.py = sw.ux;
      sw.yaw = norm_angle(std::atan2(dy, dx));
      sw.cx = (x0 + x1) / 2; sw.cy = (y0 + y1) / 2;
      sw.length = len; sw.offset = offset;
      swaths_.push_back(sw);
    }
  }

  bool get_transforms(double & rx, double & ry, double & ryaw,
                      double & mo_tx, double & mo_ty, double & mo_yaw) {
    try {
      auto tr = tf_buffer_->lookupTransform(
        "map", "base_footprint", tf2::TimePointZero,
        tf2::durationFromSec(0.1));
      rx   = tr.transform.translation.x;
      ry   = tr.transform.translation.y;
      ryaw = quat_yaw(tr.transform.rotation.x, tr.transform.rotation.y,
                      tr.transform.rotation.z, tr.transform.rotation.w);

      auto tm = tf_buffer_->lookupTransform(
        "odom", "map", tf2::TimePointZero,
        tf2::durationFromSec(0.1));
      mo_tx  = tm.transform.translation.x;
      mo_ty  = tm.transform.translation.y;
      mo_yaw = quat_yaw(tm.transform.rotation.x, tm.transform.rotation.y,
                        tm.transform.rotation.z, tm.transform.rotation.w);
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

  void map_to_world(double mx, double my,
                    double tx, double ty, double yaw,
                    double & wx, double & wy) {
    wx = tx + mx * std::cos(yaw) - my * std::sin(yaw);
    wy = ty + mx * std::sin(yaw) + my * std::cos(yaw);
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
    double rx, ry, ryaw, mo_tx, mo_ty, mo_yaw;
    if (!get_transforms(rx, ry, ryaw, mo_tx, mo_ty, mo_yaw)) return;

    const Swath * sw = nearest_swath(rx, ry, ryaw);
    if (!sw) return;

    // Swath changed — remove all segments from previous swath
    if (sw != current_swath_) {
      remove_all();
      current_swath_ = sw;
    }

    double a = along(*sw, rx, ry);

    // --- Diagnostic log every ~2 s (4 ticks at 0.5 Hz) ---
    if (++diag_tick_ % 4 == 0) {
      // Swath index
      int si = (int)(sw - swaths_.data());
      // Row positions in map frame (what RL sees)
      double perp_dist = (rx - sw->cx) * (-sw->uy) + (ry - sw->cy) * sw->ux;
      double row_L_mx = rx + sw->offset * sw->px;
      double row_L_my = ry + sw->offset * sw->py;
      double row_R_mx = rx - sw->offset * sw->px;
      double row_R_my = ry - sw->offset * sw->py;
      // Same points in Gazebo world frame (odom)
      double row_L_wx, row_L_wy, row_R_wx, row_R_wy;
      map_to_world(row_L_mx, row_L_my, mo_tx, mo_ty, mo_yaw, row_L_wx, row_L_wy);
      map_to_world(row_R_mx, row_R_my, mo_tx, mo_ty, mo_yaw, row_R_wx, row_R_wy);
      RCLCPP_INFO(get_logger(),
        "[DIAG] swath=%d along=%.2f perp=%.3f | "
        "robot map=(%.3f,%.3f) yaw=%.2f | "
        "odom->map tx=(%.3f,%.3f) yaw=%.3f | "
        "rowL map=(%.3f,%.3f) gz=(%.3f,%.3f) | "
        "rowR map=(%.3f,%.3f) gz=(%.3f,%.3f)",
        si, a, perp_dist,
        rx, ry, ryaw,
        mo_tx, mo_ty, mo_yaw,
        row_L_mx, row_L_my, row_L_wx, row_L_wy,
        row_R_mx, row_R_my, row_R_wx, row_R_wy);
    }

    // Remove segments that have fallen behind
    {
      std::lock_guard<std::mutex> lk(mtx_);
      for (auto it = spawned_.begin(); it != spawned_.end(); ) {
        if (a - it->second.along > behind_) {
          gz::msgs::Entity req;
          req.set_name(it->first);
          req.set_type(gz::msgs::Entity::MODEL);
          gz::msgs::Boolean rep;
          bool result = false;
          gz_node_.Request(remove_srv_, req, 500, rep, result);
          it = spawned_.erase(it);
        } else {
          ++it;
        }
      }
    }

    double spawn_a = a + ahead_;
    if (std::abs(spawn_a) > sw->length / 2 + seg_len_) return;

    // Throttle by min_move_m
    if (last_spawn_along_ && std::abs(spawn_a - *last_spawn_along_) < min_move_) return;
    last_spawn_along_ = spawn_a;

    double seg_cx = sw->cx + spawn_a * sw->ux;
    double seg_cy = sw->cy + spawn_a * sw->uy;

    int ctr;
    { std::lock_guard<std::mutex> lk(mtx_); ctr = counter_++; }

    for (int sign : {+1, -1}) {
      double mx = seg_cx + sign * sw->offset * sw->px;
      double my = seg_cy + sign * sw->offset * sw->py;
      double wx, wy;
      map_to_world(mx, my, mo_tx, mo_ty, mo_yaw, wx, wy);

      std::string name = "gcr_" + std::to_string(ctr) + (sign > 0 ? "_L" : "_R");

      gz::msgs::EntityFactory req;
      req.set_name(name);
      req.set_sdf(make_sdf(name, wx, wy, sw->yaw, seg_len_, width_));
      gz::msgs::Boolean rep;
      bool result = false;
      if (gz_node_.Request(spawn_srv_, req, 500, rep, result) && rep.data()) {
        std::lock_guard<std::mutex> lk(mtx_);
        spawned_[name] = {spawn_a};
        RCLCPP_DEBUG(get_logger(), "Spawned %s", name.c_str());
      } else {
        RCLCPP_WARN(get_logger(), "Failed to spawn %s", name.c_str());
      }
    }
  }

  std::string world_, spawn_srv_, remove_srv_;
  double ahead_, behind_, width_, seg_len_, min_move_;
  std::vector<Swath> swaths_;
  std::unordered_map<std::string, Segment> spawned_;
  std::mutex mtx_;
  int counter_ = 0;
  std::optional<double> last_spawn_along_;
  const Swath * current_swath_ = nullptr;
  int diag_tick_ = 0;

  gz::transport::Node gz_node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CropRowSpawner>());
  rclcpp::shutdown();
  return 0;
}
