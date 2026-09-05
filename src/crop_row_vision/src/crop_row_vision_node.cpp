// crop_row_vision_node — camera-based crop row + tool implement detector.
//
// Subscribes to the downward-facing OAK-D image, segments the two green crop
// rows straddling image center and the blue implement_center marker, and
// publishes the lateral offset needed to move the toolbar so implement_center
// sits on the row midpoint — a true visual tracking error, not an open-loop
// row-only measurement.
//
// implement_center is blue while implement_left/right stay red (see
// description.urdf) specifically so this node can identify it unambiguously
// by color. Picking "the red blob nearest image center" among 3
// identically-colored markers used to flip-flop between implement_center and
// implement_left/right frame-to-frame, since all three can sit close to the
// image midline — that produced large, physically-impossible jumps in the
// published offset (confirmed via crop_row_vision/diagnostics: implement pixel
// position jumping ~150-290px between consecutive 5 Hz frames while the row
// blobs moved smoothly). Do not make implement_left/right blue too.
//
// Subscribes:
//   /oakd/rgb/image  (sensor_msgs/Image, bgr8) — downward camera, straight
//                     boresight (oakd_joint rpy="0 M_PI/2 0"). DO NOT reinterpret
//                     axes without re-deriving _pixel_to_body(): image top is
//                     +X_body (forward), image right is +Y_body (left of robot) —
//                     see grid_decoder.py's empirically-verified convention.
//
// Publishes:
//   /plant_row_offset  (std_msgs/Float32, meters, +right; NaN = rows not found)
//                       Drop-in replacement for plant_row_detector_sim's topic —
//                       tool_slider_controller is unchanged downstream.
//   /crop_row_vision/diagnostics  (std_msgs/String, JSON, published every frame)
//                       Per-frame blob positions for offline analysis: every
//                       detected green/implement blob's (cx,cy,area) in pixel
//                       coords, the chosen row_mid_px / implement_px (null if
//                       not found), and img_w/img_h.
//
// Detection:
//   1. Threshold HSV for green (rows, crop_row_spawner's fixed color) and blue
//      (implement_center marker) blobs.
//   2. Row centroids: take the two green blob columns straddling image-center
//      column (one left-of-center, one right-of-center by pixel u); their
//      midpoint pixel column is the target row-center track.
//   3. Implement centroid: the (normally single) blue blob. If more than one
//      is ever detected, pick whichever is closest to the previous frame's
//      accepted position (falls back to image-center on the first frame) —
//      temporal continuity as a safety net against spurious blobs.
//   4. offset = body_y(row_midpoint) - body_y(implement) via ground-plane pixel
//      scale (height * tan(HFOV/2) per half-image-width); sign flipped to match
//      the existing +right convention.
//
// Parameters:
//   camera_height_m     camera height above ground (default: 0.80, oakd_joint z)
//   camera_hfov_rad      horizontal FOV (default: 1.414, OAK-D Lite RGB)
//   row_lost_after_frames  consecutive failed detections before publishing NaN
//                          (default: 5)
//   green_lo, green_hi           HSV threshold range for crop rows
//   implement_lo, implement_hi   HSV threshold range for implement_center (blue)
//   min_blob_area_px     minimum contour area to accept a detection (default: 6)
//   debug_publish        publish annotated debug image (default: false)

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "image_transport/image_transport.hpp"
#include "opencv2/opencv.hpp"

namespace
{

struct Blob
{
  double cx;
  double cy;
  double area;
};

std::vector<Blob> find_blobs(
  const cv::Mat & hsv, const cv::Scalar & lo, const cv::Scalar & hi, double min_area)
{
  cv::Mat mask;
  cv::inRange(hsv, lo, hi, mask);
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, cv::Mat(), cv::Point(-1, -1), 1);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  std::vector<Blob> blobs;
  for (const auto & c : contours) {
    double area = cv::contourArea(c);
    if (area < min_area) continue;
    cv::Moments m = cv::moments(c);
    if (m.m00 <= 0.0) continue;
    blobs.push_back({m.m10 / m.m00, m.m01 / m.m00, area});
  }
  return blobs;
}

}  // namespace

class CropRowVisionNode : public rclcpp::Node
{
public:
  CropRowVisionNode()
  : rclcpp::Node("crop_row_vision_node"), miss_count_(0)
  {
    declare_parameter("camera_height_m", 0.80);
    declare_parameter("camera_hfov_rad", 1.414);
    declare_parameter("row_lost_after_frames", 5);
    declare_parameter("min_blob_area_px", 6.0);
    declare_parameter("debug_publish", false);

    declare_parameter("green_lo", std::vector<int64_t>{35, 40, 40});
    declare_parameter("green_hi", std::vector<int64_t>{85, 255, 255});
    // implement_center is blue (rgba 0.0,0.2,1.0) — see description.urdf.
    // implement_left/right stay red and are intentionally ignored.
    declare_parameter("implement_lo", std::vector<int64_t>{100, 80, 60});
    declare_parameter("implement_hi", std::vector<int64_t>{130, 255, 255});

    cam_height_ = get_parameter("camera_height_m").as_double();
    cam_hfov_   = get_parameter("camera_hfov_rad").as_double();
    lost_after_ = static_cast<int>(get_parameter("row_lost_after_frames").as_int());
    min_area_   = get_parameter("min_blob_area_px").as_double();
    debug_      = get_parameter("debug_publish").as_bool();

    auto to_scalar = [](const std::vector<int64_t> & v) {
      return cv::Scalar(static_cast<double>(v[0]), static_cast<double>(v[1]),
                         static_cast<double>(v[2]));
    };
    green_lo_ = to_scalar(get_parameter("green_lo").as_integer_array());
    green_hi_ = to_scalar(get_parameter("green_hi").as_integer_array());
    implement_lo_ = to_scalar(get_parameter("implement_lo").as_integer_array());
    implement_hi_ = to_scalar(get_parameter("implement_hi").as_integer_array());

    pub_offset_ = create_publisher<std_msgs::msg::Float32>("plant_row_offset", 10);
    pub_diag_   = create_publisher<std_msgs::msg::String>("crop_row_vision/diagnostics", 10);

    RCLCPP_INFO(get_logger(),
      "crop_row_vision_node: height=%.2fm hfov=%.3frad lost_after=%d frames",
      cam_height_, cam_hfov_, lost_after_);
  }

  // Must be called once, after construction (image_transport needs
  // shared_from_this(), which is unavailable inside the constructor).
  void init()
  {
    it_ = std::make_unique<image_transport::ImageTransport>(shared_from_this());
    sub_image_ = it_->subscribe(
      "oakd/rgb/image", 1,
      std::bind(&CropRowVisionNode::on_image, this, std::placeholders::_1));

    if (debug_) {
      pub_debug_ = it_->advertise("crop_row_vision/debug_image", 1);
    }
  }

private:
  // Ground-plane pixel->body-Y scale, straight-down boresight.
  // image right (+u) -> +Y_body (left of robot); see grid_decoder.py.
  double px_to_body_y(double px, int img_w) const
  {
    double norm_u = px / static_cast<double>(img_w);
    double scale_h = cam_height_ * std::tan(cam_hfov_ / 2.0);  // m per half-width
    return (norm_u - 0.5) * 2.0 * scale_h;
  }

  std::optional<std::pair<double, double>> find_row_straddle(
    const std::vector<Blob> & greens, int img_w) const
  {
    double center_u = img_w / 2.0;
    const Blob * left = nullptr;
    const Blob * right = nullptr;
    double best_left = -1.0, best_right = 1e18;

    for (const auto & b : greens) {
      if (b.cx < center_u && b.cx > best_left) { best_left = b.cx; left = &b; }
      if (b.cx >= center_u && b.cx < best_right) { best_right = b.cx; right = &b; }
    }
    if (!left || !right) return std::nullopt;
    return std::make_pair(left->cx, right->cx);
  }

  // implement_center is blue — implement_left/right are red and never appear
  // in `implements`, so normally there is exactly one blob here. If more than
  // one ever appears (noise, reflection), track the previous frame's accepted
  // position instead of re-picking by image-center each time — that heuristic
  // is what caused the vision node to flip between different physical markers
  // when they were all the same color (see tool_slider oscillation writeup).
  std::optional<Blob> find_implement_center(const std::vector<Blob> & implements)
  {
    if (implements.empty()) return std::nullopt;
    if (implements.size() == 1) {
      last_implement_cx_ = implements[0].cx;
      return implements[0];
    }
    double ref = last_implement_cx_.value_or(implements[0].cx);
    const Blob * best = nullptr;
    double best_dist = 1e18;
    for (const auto & b : implements) {
      double d = std::abs(b.cx - ref);
      if (d < best_dist) { best_dist = d; best = &b; }
    }
    last_implement_cx_ = best->cx;
    return *best;
  }

  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    const cv::Mat & bgr = cv_ptr->image;
    int img_w = bgr.cols;
    int img_h = bgr.rows;

    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

    auto greens = find_blobs(hsv, green_lo_, green_hi_, min_area_);
    auto implements = find_blobs(hsv, implement_lo_, implement_hi_, min_area_);

    auto straddle = find_row_straddle(greens, img_w);
    auto implement = find_implement_center(implements);

    std_msgs::msg::Float32 out;

    if (!straddle || !implement) {
      ++miss_count_;
      if (miss_count_ >= lost_after_) {
        out.data = std::numeric_limits<float>::quiet_NaN();
        pub_offset_->publish(out);
        last_implement_cx_.reset();
      }
      publish_debug(bgr, greens, implements, std::nullopt, std::nullopt);
      publish_diagnostics(greens, implements, img_w, img_h, std::nullopt, std::nullopt, std::nullopt);
      return;
    }
    miss_count_ = 0;

    double row_mid_px = (straddle->first + straddle->second) / 2.0;
    double row_mid_y   = px_to_body_y(row_mid_px, img_w);
    double impl_y       = px_to_body_y(implement->cx, img_w);

    // px_to_body_y is +left; offset contract is +right, so flip sign.
    double offset_right = -(row_mid_y - impl_y);

    out.data = static_cast<float>(offset_right);
    pub_offset_->publish(out);

    publish_debug(bgr, greens, implements, row_mid_px, implement->cx);
    publish_diagnostics(greens, implements, img_w, img_h, row_mid_px, implement->cx, offset_right);
  }

  // JSON diagnostics: per-frame blob pixel positions, for offline analysis of
  // whether detections actually change frame-to-frame or are stuck.
  void publish_diagnostics(
    const std::vector<Blob> & greens, const std::vector<Blob> & implements,
    int img_w, int img_h,
    std::optional<double> row_mid_px, std::optional<double> implement_px,
    std::optional<double> offset_right)
  {
    if (pub_diag_->get_subscription_count() == 0) return;

    auto blob_array = [](const std::vector<Blob> & blobs) {
      std::string s = "[";
      for (size_t i = 0; i < blobs.size(); ++i) {
        if (i) s += ",";
        char buf[96];
        std::snprintf(buf, sizeof(buf), "{\"cx\":%.2f,\"cy\":%.2f,\"area\":%.1f}",
                      blobs[i].cx, blobs[i].cy, blobs[i].area);
        s += buf;
      }
      s += "]";
      return s;
    };

    auto opt_num = [](std::optional<double> v) {
      if (!v) return std::string("null");
      char buf[32];
      std::snprintf(buf, sizeof(buf), "%.4f", *v);
      return std::string(buf);
    };

    char head[128];
    std::snprintf(head, sizeof(head), "{\"img_w\":%d,\"img_h\":%d,", img_w, img_h);

    std_msgs::msg::String msg;
    msg.data = std::string(head) +
      "\"row_mid_px\":" + opt_num(row_mid_px) + "," +
      "\"implement_px\":" + opt_num(implement_px) + "," +
      "\"offset_right\":" + opt_num(offset_right) + "," +
      "\"greens\":" + blob_array(greens) + "," +
      "\"implements\":" + blob_array(implements) + "}";
    pub_diag_->publish(msg);
  }

  void publish_debug(
    const cv::Mat & bgr, const std::vector<Blob> & greens, const std::vector<Blob> & implements,
    std::optional<double> row_mid_px, std::optional<double> implement_px)
  {
    if (!debug_) return;
    cv::Mat dbg = bgr.clone();
    for (const auto & b : greens) {
      cv::circle(dbg, cv::Point(static_cast<int>(b.cx), static_cast<int>(b.cy)), 4,
                 cv::Scalar(0, 255, 0), 2);
    }
    for (const auto & b : implements) {
      cv::circle(dbg, cv::Point(static_cast<int>(b.cx), static_cast<int>(b.cy)), 4,
                 cv::Scalar(255, 0, 0), 2);
    }
    if (row_mid_px) {
      cv::line(dbg, cv::Point(static_cast<int>(*row_mid_px), 0),
                cv::Point(static_cast<int>(*row_mid_px), dbg.rows), cv::Scalar(255, 255, 0), 1);
    }
    if (implement_px) {
      cv::line(dbg, cv::Point(static_cast<int>(*implement_px), 0),
                cv::Point(static_cast<int>(*implement_px), dbg.rows), cv::Scalar(0, 255, 255), 1);
    }
    auto out_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", dbg).toImageMsg();
    pub_debug_.publish(out_msg);
  }

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_offset_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_diag_;
  std::unique_ptr<image_transport::ImageTransport> it_;
  image_transport::Subscriber sub_image_;
  image_transport::Publisher pub_debug_;

  double cam_height_;
  double cam_hfov_;
  int lost_after_;
  double min_area_;
  bool debug_;
  int miss_count_;
  std::optional<double> last_implement_cx_;

  cv::Scalar green_lo_, green_hi_;
  cv::Scalar implement_lo_, implement_hi_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CropRowVisionNode>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
