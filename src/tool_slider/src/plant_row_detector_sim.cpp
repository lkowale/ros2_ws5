// plant_row_detector_sim — simulates a vision-based plant row detector.
//
// Publishes /plant_row_offset (std_msgs/Float32, meters, signed: +right)
// at 30 Hz using a configurable sine wave to simulate robot drift relative
// to the planted row.
//
// Parameters:
//   mode          "sine" | "static"   (default: "sine")
//   static_offset  meters             (default: 0.0, used in "static" mode)
//   sine_amplitude meters             (default: 0.04)
//   sine_period_s  seconds            (default: 8.0)
//   detection_rate_hz                 (default: 30.0)
//   row_lost_after_s  seconds of no detection to publish NaN (default: -1 = never lost)

#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

class PlantRowDetectorSim : public rclcpp::Node
{
public:
  PlantRowDetectorSim()
  : rclcpp::Node("plant_row_detector_sim")
  {
    declare_parameter("mode", "sine");
    declare_parameter("static_offset", 0.0);
    declare_parameter("sine_amplitude", 0.04);
    declare_parameter("sine_period_s", 8.0);
    declare_parameter("detection_rate_hz", 30.0);

    mode_           = get_parameter("mode").as_string();
    static_offset_  = get_parameter("static_offset").as_double();
    amplitude_      = get_parameter("sine_amplitude").as_double();
    period_         = get_parameter("sine_period_s").as_double();
    double rate_hz  = get_parameter("detection_rate_hz").as_double();

    pub_ = create_publisher<std_msgs::msg::Float32>("plant_row_offset", 10);

    double dt = 1.0 / rate_hz;
    timer_ = create_wall_timer(
      std::chrono::duration<double>(dt),
      std::bind(&PlantRowDetectorSim::tick, this));

    RCLCPP_INFO(get_logger(),
      "plant_row_detector_sim: mode=%s amplitude=%.3fm period=%.1fs rate=%.0fHz",
      mode_.c_str(), amplitude_, period_, rate_hz);
  }

private:
  void tick()
  {
    double t = now().seconds();
    double offset = 0.0;

    if (mode_ == "sine") {
      offset = amplitude_ * std::sin(2.0 * M_PI * t / period_);
    } else {
      offset = static_offset_;
    }

    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(offset);
    pub_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string mode_;
  double static_offset_;
  double amplitude_;
  double period_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlantRowDetectorSim>());
  rclcpp::shutdown();
  return 0;
}
