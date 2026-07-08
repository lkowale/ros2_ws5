// tool_slider_controller — vision-guided toolbar position controller.
//
// Subscribes to:
//   /plant_row_offset  (std_msgs/Float32, meters) — lateral offset of detected
//                       row center relative to camera optical axis; NaN = row lost
//
// Publishes:
//   /tool_bar_position_cmd  (std_msgs/Float32, meters from center, +right)
//
// The command is a direct feedforward from vision: cmd = clamp(offset + cal_offset, ±limit).
// When row is lost (NaN received or no message for timeout_s), the pause action fires:
//   - freezes last valid command
//   - publishes /tool_slider/row_lost (std_msgs/Bool, true)
//
// Parameters:
//   cal_offset_m      camera-to-toolbar lateral calibration offset (default: 0.0)
//   slider_limit_m    mechanical half-travel limit (default: 0.10)
//   row_lost_timeout_s  seconds without valid detection before pause (default: 0.5)

#include <chrono>
#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

class ToolSliderController : public rclcpp::Node
{
public:
  ToolSliderController()
  : rclcpp::Node("tool_slider_controller"),
    last_cmd_(0.0f),
    row_lost_(false)
  {
    declare_parameter("cal_offset_m", 0.0);
    declare_parameter("slider_limit_m", 0.10);
    declare_parameter("row_lost_timeout_s", 0.5);

    cal_offset_ = static_cast<float>(get_parameter("cal_offset_m").as_double());
    limit_      = static_cast<float>(get_parameter("slider_limit_m").as_double());
    timeout_s_  = get_parameter("row_lost_timeout_s").as_double();

    sub_ = create_subscription<std_msgs::msg::Float32>(
      "plant_row_offset", 10,
      std::bind(&ToolSliderController::cb_offset, this, std::placeholders::_1));

    pub_cmd_      = create_publisher<std_msgs::msg::Float32>("tool_bar_position_cmd", 10);
    pub_row_lost_ = create_publisher<std_msgs::msg::Bool>("tool_slider/row_lost", 10);

    // watchdog at 10 Hz
    watchdog_ = create_wall_timer(100ms,
      std::bind(&ToolSliderController::watchdog, this));

    last_msg_time_ = now();

    RCLCPP_INFO(get_logger(),
      "tool_slider_controller: cal=%.3fm limit=±%.3fm timeout=%.2fs",
      cal_offset_, limit_, timeout_s_);
  }

private:
  void cb_offset(const std_msgs::msg::Float32::SharedPtr msg)
  {
    last_msg_time_ = now();

    if (std::isnan(msg->data)) {
      handle_row_lost();
      return;
    }

    float cmd = msg->data + cal_offset_;
    // clamp to mechanical limits
    cmd = std::max(-limit_, std::min(limit_, cmd));
    last_cmd_ = cmd;

    if (row_lost_) {
      row_lost_ = false;
      RCLCPP_INFO(get_logger(), "Row re-acquired — resuming slider control");
      publish_row_lost(false);
    }

    std_msgs::msg::Float32 out;
    out.data = cmd;
    pub_cmd_->publish(out);
  }

  void watchdog()
  {
    double age = (now() - last_msg_time_).seconds();
    if (!row_lost_ && age > timeout_s_) {
      handle_row_lost();
    }
  }

  void handle_row_lost()
  {
    if (!row_lost_) {
      row_lost_ = true;
      RCLCPP_WARN(get_logger(),
        "Row lost — freezing toolbar at %.3fm, publishing pause signal", last_cmd_);
      publish_row_lost(true);
    }
    // keep publishing last valid cmd so actuator stays put
    std_msgs::msg::Float32 out;
    out.data = last_cmd_;
    pub_cmd_->publish(out);
  }

  void publish_row_lost(bool lost)
  {
    std_msgs::msg::Bool msg;
    msg.data = lost;
    pub_row_lost_->publish(msg);
  }

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_cmd_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_row_lost_;
  rclcpp::TimerBase::SharedPtr watchdog_;

  float last_cmd_;
  float cal_offset_;
  float limit_;
  double timeout_s_;
  bool row_lost_;
  rclcpp::Time last_msg_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ToolSliderController>());
  rclcpp::shutdown();
  return 0;
}
