// tool_slider_controller — vision-guided toolbar position controller.
//
// Subscribes to:
//   /plant_row_offset  (std_msgs/Float32, meters) — row-midpoint minus
//                       implement-center position (already an error signal,
//                       +right); NaN = row lost
//
// Publishes:
//   /tool_bar_position_cmd  (std_msgs/Float32, meters from center, +right)
//
// PID on the vision error drives an incremental adjustment to the commanded
// toolbar position each tick (cmd += Kp*e + Ki*integral + Kd*de/dt, clamped
// to the mechanical limit). A bare feedforward (cmd = offset) was tried first
// and self-oscillated: the vision offset already includes the toolbar's own
// position, so treating it as an absolute position command with no damping
// snapped the toolbar to the ±limit every tick the sign flipped, and the
// actuator's 0.3s lag turned that into a sustained limit-cycle (confirmed via
// tool_slider_logger: implement pixel position itself oscillating even though
// its detection was verified stable/unambiguous — the plant was oscillating,
// not the sensor). PID with derivative damping and a lower proportional gain
// fixes this the standard way.
//
// When row is lost (NaN received or no message for timeout_s), the pause action fires:
//   - freezes last valid command, resets the integral term
//   - publishes /tool_slider/row_lost (std_msgs/Bool, true)
//
// Parameters:
//   cal_offset_m      camera-to-toolbar lateral calibration offset (default: 0.0)
//   slider_limit_m    mechanical half-travel limit (default: 0.10)
//   row_lost_timeout_s  seconds without valid detection before pause (default: 0.5)
//   kp, ki, kd        PID gains (defaults: 0.5, 0.05, 0.05)
//   integral_limit_m  anti-windup clamp on the integral term (default: 0.05)

#include <algorithm>
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
    cmd_(0.0f),
    integral_(0.0f),
    prev_error_(0.0f),
    have_prev_error_(false),
    row_lost_(false)
  {
    declare_parameter("cal_offset_m", 0.0);
    declare_parameter("slider_limit_m", 0.10);
    declare_parameter("row_lost_timeout_s", 0.5);
    declare_parameter("kp", 0.5);
    declare_parameter("ki", 0.05);
    declare_parameter("kd", 0.05);
    declare_parameter("integral_limit_m", 0.05);

    cal_offset_    = static_cast<float>(get_parameter("cal_offset_m").as_double());
    limit_         = static_cast<float>(get_parameter("slider_limit_m").as_double());
    timeout_s_     = get_parameter("row_lost_timeout_s").as_double();
    kp_            = static_cast<float>(get_parameter("kp").as_double());
    ki_            = static_cast<float>(get_parameter("ki").as_double());
    kd_            = static_cast<float>(get_parameter("kd").as_double());
    integral_limit_ = static_cast<float>(get_parameter("integral_limit_m").as_double());

    sub_ = create_subscription<std_msgs::msg::Float32>(
      "plant_row_offset", 10,
      std::bind(&ToolSliderController::cb_offset, this, std::placeholders::_1));

    pub_cmd_      = create_publisher<std_msgs::msg::Float32>("tool_bar_position_cmd", 10);
    pub_row_lost_ = create_publisher<std_msgs::msg::Bool>("tool_slider/row_lost", 10);

    // watchdog at 10 Hz
    watchdog_ = create_wall_timer(100ms,
      std::bind(&ToolSliderController::watchdog, this));

    last_msg_time_ = now();

    // Live gain tuning: `ros2 param set /tool_slider_controller kp <val>` (also
    // ki, kd, integral_limit_m) takes effect on the next control tick, no
    // restart needed.
    param_cb_handle_ = add_on_set_parameters_callback(
      std::bind(&ToolSliderController::on_set_parameters, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "tool_slider_controller: PID(kp=%.3f ki=%.3f kd=%.3f) cal=%.3fm limit=±%.3fm timeout=%.2fs "
      "— gains tunable live via ros2 param set",
      kp_, ki_, kd_, cal_offset_, limit_, timeout_s_);
  }

private:
  rcl_interfaces::msg::SetParametersResult on_set_parameters(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & p : params) {
      if ((p.get_name() == "kp" || p.get_name() == "ki" || p.get_name() == "kd" ||
           p.get_name() == "integral_limit_m") && p.as_double() < 0.0)
      {
        result.successful = false;
        result.reason = p.get_name() + " must be >= 0";
        return result;
      }
    }

    for (const auto & p : params) {
      if (p.get_name() == "kp") kp_ = static_cast<float>(p.as_double());
      else if (p.get_name() == "ki") ki_ = static_cast<float>(p.as_double());
      else if (p.get_name() == "kd") kd_ = static_cast<float>(p.as_double());
      else if (p.get_name() == "integral_limit_m") integral_limit_ = static_cast<float>(p.as_double());
      else if (p.get_name() == "cal_offset_m") cal_offset_ = static_cast<float>(p.as_double());
      else if (p.get_name() == "slider_limit_m") limit_ = static_cast<float>(p.as_double());
      else continue;
      RCLCPP_INFO(get_logger(), "Parameter updated: %s = %.4f", p.get_name().c_str(), p.as_double());
    }
    return result;
  }

  void cb_offset(const std_msgs::msg::Float32::SharedPtr msg)
  {
    rclcpp::Time now_t = now();
    double dt = (now_t - last_msg_time_).seconds();
    last_msg_time_ = now_t;

    if (std::isnan(msg->data)) {
      handle_row_lost();
      return;
    }

    float error = msg->data + cal_offset_;

    // dt sanity: first sample after startup/row-loss has no meaningful dt.
    if (dt <= 0.0 || dt > 1.0 || !have_prev_error_) {
      dt = 0.0;
    }

    integral_ += error * static_cast<float>(dt);
    integral_ = std::max(-integral_limit_, std::min(integral_limit_, integral_));

    float derivative = (dt > 0.0) ? (error - prev_error_) / static_cast<float>(dt) : 0.0f;
    prev_error_ = error;
    have_prev_error_ = true;

    float delta = kp_ * error + ki_ * integral_ + kd_ * derivative;
    cmd_ = std::max(-limit_, std::min(limit_, cmd_ + delta));

    if (row_lost_) {
      row_lost_ = false;
      RCLCPP_INFO(get_logger(), "Row re-acquired — resuming slider control");
      publish_row_lost(false);
    }

    std_msgs::msg::Float32 out;
    out.data = cmd_;
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
      integral_ = 0.0f;
      have_prev_error_ = false;
      RCLCPP_WARN(get_logger(),
        "Row lost — freezing toolbar at %.3fm, publishing pause signal", cmd_);
      publish_row_lost(true);
    }
    // keep publishing last commanded position so actuator stays put
    std_msgs::msg::Float32 out;
    out.data = cmd_;
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
  OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  float cmd_;
  float integral_;
  float prev_error_;
  bool have_prev_error_;
  float cal_offset_;
  float limit_;
  double timeout_s_;
  float kp_, ki_, kd_;
  float integral_limit_;
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
