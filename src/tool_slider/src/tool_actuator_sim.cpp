// tool_actuator_sim — simulates the toolbar linear actuator.
//
// Models a first-order low-pass lag between commanded and actual position,
// clamps to mechanical limits, and broadcasts the TF: tool_link → tool_bar.
//
// Subscribes:
//   /tool_bar_position_cmd  (std_msgs/Float32, meters, +right)
//
// Publishes:
//   /tool_bar_position  (std_msgs/Float32, meters — actual position after lag)
//
// TF:
//   parent: tool_link
//   child:  tool_bar
//   translation: (0, actual_position, 0)  — lateral only
//
// Parameters:
//   time_constant_s   first-order lag τ (default: 0.3)
//   slider_limit_m    mechanical half-travel (default: 0.10)
//   update_rate_hz    (default: 50.0)

#include <chrono>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class ToolActuatorSim : public rclcpp::Node
{
public:
  ToolActuatorSim()
  : rclcpp::Node("tool_actuator_sim"),
    actual_(0.0f),
    cmd_(0.0f)
  {
    declare_parameter("time_constant_s", 0.3);
    declare_parameter("slider_limit_m", 0.10);
    declare_parameter("update_rate_hz", 50.0);

    tau_    = get_parameter("time_constant_s").as_double();
    limit_  = static_cast<float>(get_parameter("slider_limit_m").as_double());
    double rate_hz = get_parameter("update_rate_hz").as_double();
    dt_ = 1.0 / rate_hz;

    sub_ = create_subscription<std_msgs::msg::Float32>(
      "tool_bar_position_cmd", 10,
      [this](const std_msgs::msg::Float32::SharedPtr msg) { cmd_ = msg->data; });

    pub_ = create_publisher<std_msgs::msg::Float32>("tool_bar_position", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(dt_),
      std::bind(&ToolActuatorSim::tick, this));

    RCLCPP_INFO(get_logger(),
      "tool_actuator_sim: τ=%.2fs limit=±%.3fm rate=%.0fHz  TF: tool_link→tool_bar",
      tau_, limit_, rate_hz);
  }

private:
  void tick()
  {
    // first-order lag: actual += (cmd - actual) * dt / tau
    float alpha = static_cast<float>(dt_ / tau_);
    actual_ += alpha * (cmd_ - actual_);
    // clamp
    actual_ = std::max(-limit_, std::min(limit_, actual_));

    // publish actual position
    std_msgs::msg::Float32 pos;
    pos.data = actual_;
    pub_->publish(pos);

    // broadcast TF tool_link → tool_bar
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now();
    tf.header.frame_id = "tool_link";
    tf.child_frame_id  = "tool_bar";
    tf.transform.translation.x = 0.0;
    tf.transform.translation.y = actual_;   // lateral offset, +right
    tf.transform.translation.z = 0.0;
    tf.transform.rotation.x = 0.0;
    tf.transform.rotation.y = 0.0;
    tf.transform.rotation.z = 0.0;
    tf.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(tf);
  }

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  float actual_;
  float cmd_;
  float limit_;
  double tau_;
  double dt_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ToolActuatorSim>());
  rclcpp::shutdown();
  return 0;
}
