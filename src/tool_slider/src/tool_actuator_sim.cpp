// tool_actuator_sim — simulates the toolbar linear actuator.
//
// Models a first-order low-pass lag between commanded and actual position,
// clamps to mechanical limits. Drives the Gazebo prismatic joint via
// /tool_bar_joint/cmd_pos (std_msgs/Float64) which is bridged ROS→GZ.
//
// Subscribes:
//   /tool_bar_position_cmd  (std_msgs/Float32, meters, +right)
//
// Publishes:
//   /tool_bar_position         (std_msgs/Float32, meters — actual position after lag)
//   /tool_bar_joint/cmd_pos    (std_msgs/Float64, meters — Gazebo joint command)
//
// Parameters:
//   time_constant_s   first-order lag τ (default: 0.3)
//   slider_limit_m    mechanical half-travel (default: 0.10)
//   update_rate_hz    (default: 50.0)

#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"

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

    pub_pos_  = create_publisher<std_msgs::msg::Float32>("tool_bar_position", 10);
    pub_joint_ = create_publisher<std_msgs::msg::Float64>("/tool_bar_joint/cmd_pos", 10);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(dt_),
      std::bind(&ToolActuatorSim::tick, this));

    RCLCPP_INFO(get_logger(),
      "tool_actuator_sim: τ=%.2fs limit=±%.3fm rate=%.0fHz  joint: tool_bar_joint",
      tau_, limit_, rate_hz);
  }

private:
  void tick()
  {
    // first-order lag: actual += (cmd - actual) * dt / tau
    float alpha = static_cast<float>(dt_ / tau_);
    actual_ += alpha * (cmd_ - actual_);
    actual_ = std::max(-limit_, std::min(limit_, actual_));

    // publish actual position for feedback
    std_msgs::msg::Float32 pos;
    pos.data = actual_;
    pub_pos_->publish(pos);

    // drive Gazebo prismatic joint
    std_msgs::msg::Float64 joint_cmd;
    joint_cmd.data = static_cast<double>(actual_);
    pub_joint_->publish(joint_cmd);
  }

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_pos_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_joint_;
  rclcpp::TimerBase::SharedPtr timer_;

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
