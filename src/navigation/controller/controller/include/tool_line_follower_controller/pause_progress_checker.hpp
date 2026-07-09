#ifndef TOOL_LINE_FOLLOWER_CONTROLLER__PAUSE_PROGRESS_CHECKER_HPP_
#define TOOL_LINE_FOLLOWER_CONTROLLER__PAUSE_PROGRESS_CHECKER_HPP_

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/progress_checker.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "solbot5_msgs/msg/pause.hpp"

namespace pause_progress_checker
{

// Progress checker that publishes a high-severity pause instead of throwing an exception.
// The nav2 controller will keep running (returning the last velocity) while paused,
// and resume automatically once the operator clears the pause.
class PauseProgressChecker : public nav2_core::ProgressChecker
{
public:
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name) override;

  bool check(geometry_msgs::msg::PoseStamped & current_pose) override;

  void reset() override;

private:
  static double poseDistance(
    const geometry_msgs::msg::Pose2D & a,
    const geometry_msgs::msg::Pose2D & b);

  void publishPause(bool paused);

  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("PauseProgressChecker")};
  rclcpp::Publisher<solbot5_msgs::msg::Pause>::SharedPtr pause_pub_;

  double radius_;
  rclcpp::Duration time_allowance_{0, 0};

  geometry_msgs::msg::Pose2D baseline_pose_;
  rclcpp::Time baseline_time_;
  bool baseline_pose_set_{false};
  bool paused_{false};

  std::string plugin_name_;
};

}  // namespace pause_progress_checker

#endif  // TOOL_LINE_FOLLOWER_CONTROLLER__PAUSE_PROGRESS_CHECKER_HPP_
