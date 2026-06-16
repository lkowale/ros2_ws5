#ifndef DRIVE_ARC__DRIVE_ARC_BEHAVIOR_HPP_
#define DRIVE_ARC__DRIVE_ARC_BEHAVIOR_HPP_

#include <memory>
#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "solbot5_msgs/action/drive_arc.hpp"

namespace drive_arc
{

class DriveArcBehavior : public nav2_behaviors::TimedBehavior<solbot5_msgs::action::DriveArc>
{
  using ActionT = solbot5_msgs::action::DriveArc;

public:
  DriveArcBehavior();
  ~DriveArcBehavior() = default;

  nav2_behaviors::ResultStatus onRun(
    const std::shared_ptr<const ActionT::Goal> command) override;

  nav2_behaviors::ResultStatus onCycleUpdate() override;

  nav2_core::CostmapInfoType getResourceInfo() override
  {
    return nav2_core::CostmapInfoType::LOCAL;
  }

protected:
  void onConfigure() override;

private:
  geometry_msgs::msg::PoseStamped initial_pose_;
  double target_angle_;
  double linear_speed_;
  double angular_speed_;
  double radius_;
  rclcpp::Time end_time_;
  rclcpp::Duration time_allowance_{0, 0};
  typename ActionT::Feedback::SharedPtr feedback_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  void publishPredictedPath(double radius, double angle, double speed);
};

}  // namespace drive_arc

#endif  // DRIVE_ARC__DRIVE_ARC_BEHAVIOR_HPP_
