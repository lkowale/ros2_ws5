#ifndef ONE_LINE_NAV__ONE_LINE_NAVIGATOR_HPP_
#define ONE_LINE_NAV__ONE_LINE_NAVIGATOR_HPP_

#include <string>
#include <memory>
#include <vector>

#include "nav2_core/behavior_tree_navigator.hpp"
#include "solbot5_msgs/action/run_one_line.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace one_line_nav
{

class OneLineNavigator
  : public nav2_core::BehaviorTreeNavigator<solbot5_msgs::action::RunOneLine>
{
public:
  using ActionT = solbot5_msgs::action::RunOneLine;

  OneLineNavigator() = default;
  ~OneLineNavigator() override = default;

  bool configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
    std::shared_ptr<nav2_util::OdomSmoother> odom_smoother) override;

  std::string getName() override {return std::string("run_one_line");}

  std::string getDefaultBTFilepath(
    rclcpp_lifecycle::LifecycleNode::WeakPtr node) override;

protected:
  bool goalReceived(ActionT::Goal::ConstSharedPtr goal) override;
  void onLoop() override;
  void onPreempt(ActionT::Goal::ConstSharedPtr goal) override;
  void goalCompleted(
    typename ActionT::Result::SharedPtr result,
    const nav2_behavior_tree::BtStatus final_bt_status) override;

  bool loadLineFromFile(
    const std::string & field_name,
    double & start_x, double & start_y, double & start_yaw,
    double & end_x, double & end_y, double & end_yaw);

  std::string fields_directory_;
};

}  // namespace one_line_nav

#endif  // ONE_LINE_NAV__ONE_LINE_NAVIGATOR_HPP_
