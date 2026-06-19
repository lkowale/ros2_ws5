#ifndef RS_TEST_NAV__RS_TEST_NAVIGATOR_HPP_
#define RS_TEST_NAV__RS_TEST_NAVIGATOR_HPP_

#include <string>
#include <memory>
#include <vector>

#include "nav2_core/behavior_tree_navigator.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "solbot5_msgs/action/run_rs_test.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace rs_test_nav
{

class RsTestNavigator
  : public nav2_core::BehaviorTreeNavigator<solbot5_msgs::action::RunRsTest>
{
public:
  using ActionT = solbot5_msgs::action::RunRsTest;

  RsTestNavigator() = default;
  ~RsTestNavigator() override = default;

  bool configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
    std::shared_ptr<nav2_util::OdomSmoother> odom_smoother) override;

  std::string getName() override {return "run_rs_test";}

  std::string getDefaultBTFilepath(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node) override;

protected:
  bool goalReceived(ActionT::Goal::ConstSharedPtr goal) override;
  void onLoop() override;
  void onPreempt(ActionT::Goal::ConstSharedPtr goal) override;
  void goalCompleted(
    typename ActionT::Result::SharedPtr result,
    const nav2_behavior_tree::BtStatus final_bt_status) override;

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  geometry_msgs::msg::PoseStamped latest_robot_pose_;

  std::vector<geometry_msgs::msg::PoseStamped> goals_;
  std::vector<std::string> labels_;
  size_t current_idx_{0};
};

}  // namespace rs_test_nav

#endif  // RS_TEST_NAV__RS_TEST_NAVIGATOR_HPP_
