#ifndef NAV_GOAL_SUBSCRIBER_NODE_HPP
#define NAV_GOAL_SUBSCRIBER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include <std_msgs/msg/bool.hpp>

class NavGoalSubscriberNode : public rclcpp::Node {
public:
  NavGoalSubscriberNode();

private:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  // Publish initial pose to AMCL
  void publish_initial_pose();

  // Navigation goal handling
  void goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void send_navigation_goal(
      const geometry_msgs::msg::PoseStamped::SharedPtr goal_pose);

  // Action callbacks
  void goal_response_callback(const GoalHandleNavigate::SharedPtr &goal_handle);
  void result_callback(const GoalHandleNavigate::WrappedResult &result);

  // Manual corridor mode callback
  void manual_mode_callback(const std_msgs::msg::Bool::SharedPtr msg);

  // Member variables
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      goal_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      current_pose_sub;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_;

  // Subscriber to check manual corridor mode status
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr manual_mode_sub_;

  rclcpp::TimerBase::SharedPtr retry_goal_timer_;
  int retry_count_;

  bool is_navigating_;
  bool nav_stack_active_;

  // Manual corridor mode state
  bool actual_manual_nav_active;
  // Track if at least one goal was received
  bool has_received_goal_;
  // Store last received goal
  geometry_msgs::msg::PoseStamped saved_goal_;
};

#endif // NAV_GOAL_SUBSCRIBER_NODE_HPP