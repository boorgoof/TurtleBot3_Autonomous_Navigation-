#include "../include/nav_goal_subscriber_node.hpp"

NavGoalSubscriberNode::NavGoalSubscriberNode()
    : Node("nav_goal_subscriber_node"), is_navigating_(false),
      nav_stack_active_(false), actual_manual_nav_active(false),
      has_received_goal_(false) {

  RCLCPP_INFO(this->get_logger(), "Navigation Goal Subscriber Node started");

  // Initialize Nav2 action client for sending goals
  RCLCPP_INFO(this->get_logger(), "Initializing navigation action client...");
  nav_action_client_ =
      rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

  // Wait for action server to be available
  RCLCPP_INFO(this->get_logger(),
              "Waiting for Nav2 action server to become active...");
  if (!nav_action_client_->wait_for_action_server(std::chrono::seconds(3000))) {
    RCLCPP_ERROR(this->get_logger(),
                 "Nav2 action server not available after 3000 seconds!");
    nav_stack_active_ = false;
  } else {
    RCLCPP_INFO(this->get_logger(), "Nav2 action server is ready!");
    nav_stack_active_ = true;

    // Publish initial pose for AMCL localization
    publish_initial_pose();
  }

  // Subscribe to goal pose from apriltag detection with transient_local QoS: it
  // allows to receive last published goal even if published before subscription
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  qos.transient_local();
  qos.reliable();
  goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", qos,
      std::bind(&NavGoalSubscriberNode::goal_pose_callback, this,
                std::placeholders::_1));

  // Subscribe to manual corridor mode status
  manual_mode_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/manual_corridor_active", 10,
      std::bind(&NavGoalSubscriberNode::manual_mode_callback, this,
                std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Navigation initialization complete");
}

void NavGoalSubscriberNode::publish_initial_pose() {

  RCLCPP_INFO(this->get_logger(),
              "Publishing initial pose to /initialpose for AMCL localization");

  // Create publisher for initial pose
  auto initial_pose_pub =
      this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/initialpose", 10);

  // Wait for publisher to connect
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  auto initial_pose = geometry_msgs::msg::PoseWithCovarianceStamped();
  initial_pose.header.frame_id = "map";
  initial_pose.header.stamp = this->now();

  // Initial position at origin
  initial_pose.pose.pose.position.x = 0.0;
  initial_pose.pose.pose.position.y = 0.0;
  initial_pose.pose.pose.position.z = 0.0;

  // Orientation pointing
  initial_pose.pose.pose.orientation.x = 0.0;
  initial_pose.pose.pose.orientation.y = 0.0;
  initial_pose.pose.pose.orientation.z = 0.0;
  initial_pose.pose.pose.orientation.w = 1.0;

  // Covariance matrix (uncertainty in pose estimate)
  initial_pose.pose.covariance[0] = 0.25;
  initial_pose.pose.covariance[7] = 0.25;
  initial_pose.pose.covariance[35] = 0.06; // about 15 degrees

  // Publish multiple times to ensure AMCL receives it
  RCLCPP_INFO(this->get_logger(),
              "Initial pose: x=0.0, y=0.0, yaw=0.0 (pointing right)");
  for (int i = 0; i < 3; i++) {
    initial_pose_pub->publish(initial_pose);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  RCLCPP_INFO(this->get_logger(), "Initial pose published successfully");
}

void NavGoalSubscriberNode::goal_pose_callback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(),
              "Received goal pose: x=%.3f, y=%.3f, z=%.3f (frame: %s)",
              msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
              msg->header.frame_id.c_str());

  // Save the goal regardless of current state
  saved_goal_ = *msg;
  has_received_goal_ = true;

  // If it is in manual mode, it just save the goal (don't navigate)
  if (actual_manual_nav_active) {
    RCLCPP_INFO(this->get_logger(),
                "Now manual mode is active. The robot will navigate to the new "
                "goal when manual mode ends.");
    return;
  }

  // If we are already navigating, we update the goal by canceling current and
  // sending new one
  if (is_navigating_) {
    RCLCPP_INFO(this->get_logger(),
                "The robot is already navigating. The robot update the goal by "
                "canceling current and sending new one.");
  }

  // Send the new goal
  send_navigation_goal(msg);
}

void NavGoalSubscriberNode::send_navigation_goal(
    const geometry_msgs::msg::PoseStamped::SharedPtr goal_pose) {
  if (!nav_stack_active_) {
    RCLCPP_ERROR(this->get_logger(), "Navigation stack is not active yet!");
    return;
  }

  // Check if action server is ready
  if (!nav_action_client_->action_server_is_ready()) {
    RCLCPP_WARN(this->get_logger(), "Nav2 action server not ready, waiting...");
    if (!nav_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(),
                   "Nav2 action server is still not available!");
      return;
    }
  }

  // Goal must be in map frame for AMCL-based navigation
  auto goal_in_map = geometry_msgs::msg::PoseStamped();
  goal_in_map.header.frame_id = "map";
  goal_in_map.header.stamp = this->now();

  // Copy position and orientation
  goal_in_map.pose = goal_pose->pose;

  RCLCPP_INFO(this->get_logger(),
              "Converting goal to map frame: x=%.3f, y=%.3f",
              goal_in_map.pose.position.x, goal_in_map.pose.position.y);

  // Create navigation goal message
  nav2_msgs::action::NavigateToPose::Goal goal_msg = NavigateToPose::Goal();
  goal_msg.pose = goal_in_map;

  RCLCPP_INFO(this->get_logger(),
              "Sending navigation goal to /navigate_to_pose action server");

  // Setup action callbacks
  rclcpp_action::Client<NavGoalSubscriberNode::NavigateToPose>::SendGoalOptions
      send_goal_options =
          rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      std::bind(&NavGoalSubscriberNode::goal_response_callback, this,
                std::placeholders::_1);
  send_goal_options.result_callback = std::bind(
      &NavGoalSubscriberNode::result_callback, this, std::placeholders::_1);

  // Send goal to bt_navigator
  is_navigating_ = true;
  nav_action_client_->async_send_goal(goal_msg, send_goal_options);
}

void NavGoalSubscriberNode::goal_response_callback(
    const GoalHandleNavigate::SharedPtr &goal_handle) {
  if (!goal_handle) {

    // Goal rejected. bt_navigator is still initializing
    RCLCPP_WARN(this->get_logger(),
                "bt_navigator still initializing. Goal rejected momentarily. "
                "Will automatically retry sending the goal after 3 seconds");
    is_navigating_ = false;

    // Retry to accept the goal after a delay (we check manual is not active)
    if (has_received_goal_ && !actual_manual_nav_active) {

      // Cancel existing retry timer (if any)
      if (retry_goal_timer_) {
        retry_goal_timer_->cancel();
      }

      // Create timer and retry sending goal
      retry_goal_timer_ =
          this->create_wall_timer(std::chrono::seconds(1), [this]() {
            if (!is_navigating_ && !actual_manual_nav_active &&
                has_received_goal_) {
              RCLCPP_INFO(this->get_logger(), "Retrying to send goal to Nav2");
              auto goal_ptr = std::make_shared<geometry_msgs::msg::PoseStamped>(
                  saved_goal_);
              send_navigation_goal(goal_ptr);

              // Cancel timer after execution
              if (retry_goal_timer_) {
                retry_goal_timer_->cancel();
              }
            }
          });
    }
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted successfully by nav2");
    // Cancel existing retry timer (if any)
    if (retry_goal_timer_) {
      retry_goal_timer_->cancel();
    }
  }
}

void NavGoalSubscriberNode::result_callback(
    const GoalHandleNavigate::WrappedResult &result) {
  is_navigating_ = false;

  switch (result.code) {
  case rclcpp_action::ResultCode::SUCCEEDED:
    RCLCPP_INFO(this->get_logger(), "Navigation succeeded.");
    break;
  case rclcpp_action::ResultCode::ABORTED:
    RCLCPP_ERROR(this->get_logger(), "Navigation aborted.");
    break;
  case rclcpp_action::ResultCode::CANCELED:
    RCLCPP_WARN(this->get_logger(), "Navigation was canceled.");
    break;
  default:
    RCLCPP_ERROR(this->get_logger(),
                 "Navigation failed with unknown result code");
    break;
  }
}

void NavGoalSubscriberNode::manual_mode_callback(
    const std_msgs::msg::Bool::SharedPtr msg) {
  bool previous_manual_nav_active = actual_manual_nav_active;
  actual_manual_nav_active = msg->data;

  // If manual mode has finished, resume navigation to saved goal
  if (previous_manual_nav_active && !actual_manual_nav_active) {
    if (!has_received_goal_) {
      RCLCPP_WARN(
          this->get_logger(),
          "Manual mode ended but no goal available to resume navigation.");
      return;
    }

    RCLCPP_INFO(this->get_logger(),
                "Manual mode ended. Resuming Nav2 navigation to saved goal");

    auto goal_ptr =
        std::make_shared<geometry_msgs::msg::PoseStamped>(saved_goal_);
    send_navigation_goal(goal_ptr);
    is_navigating_ = true;
  }
}
