
#ifndef CORRIDOR_NAVIGATOR_HPP
#define CORRIDOR_NAVIGATOR_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include <std_msgs/msg/bool.hpp>

class CorridorNavigator : public rclcpp::Node
{
public:
    CorridorNavigator();

private:

    enum class State {
        IDLE,             
        NAV2_STATE,       
        MANUAL_CORRIDOR_STATE
    };

    // State
    State current_state_;
    bool in_corridor_;
    
    // Constants for navigation
    const double LINEAR_SPEED = 3;
    const double MAX_ANGULAR_SPEED = 2;
    
    // nav2 goal management
    int8_t nav2_status_;
    
    // sublscribers, client and publishers
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr corridor_state_sub_;
    rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr nav2_status_sub_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav2_client_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr manual_mode_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    
    // Callbacks
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void corridorStateCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void nav2StatusCallback(const action_msgs::msg::GoalStatusArray::SharedPtr msg);
    

    // Helper functions
    geometry_msgs::msg::Twist corridorExitController(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    int angleToScanRangesIndex(const sensor_msgs::msg::LaserScan & scan, float angle);
    void publishManualMode(bool active);
    void cancelNav2();
    bool nav2GoalIsActive() const;
    bool nav2GoalIsCanceled() const;
};

#endif // CORRIDOR_NAVIGATOR_HPP