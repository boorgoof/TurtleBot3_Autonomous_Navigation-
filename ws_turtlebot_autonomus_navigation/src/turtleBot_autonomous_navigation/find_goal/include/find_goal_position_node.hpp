#ifndef FIND_GOAL_POSITION_NODE_HPP_
#define FIND_GOAL_POSITION_NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "apriltag_msgs/msg/april_tag_detection_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class FindGoalPositionNode : public rclcpp::Node
{
public:
    FindGoalPositionNode();

private:
    // Callback that get apriltag's frames with rispect to odom 
    void goal_pos_callback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg);
    
    // Helper functions 
    bool apriltag_position_wrt_odom( const std::string & tag_frame, const rclcpp::Time & stamp, double & x, double & y);
    geometry_msgs::msg::PoseStamped compute_goal_pose(double x1, double y1, double x2, double y2);
    bool has_goal_changed(const geometry_msgs::msg::PoseStamped & new_goal);
    
    // Subscriber to /apriltag/detections
    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detections_sub_;

    // TF2
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Publisher for the goal position
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pubblisher_;

    // variables
    std::string tag_frame_prefix_;
    std::string odom_frame_ = "odom";
        
    // Variable to store previous goal
    geometry_msgs::msg::PoseStamped previous_goal_;
    bool has_previous_goal_;
    double position_threshold_;
    double orientation_threshold_;

};

#endif  // FIND_GOAL_POSITION_NODE_HPP_