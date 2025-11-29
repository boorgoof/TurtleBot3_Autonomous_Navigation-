#include "../include/find_goal_position_node.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

FindGoalPositionNode::FindGoalPositionNode() : Node("find_goal_position_node")
{
    RCLCPP_INFO(this->get_logger(), "find_goal_position_node started");

    // Initialize TF2 buffer and listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    tag_frame_prefix_ = "tag36h11:";

    // Publisher for the goal position with QoS: it stores the last message for subscribers that join later.
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
    qos.transient_local();
    qos.reliable();
    goal_pose_pubblisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", qos);

    detections_sub_ = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>( "/apriltag/detections", rclcpp::SensorDataQoS(),
        [this](const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
        {
            this->goal_pos_callback(msg);
        });
    
    has_previous_goal_ = false;
    position_threshold_ = 0.01;  
    orientation_threshold_ = 0.05;
}

void FindGoalPositionNode::goal_pos_callback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg){
    
    // Check that two apriltags are detected
    const auto num_detections = msg->detections.size();
    if (num_detections < 2) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Error apriltag detected: (%zu). Expect 2 apriltag", num_detections);
        return;
    }

    // Get frames of the two apriltags
    const std::string frame_apriltag1 = tag_frame_prefix_ + std::to_string(msg->detections[0].id);
    const std::string frame_apriltag2 = tag_frame_prefix_ + std::to_string(msg->detections[1].id);

    double x1, y1, x2, y2;
    // Get apriltag1 positions w.r.t odom
    if (!apriltag_position_wrt_odom(frame_apriltag1, msg->header.stamp, x1, y1)) {
        RCLCPP_WARN(this->get_logger(), "Failed to get pose of %s w.r.t odom", frame_apriltag1.c_str());
        return;
    }
    // Get apriltag2 positions w.r.t odom
    if (!apriltag_position_wrt_odom(frame_apriltag2, msg->header.stamp, x2, y2)) {
        RCLCPP_WARN(this->get_logger(), "Failed to get pose of %s w.r.t odom", frame_apriltag2.c_str());
        return;
    }

    // Compute goal pose
    geometry_msgs::msg::PoseStamped goal = compute_goal_pose(x1, y1, x2, y2);

    // Check if goal has changed significantly
    if (has_goal_changed(goal)) {
        goal_pose_pubblisher_->publish(goal);
        RCLCPP_INFO( this->get_logger(), "Tag1 w.r.t odom: (%.3f, %.3f). Tag2 w.r.t odom: (%.3f, %.3f)", x1, y1, x2, y2);
        RCLCPP_INFO(this->get_logger(), "Published the goal position w.r.t odom: (%.3f, %.3f)", goal.pose.position.x, goal.pose.position.y);
        
        // Update previous goal
        previous_goal_ = goal;
        has_previous_goal_ = true;
    }

}


bool FindGoalPositionNode::apriltag_position_wrt_odom(const std::string & tag_frame, const rclcpp::Time & stamp, double & x, double & y){
    
    geometry_msgs::msg::TransformStamped tf_apriltag;
    try {
        tf_apriltag = tf_buffer_->lookupTransform( odom_frame_ , tag_frame, stamp, tf2::durationFromSec(0.1));  
    } catch (const tf2::TransformException & ex) {
        return false;
    }

    x = tf_apriltag.transform.translation.x;
    y = tf_apriltag.transform.translation.y;
    return true;
}


geometry_msgs::msg::PoseStamped FindGoalPositionNode::compute_goal_pose(double x1, double y1, double x2, double y2) {
  
    const double mid_x = 0.5 * (x1 + x2);
    const double mid_y = 0.5 * (y1 + y2);

    // RCLCPP_INFO( this->get_logger(), "Goal position w.r.t odom: (%.3f, %.3f)", mid_x, mid_y);

    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = this->now();
    goal.header.frame_id = odom_frame_ ;
    goal.pose.position.x = mid_x;
    goal.pose.position.y = mid_y;
    goal.pose.position.z = 0.0;

    const double diff_x = x1 - x2;
    const double diff_y = y1 - y2;
    double z_orientation = std::atan2(diff_y, -diff_x);

    goal.pose.orientation.x = 0.0;
    goal.pose.orientation.y = 0.0;
    goal.pose.orientation.z = std::sin(z_orientation / 2.0);
    goal.pose.orientation.w = std::cos(z_orientation / 2.0);

    return goal;
}

bool FindGoalPositionNode::has_goal_changed(const geometry_msgs::msg::PoseStamped & new_goal) {
    
    // check if there is a previous goal
    if (!has_previous_goal_) {
        return true;
    }

    // calculate position and orientation differences
    const double dx = new_goal.pose.position.x - previous_goal_.pose.position.x;
    const double dy = new_goal.pose.position.y - previous_goal_.pose.position.y;
    const double position_diff = std::sqrt(dx * dx + dy * dy);
    const double orientation_diff = std::abs(new_goal.pose.orientation.z - previous_goal_.pose.orientation.z);

    // check if differences exceed thresholds
    return (position_diff > position_threshold_ || orientation_diff > orientation_threshold_);
}