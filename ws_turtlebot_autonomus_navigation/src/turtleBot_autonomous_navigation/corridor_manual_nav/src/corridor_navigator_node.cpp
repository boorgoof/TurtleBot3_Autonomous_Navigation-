#include "../include/corridor_navigator.hpp"

CorridorNavigator::CorridorNavigator() : Node("corridor_navigator_node"), current_state_(State::IDLE), in_corridor_(false), nav2_status_ (action_msgs::msg::GoalStatus::STATUS_UNKNOWN) {
    RCLCPP_INFO(this->get_logger(), "corridor_navigator_node started");
    
    
    // Subscriber for corridor state 
    corridor_state_sub_ = this->create_subscription<std_msgs::msg::Bool>("/corridor_state", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg)
        {
            this->corridorStateCallback(msg);
        });

    // Subscriber LaserScan 
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10,
        [this](const sensor_msgs::msg::LaserScan::SharedPtr msg)
        {
            this->scanCallback(msg);
        });

    
    // Subscriber Nav2 status
    nav2_status_sub_ = this->create_subscription<action_msgs::msg::GoalStatusArray>("/navigate_to_pose/_action/status", 10,
        [this](const action_msgs::msg::GoalStatusArray::SharedPtr msg)
        {
            this->nav2StatusCallback(msg);
        });
    
    // Publisher to cmd_vel for manual control
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    // Nav2 action client
    nav2_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>( this, "navigate_to_pose");

    // Publisher to indicate manual corridor mode active
    manual_mode_pub_ = this->create_publisher<std_msgs::msg::Bool>("/manual_corridor_active", 10);
    
}


void CorridorNavigator::corridorStateCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    bool was_in_corridor = in_corridor_;
    in_corridor_ = msg->data; 

    switch (current_state_) {

        case State::IDLE:

            // if the robot is in the corridor, switch to MANUAL_CORRIDOR_STATE mode
            if (in_corridor_) {
                RCLCPP_INFO(this->get_logger(), "The robot is in the corridor. Activating MANUAL mode");
                current_state_ = State::MANUAL_CORRIDOR_STATE;
                publishManualMode(true);
            }
            // if the nav2 action server is ready and the robot is not in the corridor, switch to NAV2_STATE
            else if (nav2GoalIsActive() && nav2_client_->action_server_is_ready()) {
                current_state_ = State::NAV2_STATE;
            }
            break;

        case State::NAV2_STATE:
            
            if (in_corridor_ && !was_in_corridor) {

                RCLCPP_INFO(this->get_logger(), "Entering corridor. Switching to MANUAL mode");
                
                // cancel nav2 goal
                cancelNav2();

                // switch state and publish manual mode active
                current_state_ = State::MANUAL_CORRIDOR_STATE;
                publishManualMode(true);
            }
            break;
            
        case State::MANUAL_CORRIDOR_STATE:
            
            if (!in_corridor_) {

                RCLCPP_INFO(this->get_logger(), "Exiting corridor.");

                publishManualMode(false);
                

                // if nav2 goal state is canceled and nav2_client is ready, resume nav2 to its saved goal
                if (nav2GoalIsCanceled() &&  nav2_client_->action_server_is_ready()) {
                    current_state_ = State::NAV2_STATE;
                    RCLCPP_INFO(this->get_logger(), "Nav2 resuming to its saved goal. Switching to NAV2_STATE");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Nav2 action server not ready or no active goal. Switching to IDLE");
                    current_state_ = State::IDLE;
                }
            }
            break;
        default:
            break;
    }
    
}

void CorridorNavigator::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
    
    if (current_state_ != State::MANUAL_CORRIDOR_STATE) {
        return;
    }
    // If in manual corridor mode, compute the custom navigation and publish cmd_vel: the robot moves forward centering itself between the walls
    auto cmd = corridorExitController(msg);
    cmd_vel_pub_->publish(cmd);
}

// custom controller to exit the corridor. Commands center the robot between the walls
geometry_msgs::msg::Twist CorridorNavigator::corridorExitController( const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    
    geometry_msgs::msg::Twist cmd;

    // 10 degrees of margin from the lateral directions. Note: pi/18 = 0.174533 radians
    float margin = 0.174533f; 

    // lateral directions in radians
    float left_center   = M_PI_2;          
    float right_center  = 3.0f * M_PI_2;

    // left ranges indexes
    int left_start = angleToScanRangesIndex(*scan, left_center - margin);
    int left_end = angleToScanRangesIndex(*scan, left_center + margin);

    // right ranges indexes
    int right_start = angleToScanRangesIndex(*scan, right_center - margin);
    int right_end = angleToScanRangesIndex(*scan, right_center + margin);


    float right_dist = 0.0f, left_dist = 0.0f;
    int right_count = 0, left_count = 0;

    // Average distance of the robot from the right wall and left wall
    for (int i = right_start; i < right_end; ++i) {
        if (std::isfinite(scan->ranges[i]) && scan->ranges[i] > 0.01f) {
            right_dist += scan->ranges[i];
            right_count++;
        }
    }
    right_dist = (right_count > 0) ? (right_dist / right_count) : 10.0f;

    for (int i = left_start; i < left_end; ++i) {
        if (std::isfinite(scan->ranges[i]) && scan->ranges[i] > 0.01f) {
            left_dist += scan->ranges[i];
            left_count++;
        }
    }
    left_dist  = (left_count  > 0) ? (left_dist  / left_count)  : 10.0f;

    // set the robot to a constant speed 
    cmd.linear.x = LINEAR_SPEED;

    

    // center the robot between the two walls
    const float Kp = 0.8f; 
    float error = right_dist - left_dist;   
    float angular_cmd = -Kp * error;

    // clamp angular speed
    if (angular_cmd >  MAX_ANGULAR_SPEED) angular_cmd =  MAX_ANGULAR_SPEED;
    if (angular_cmd < -MAX_ANGULAR_SPEED) angular_cmd = -MAX_ANGULAR_SPEED;

    cmd.angular.z = angular_cmd;

    RCLCPP_INFO(this->get_logger(), "Centering cmd: L=%.2f, R=%.2f | v_x=%.2f, v_z=%.2f",left_dist, right_dist, cmd.linear.x, cmd.angular.z);

    return cmd;
}

void CorridorNavigator::cancelNav2() {
    
    RCLCPP_WARN(this->get_logger(), "Nav2 goal cancellation requested");
    
    // if nav2 is active, send the cancellation request   
    if (nav2GoalIsActive() && nav2_client_->action_server_is_ready()) {
        nav2_client_->async_cancel_all_goals();
        RCLCPP_INFO(this->get_logger(), "Nav2 goal cancellation request sent");
    } else {
        RCLCPP_WARN(this->get_logger(), "It's not possible to cancel Nav2 goals");
    }
}

// Convert angle in radians to index in the LaserScan ranges array
int CorridorNavigator::angleToScanRangesIndex(const sensor_msgs::msg::LaserScan & scan, float angle){

    const int num_ranges = static_cast<int>(scan.ranges.size());

    // index calculation of the angle
    float idx_f = (angle - scan.angle_min) / scan.angle_increment;
    int idx = static_cast<int>(std::round(idx_f));

    // clamp index to valid range
    idx = std::clamp(idx, 0, num_ranges - 1);
    return idx;
}

void CorridorNavigator::publishManualMode(bool active){
    std_msgs::msg::Bool msg;
    msg.data = active;
    manual_mode_pub_->publish(msg);
}

void CorridorNavigator::nav2StatusCallback(const action_msgs::msg::GoalStatusArray::SharedPtr msg) {
    if (msg->status_list.empty()) {
        nav2_status_ = action_msgs::msg::GoalStatus::STATUS_UNKNOWN;
        return;
    }

    // Get the status of the last goal in the status list
    const action_msgs::msg::GoalStatus & last = msg->status_list.back();
    nav2_status_ = last.status;
}

// Check if Nav2 has an active goal
bool CorridorNavigator::nav2GoalIsActive() const {
    using S = action_msgs::msg::GoalStatus;
    if (S::STATUS_ACCEPTED || S::STATUS_EXECUTING ||S::STATUS_CANCELING) {
        return true;
    } else {
        return false;
    }

}

// Check if Nav2 goal is canceled
bool CorridorNavigator::nav2GoalIsCanceled() const {
    return nav2_status_ == action_msgs::msg::GoalStatus::STATUS_CANCELED;
}
