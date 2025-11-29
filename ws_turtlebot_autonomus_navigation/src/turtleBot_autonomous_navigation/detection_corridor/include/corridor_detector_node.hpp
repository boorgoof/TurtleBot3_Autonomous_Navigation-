#ifndef CORRIDOR_DETECTOR_NODE_HPP
#define CORRIDOR_DETECTOR_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include <vector>

class CorridorDetectorNode : public rclcpp::Node
{
public:
    CorridorDetectorNode();

private:
    struct Point {
        double x;
        double y;
    };

    struct Line {
        double a;
        double b;
        double c;
    };

    // Callback for laser scan data
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    // Check if robot is in a corridor based on scan data using line fitting
    bool detect_corridor(const sensor_msgs::msg::LaserScan::SharedPtr scan);

    // Convert LaserScan to Cartesian points
    std::vector<Point> laser_to_cartesian(const sensor_msgs::msg::LaserScan::SharedPtr scan);

    // Fit a line using RANSAC
    bool fit_line_ransac(const std::vector<Point>& points, 
                        Line& line_out, 
                        std::vector<Point>& inliers, 
                        std::vector<Point>& outliers);

    // Check parallelism
    bool are_lines_parallel(const Line& l1, const Line& l2);

    // Get distance between parallel lines
    double calculate_corridor_width(const Line& l1, const Line& l2);

    // Check if origin (robot) is between lines
    bool is_robot_between_lines(const Line& l1, const Line& l2);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr corridor_state_pub_;

    // Current corridor state
    bool in_corridor_;
    
    // Hysteresis to prevent flickering
    int consecutive_corridor_detections_;
    int consecutive_open_detections_;
    
    const float CORRIDOR_MAX_WIDTH = 1.2;
    const float CORRIDOR_MIN_WIDTH = 0.5;
    
    // RANSAC Parameters
    const int RANSAC_ITERATIONS = 200;
    const double RANSAC_THRESHOLD = 0.1;
    const int MIN_INLIERS = 15;
    const double ANGLE_TOLERANCE_RAD = 0.3;
};

#endif  // CORRIDOR_DETECTOR_NODE_HPP
