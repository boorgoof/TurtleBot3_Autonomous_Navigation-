#include "corridor_detector_node.hpp"
#include <cmath>
#include <algorithm>
#include <limits>
#include <cstdlib>
#include <ctime>

CorridorDetectorNode::CorridorDetectorNode() 
    : Node("corridor_detector_node"),
      in_corridor_(false),
      consecutive_corridor_detections_(0),
      consecutive_open_detections_(0)
{
    std::srand(std::time(nullptr));

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&CorridorDetectorNode::scan_callback, this, std::placeholders::_1)
    );

    corridor_state_pub_ = this->create_publisher<std_msgs::msg::Bool>("/corridor_state", 10);

    RCLCPP_INFO(this->get_logger(), "Corridor Detector initialized");
}

void CorridorDetectorNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    bool corridor_detected = detect_corridor(msg);

    if (corridor_detected) {
        consecutive_corridor_detections_++;
        consecutive_open_detections_ = 0;
        
        if (consecutive_corridor_detections_ >= 2 && !in_corridor_) {
            in_corridor_ = true;
            RCLCPP_INFO(this->get_logger(), "Entering corridor");
        }
    } else {
        consecutive_open_detections_++;
        consecutive_corridor_detections_ = 0;
        
        if (consecutive_open_detections_ >= 5 && in_corridor_) {
            in_corridor_ = false;
            RCLCPP_INFO(this->get_logger(), "Exiting corridor");
        }
    }

    auto state_msg = std_msgs::msg::Bool();
    state_msg.data = in_corridor_;
    corridor_state_pub_->publish(state_msg);
}

std::vector<CorridorDetectorNode::Point> CorridorDetectorNode::laser_to_cartesian(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    std::vector<Point> points;
    float angle = scan->angle_min;
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        float r = scan->ranges[i];
        if (std::isfinite(r) && r >= scan->range_min && r <= scan->range_max) {
            Point p;
            p.x = r * std::cos(angle);
            p.y = r * std::sin(angle);
            points.push_back(p);
        }
        angle += scan->angle_increment;
    }
    return points;
}

bool CorridorDetectorNode::fit_line_ransac(const std::vector<Point>& points, 
                                           Line& line_out, 
                                           std::vector<Point>& inliers, 
                                           std::vector<Point>& outliers)
{
    if (points.size() < 2) return false;

    int best_inlier_count = 0;
    Line best_line = {0, 0, 0};

    for (int i = 0; i < RANSAC_ITERATIONS; ++i) {

        int idx1 = std::rand() % points.size();
        int idx2 = std::rand() % points.size();
        
        if (idx1 == idx2) continue;

        const Point& p1 = points[idx1];
        const Point& p2 = points[idx2];

        // Compute Line ax + by + c = 0
        double a = p1.y - p2.y;
        double b = p2.x - p1.x;
        double c = -a * p1.x - b * p1.y;

        // Normalize
        double norm = std::sqrt(a*a + b*b);
        if (norm < 1e-6) continue;

        a /= norm;
        b /= norm;
        c /= norm;

        int current_inliers = 0;
        for (const auto& p : points) {
            double dist = std::abs(a * p.x + b * p.y + c);
            if (dist < RANSAC_THRESHOLD) {
                current_inliers++;
            }
        }

        if (current_inliers > best_inlier_count) {
            best_inlier_count = current_inliers;
            best_line = {a, b, c};
        }
    }

    if (best_inlier_count < MIN_INLIERS) return false;

    line_out = best_line;
    inliers.clear();
    outliers.clear();

    // Separate points
    for (const auto& p : points) {
        double dist = std::abs(best_line.a * p.x + best_line.b * p.y + best_line.c);
        if (dist < RANSAC_THRESHOLD) {
            inliers.push_back(p);
        } else {
            outliers.push_back(p);
        }
    }

    return true;
}

bool CorridorDetectorNode::are_lines_parallel(const Line& l1, const Line& l2)
{
    // Dot product of normals (a,b)
    double dot = l1.a * l2.a + l1.b * l2.b;
    // If parallel, dot should be close to 1 or -1
    return std::abs(std::abs(dot) - 1.0) < ANGLE_TOLERANCE_RAD; 
}

double CorridorDetectorNode::calculate_corridor_width(const Line& l1, const Line& l2)
{
    double x1 = -l1.a * l1.c;
    double y1 = -l1.b * l1.c;
    
    // Distance from P1 to L2
    return std::abs(l2.a * x1 + l2.b * y1 + l2.c);
}

bool CorridorDetectorNode::is_robot_between_lines(const Line& l1, const Line& l2)
{
    double x1 = -l1.a * l1.c;
    double y1 = -l1.b * l1.c;
    
    double x2 = -l2.a * l2.c;
    double y2 = -l2.b * l2.c;
    
    return (x1 * x2 + y1 * y2) < 0;
}

bool CorridorDetectorNode::detect_corridor(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    auto points = laser_to_cartesian(scan);
    
    // Fit first line
    Line l1;
    std::vector<Point> l1_inliers, l1_outliers;
    if (!fit_line_ransac(points, l1, l1_inliers, l1_outliers)) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Failed to detect first line (points: %zu)", points.size());
        return false;
    }

    // Fit second line from outliers
    Line l2;
    std::vector<Point> l2_inliers, l2_outliers;
    if (!fit_line_ransac(l1_outliers, l2, l2_inliers, l2_outliers)) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Failed to detect second line (remaining points: %zu)", l1_outliers.size());
        return false;
    }

    // Check geometric properties
    if (!are_lines_parallel(l1, l2)) {
        double dot = std::abs(l1.a * l2.a + l1.b * l2.b);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Lines not parallel (dot: %.3f, tolerance: %.3f)", dot, 1.0 - ANGLE_TOLERANCE_RAD);
        return false;
    }

    // Check corridor width
    double width = calculate_corridor_width(l1, l2);
    if (width < CORRIDOR_MIN_WIDTH || width > CORRIDOR_MAX_WIDTH) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Invalid corridor width: %.3f (range: %.1f - %.1f)", width, CORRIDOR_MIN_WIDTH, CORRIDOR_MAX_WIDTH);
        return false;
    }

    // Check if robot is between lines
    if (!is_robot_between_lines(l1, l2)) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Robot not between lines");
        return false;
    }

    return true;
}