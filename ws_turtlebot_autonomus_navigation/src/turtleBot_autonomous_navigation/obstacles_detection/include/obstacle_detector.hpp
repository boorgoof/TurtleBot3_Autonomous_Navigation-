#ifndef OBSTACLE_DETECTOR_HPP
#define OBSTACLE_DETECTOR_HPP

#include <memory>
#include <vector>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "laser_geometry/laser_geometry.hpp"

#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

// Custom messages
#include "obstacles_detection/msg/circle_obstacles_array.hpp"
#include "obstacles_detection/msg/circle_obstacle.hpp"

struct Circle {
    double x, y, r;
};

struct FittedCircle {
    struct Circle circle;
    double rmse;
    rclcpp::Time last_seen;
};

class CircleDetector : public rclcpp::Node {
public:
    CircleDetector();

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    
    std::vector<FittedCircle> detectCircles(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
    bool fitCircle(const std::vector<Eigen::Vector2d>& points, FittedCircle& result);
    void updateTrackedCircles(const std::vector<FittedCircle>& new_detections, rclcpp::Time now);

    void publishPointCloud(rclcpp::Time now);
    void publishCircles(rclcpp::Time now);
    void publishMarkers(rclcpp::Time now);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Publisher<obstacles_detection::msg::CircleObstaclesArray>::SharedPtr circle_obstacles_pub_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<laser_geometry::LaserProjection> projector_;
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr accumulated_cloud_;
    
    std::vector<FittedCircle> circle_obstacles_found_;
    
    rclcpp::Time last_scan_time_;
    bool first_scan_received_;
};

#endif