#include "obstacle_detector.hpp"

CircleDetector::CircleDetector() : Node("circle_detector_node")
{
    //PointCloud
    this->declare_parameter("keep_radius", 6.0);
    this->declare_parameter("point_cloud_ttl", 3.0);

    //clustering
    this->declare_parameter("cluster_tolerance", 0.5);
    this->declare_parameter("min_points_per_cluster", 5);
    
    //circle fitting
    this->declare_parameter("min_circle_radius", 0.02);
    this->declare_parameter("max_circle_radius", 0.25);
    this->declare_parameter("circle_fit_threshold", 0.04);

    //circle tracking
    this->declare_parameter("tracking_match_radius", 0.2);
    this->declare_parameter("circle_ttl", 1.0);
    this->declare_parameter("circle_improvement", 0.005);

    //circle visualization
    this->declare_parameter("point_color_r", 0);
    this->declare_parameter("point_color_g", 0);
    this->declare_parameter("point_color_b", 255);

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&CircleDetector::scanCallback, this, std::placeholders::_1));

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/detected_circles", 10);
    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/accumulated_cloud", 10);
    circle_obstacles_pub_ = this->create_publisher<obstacles_detection::msg::CircleObstaclesArray>("/circle_obstacles", 10);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    projector_ = std::make_shared<laser_geometry::LaserProjection>();

    accumulated_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    
    RCLCPP_INFO(this->get_logger(), "Accumulated Circle Detector Started.");
}

void CircleDetector::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    rclcpp::Time current_time = msg->header.stamp;
    double dt = 0.0;

    if (!first_scan_received_) {
        last_scan_time_ = current_time;
        first_scan_received_ = true;
    } else {
        dt = (current_time - last_scan_time_).seconds();
        last_scan_time_ = current_time;
    }

    if (dt < 0) {
        accumulated_cloud_->clear();
        dt = 0.0;
    }
    //age existing points
    if (!accumulated_cloud_->empty()) {
        for (auto & p : accumulated_cloud_->points) {
            p.intensity += static_cast<float>(dt);
        }
    }
    bool has_new_scan = false;
    double robot_x = 0.0, robot_y = 0.0;
    bool has_robot_pose = false;
    pcl::PointCloud<pcl::PointXYZI>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    try {
        geometry_msgs::msg::TransformStamped t;
        t = tf_buffer_->lookupTransform("map", msg->header.frame_id, rclcpp::Time(0));
        robot_x = t.transform.translation.x;
        robot_y = t.transform.translation.y;
        has_robot_pose = true;

        sensor_msgs::msg::PointCloud2 cloud_local, cloud_global;
        projector_->transformLaserScanToPointCloud(msg->header.frame_id, *msg, cloud_local, *tf_buffer_);
        tf_buffer_->transform(cloud_local, cloud_global, "map");
        pcl::fromROSMsg(cloud_global, *new_cloud);

        for (auto &p : new_cloud->points) {
            p.intensity = 0.0f; 
        }
        has_new_scan = true;

    } catch (tf2::TransformException &ex) {}

    if (has_new_scan) {
        *accumulated_cloud_ += *new_cloud;
    }

    double keep_r_sq = std::pow(this->get_parameter("keep_radius").as_double(), 2);
    double max_age = this->get_parameter("point_cloud_ttl").as_double();

    auto it = std::remove_if(accumulated_cloud_->points.begin(), accumulated_cloud_->points.end(),
        [&](const pcl::PointXYZI& p) {
            
            if (p.intensity > max_age) return true;

            if (has_robot_pose) {
                double dx = p.x - robot_x;
                double dy = p.y - robot_y;
                if ((dx*dx + dy*dy) > keep_r_sq) return true;
            }

            return false;
        });

    accumulated_cloud_->points.erase(it, accumulated_cloud_->points.end());
    accumulated_cloud_->width = accumulated_cloud_->points.size();
    accumulated_cloud_->height = 1;

    publishPointCloud(msg->header.stamp);
    
    std::vector<FittedCircle> current_fitted_circles = detectCircles(accumulated_cloud_);
    updateTrackedCircles(current_fitted_circles, msg->header.stamp);
}

std::vector<FittedCircle> CircleDetector::detectCircles(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
    std::vector<FittedCircle> results;
    if (cloud->empty()) return results;

    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(this->get_parameter("cluster_tolerance").as_double()); 
    ec.setMinClusterSize(this->get_parameter("min_points_per_cluster").as_int());
    ec.setMaxClusterSize(5000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    double min_r = this->get_parameter("min_circle_radius").as_double();
    double max_r = this->get_parameter("max_circle_radius").as_double();
    double fit_thresh = this->get_parameter("circle_fit_threshold").as_double();

    for (const auto& point_indices : cluster_indices) {
        std::vector<Eigen::Vector2d> points_2d;
        for (const auto& idx : point_indices.indices) {
            points_2d.push_back(Eigen::Vector2d(cloud->points[idx].x, cloud->points[idx].y));
        }

        FittedCircle fc;
        if (fitCircle(points_2d, fc)) {
            if (fc.circle.r >= min_r && fc.circle.r <= max_r && fc.rmse <= fit_thresh) {
                results.push_back(fc);
            }
        }
    }
    return results;
}

bool CircleDetector::fitCircle(const std::vector<Eigen::Vector2d>& points, FittedCircle& result)
{
    size_t n = points.size();
    if (n < 3) return false;
    Eigen::MatrixXd A(n, 3);
    Eigen::VectorXd b(n);
    for (size_t i = 0; i < n; ++i) {
        A(i, 0) = points[i].x(); A(i, 1) = points[i].y(); A(i, 2) = 1.0;
        b(i) = points[i].x()*points[i].x() + points[i].y()*points[i].y();
    }
    Eigen::Vector3d sol = A.colPivHouseholderQr().solve(b);
    result.circle.x = sol(0) / 2.0;
    result.circle.y = sol(1) / 2.0;
    double r_sq = sol(2) + result.circle.x*result.circle.x + result.circle.y*result.circle.y;
    if (r_sq < 0) return false;
    result.circle.r = std::sqrt(r_sq);

    double sum_err_sq = 0.0;
    for (const auto& p : points) {
        double dist = std::sqrt(std::pow(p.x() - result.circle.x, 2) + std::pow(p.y() - result.circle.y, 2));
        sum_err_sq += std::pow(dist - result.circle.r, 2);
    }
    result.rmse = std::sqrt(sum_err_sq / n);
    return true;
}

void CircleDetector::updateTrackedCircles(const std::vector<FittedCircle>& new_detections, rclcpp::Time now)
{
    bool is_modified = false;
    double match_dist_sq = std::pow(this->get_parameter("tracking_match_radius").as_double(), 2);
    //remove old circles
    double circle_ttl = this->get_parameter("circle_ttl").as_double();
    auto it = std::remove_if(circle_obstacles_found_.begin(), circle_obstacles_found_.end(),
        [&](const FittedCircle& c) {
            return (now - c.last_seen).seconds() > circle_ttl;
        });
    if (it != circle_obstacles_found_.end()) {
        circle_obstacles_found_.erase(it, circle_obstacles_found_.end());
        is_modified = true;
    }
    //compare new detected circles with current detected circles
    double circle_improvement_ = this->get_parameter("circle_improvement").as_double();
    for (const auto& det : new_detections) {
        
        int best_idx = -1;
        double min_dist_sq = std::numeric_limits<double>::max();

        for (size_t i = 0; i < circle_obstacles_found_.size(); ++i) {
            double dx = circle_obstacles_found_[i].circle.x - det.circle.x;
            double dy = circle_obstacles_found_[i].circle.y - det.circle.y;
            double d2 = dx*dx + dy*dy;
            
            if (d2 < min_dist_sq) {
                min_dist_sq = d2;
                best_idx = i;
            }
        }

        if (best_idx != -1 && min_dist_sq < match_dist_sq) {
            //if new circle is slightly worse than old circle, it still replaces the old one as the scan is newer
            if (det.rmse - circle_improvement_ < circle_obstacles_found_[best_idx].rmse) {
                circle_obstacles_found_[best_idx].circle = det.circle;
                circle_obstacles_found_[best_idx].rmse = det.rmse;
                is_modified = true;
            }
            
            circle_obstacles_found_[best_idx].last_seen = now;
            
        } else {
            FittedCircle new_tc = det;
            new_tc.last_seen = now;
            circle_obstacles_found_.push_back(new_tc);
            is_modified = true;
        }
    }

    if (is_modified) {
        publishCircles(now);
        publishMarkers(now);
    }
}

void CircleDetector::publishPointCloud(rclcpp::Time now)
{
    pcl::PointCloud<pcl::PointXYZRGB> display_cloud;
        
    pcl::copyPointCloud(*accumulated_cloud_, display_cloud);

    uint8_t r = this->get_parameter("point_color_r").as_int();
    uint8_t g = this->get_parameter("point_color_g").as_int();
    uint8_t b = this->get_parameter("point_color_b").as_int();

    for (auto &p : display_cloud.points) {
        p.r = r;
        p.g = g;
        p.b = b;
        p.a = 255;
    }

    sensor_msgs::msg::PointCloud2 debug_out;
    pcl::toROSMsg(display_cloud, debug_out);
    debug_out.header.frame_id = "map";
    debug_out.header.stamp = now;
    cloud_pub_->publish(debug_out);
}

void CircleDetector::publishCircles(rclcpp::Time now)
{
    if (!tf_buffer_->canTransform("odom", "map", rclcpp::Time(0))) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
            "Cannot transform map -> odom. Skipping circle publication.");
        return;
    }

    obstacles_detection::msg::CircleObstaclesArray msg;
    msg.header.stamp = now;
    msg.header.frame_id = "odom";

    for (const auto& kc : circle_obstacles_found_) {
        geometry_msgs::msg::PointStamped p_map;
        p_map.header.frame_id = "map";
        p_map.header.stamp = now;
        p_map.point.x = kc.circle.x;
        p_map.point.y = kc.circle.y;
        p_map.point.z = 0.0;

        try {
            geometry_msgs::msg::PointStamped p_odom;
            p_odom = tf_buffer_->transform(p_map, "odom");

            obstacles_detection::msg::CircleObstacle oc;
            oc.center.x = p_odom.point.x;
            oc.center.y = p_odom.point.y;
            oc.center.z = 0.0;
            oc.radius = kc.circle.r;
            oc.fit_rmse = kc.rmse;
            
            msg.circles.push_back(oc);
        } catch (tf2::TransformException &ex) {
            continue;
        }
    }
    circle_obstacles_pub_->publish(msg);
}

void CircleDetector::publishMarkers(rclcpp::Time now)
{
    visualization_msgs::msg::MarkerArray ma;
    visualization_msgs::msg::Marker del;
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    ma.markers.push_back(del);

    int id=0;
    for (const auto& kc : circle_obstacles_found_) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = now;
        m.ns = "tracked";
        m.id = id++;
        m.type = visualization_msgs::msg::Marker::CYLINDER;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = kc.circle.x;
        m.pose.position.y = kc.circle.y;
        m.pose.position.z = 0.2;
        m.scale.x = kc.circle.r * 2.0;
        m.scale.y = kc.circle.r * 2.0;
        m.scale.z = 0.4;
        m.color.r = 0.0; m.color.g = 0.5; m.color.b = 0.0; m.color.a = 1;
        ma.markers.push_back(m);
    }
    marker_pub_->publish(ma);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CircleDetector>());
    rclcpp::shutdown();
    return 0;
}