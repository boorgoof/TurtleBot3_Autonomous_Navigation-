#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "corridor_detector_node.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CorridorDetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
