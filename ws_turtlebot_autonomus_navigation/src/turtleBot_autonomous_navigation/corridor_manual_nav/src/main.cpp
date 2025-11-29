#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "../include/corridor_navigator.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CorridorNavigator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}