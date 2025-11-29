#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav_goal_subscriber_node.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavGoalSubscriberNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
