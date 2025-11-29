#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "find_goal_position_node.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto goal_node = std::make_shared<FindGoalPositionNode>();
    rclcpp::spin(goal_node);
    rclcpp::shutdown();
    return 0;
}