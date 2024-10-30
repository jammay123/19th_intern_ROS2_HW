#include "rclcpp/rclcpp.hpp"
#include "turtlesim_cli/turtlesim_cli_node.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtlesimCliNode>();
    node->start_cli();
    rclcpp::shutdown();
    return 0;
}
