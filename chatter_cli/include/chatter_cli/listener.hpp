#ifndef LISTENER_HPP
#define LISTENER_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int64.hpp>

class Listener : public rclcpp::Node {
public:
    Listener();
private:
    void string_callback(const std_msgs::msg::String::SharedPtr msg);
    void int_callback(const std_msgs::msg::Int64::SharedPtr msg);
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr string_subscription_;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr int_subscription_;
};

#endif // LISTENER_HPP
