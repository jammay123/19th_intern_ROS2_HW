#include "chatter_cli/listener.hpp"

Listener::Listener() : Node("listener") {
    string_subscription_ = this->create_subscription<std_msgs::msg::String>
    ("/chatter_cli", 10, std::bind(&Listener::string_callback, this, std::placeholders::_1));

    int_subscription_ = this->create_subscription<std_msgs::msg::Int64>
    ("/chatter_count", 10, std::bind(&Listener::int_callback, this, std::placeholders::_1));
}

void Listener::string_callback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Subscribed: '%s'", msg->data.c_str());
}

void Listener::int_callback(const std_msgs::msg::Int64::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Count: '%ld'", msg->data);
}
