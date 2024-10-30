#include "chatter_cli/talker.hpp"
#include <iostream>
#include <limits>

Talker::Talker() : Node("talker") {
    publisher_ = this->create_publisher<std_msgs::msg::String>("/chatter_cli", 10);
    count_publisher_ = this->create_publisher<std_msgs::msg::Int64>("/chatter_count", 10);
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&Talker::publish_message, this));
}

void Talker::publish_message() {
    std_msgs::msg::String msg;
    std_msgs::msg::Int64 count_msg;

    std::cout << "메시지를 입력하세요: ";
    std::getline(std::cin, msg.data);

    std::cout << "몇 번째인지 입력하세요: ";
    std::cin >> count_msg.data;

    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    publisher_->publish(msg);
    count_publisher_->publish(count_msg);

    RCLCPP_INFO(this->get_logger(), "Published: '%s', Count: '%ld'", msg.data.c_str(), count_msg.data);
}
