#include "../include/my_chatter_pkg/listener_node.hpp"
#include <thread>

ListenerNode::ListenerNode(QObject* parent) : QObject(parent) {
    node_ = rclcpp::Node::make_shared("listener_node");
    subscriber_ = node_->create_subscription<std_msgs::msg::String>("qt_topic", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            onMessageReceived(msg);
        });
        spin_thread_ = std::thread([this]() {
        rclcpp::spin(node_);
    });
}

ListenerNode::~ListenerNode() {

}

void ListenerNode::onMessageReceived(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(node_->get_logger(), "Received message: %s", msg->data.c_str());
    emit messageReceived(QString::fromStdString(msg->data));
}
