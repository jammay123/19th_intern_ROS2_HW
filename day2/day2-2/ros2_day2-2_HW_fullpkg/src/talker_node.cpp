#include "my_chatter_pkg/talker_node.hpp"

TalkerNode::TalkerNode(QObject* parent) : QObject(parent) {
    node_ = rclcpp::Node::make_shared("talker_node");
    publisher_ = node_->create_publisher<std_msgs::msg::String>("qt_topic", 10);
}

TalkerNode::~TalkerNode() {
}

void TalkerNode::publishMessage(const QString& message) {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = message.toStdString();
    publisher_->publish(std::move(msg));
    emit messagePublished(message);
}
