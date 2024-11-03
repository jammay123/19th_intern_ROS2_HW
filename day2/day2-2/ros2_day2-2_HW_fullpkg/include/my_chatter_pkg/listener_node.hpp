#ifndef LISTENER_NODE_HPP
#define LISTENER_NODE_HPP

#include <QObject>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class ListenerNode : public QObject {
    Q_OBJECT

public:
    ListenerNode(QObject* parent = nullptr);
    ~ListenerNode();

signals:
    void messageReceived(const QString& message);

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
    std::thread spin_thread_;

    void onMessageReceived(const std_msgs::msg::String::SharedPtr msg);
};

#endif // LISTENER_NODE_HPP
