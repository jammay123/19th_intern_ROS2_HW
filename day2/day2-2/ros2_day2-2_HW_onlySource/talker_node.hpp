#ifndef TALKER_NODE_HPP
#define TALKER_NODE_HPP

#include <QObject>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class TalkerNode : public QObject {
    Q_OBJECT

public:
    TalkerNode(QObject* parent = nullptr);
    ~TalkerNode();

    void publishMessage(const QString& message);

signals:
    void messagePublished(const QString& message);

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

#endif // TALKER_NODE_HPP
