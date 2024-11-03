#ifndef control_turtle_pkg_QNODE_HPP_
#define control_turtle_pkg_QNODE_HPP_

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/empty.hpp>
#include "turtlesim/srv/set_pen.hpp"
#include <rclcpp/parameter_client.hpp>
#endif
#include <QThread>
#include <chrono>
#include <cmath>
#include <thread>

class QNode : public QThread
{
    Q_OBJECT
public:
    QNode();
    ~QNode();

    void sendTwist(double linear, double angular);
    void set_background_color(const std::string &color);
    void setPen(const std::string& color, int width);
    void drawCircle(double radius, double speed);
    void drawSquare(double side_length, double speed);
    void drawTriangle(double side_length, double speed);
    void run() override;

private:
    std::shared_ptr<rclcpp::Node> node;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_client_;
    rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr set_pen_client_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscription_;


Q_SIGNALS:
    void rosShutDown();
    void twistReceived(double linear, double angular); // 신호 추가

};

#endif /* control_turtle_pkg_QNODE_HPP_ */
