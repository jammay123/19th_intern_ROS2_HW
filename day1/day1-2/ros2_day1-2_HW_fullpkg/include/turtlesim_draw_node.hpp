/* turtlesim_draw_node.hpp */
#ifndef TURTLESIM_DRAW_NODE_HPP
#define TURTLESIM_DRAW_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/srv/set_pen.hpp"

class TurtlesimDraw : public rclcpp::Node
{
public:
    TurtlesimDraw();

private:
    void set_pen(int r, int g, int b, int width);
    void draw_triangle(int size);
    void draw_circle(int radius);
    void draw_square(int size);

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr set_pen_client_;
};

#endif // TURTLESIM_DRAW_NODE_HPP
