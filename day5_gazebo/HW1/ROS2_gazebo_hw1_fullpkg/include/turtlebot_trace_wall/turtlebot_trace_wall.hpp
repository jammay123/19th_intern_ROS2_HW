#ifndef TURTLEBOT_TRACE_WALL_HPP_
#define TURTLEBOT_TRACE_WALL_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

class TurtlebotTraceWall : public rclcpp::Node
{
public:
  TurtlebotTraceWall();

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;     //퍼블리셔 선언
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;   //sub 선언

  double desired_distance_from_wall_;
  double desired_distance_front_;

  double linear_velocity_;
  double angular_velocity_;
};

#endif  // TURTLEBOT_TRACE_WALL_HPP_
