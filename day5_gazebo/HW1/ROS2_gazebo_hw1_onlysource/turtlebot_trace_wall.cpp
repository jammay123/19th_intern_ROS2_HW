#include "turtlebot_trace_wall/turtlebot_trace_wall.hpp"

TurtlebotTraceWall::TurtlebotTraceWall() : Node("turtlebot_trace_wall_node")
{
  // 벽과의 거리 및 정면 거리 파라미터 선언
  this->declare_parameter<double>("desired_distance_from_wall", 0.5);
  this->declare_parameter<double>("desired_distance_front", 1.2);

  // 변수에 파라미터 값 저장
  this->get_parameter("desired_distance_from_wall", desired_distance_from_wall_);
  this->get_parameter("desired_distance_front", desired_distance_front_);

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", 10, std::bind(&TurtlebotTraceWall::scan_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Turtlebot Trace Wall node has been started.");
  RCLCPP_INFO(this->get_logger(), "Desired distance from wall: %.2f, Desired distance front: %.2f",
              desired_distance_from_wall_, desired_distance_front_);
}

// scan 토픽 콜백 함수
void TurtlebotTraceWall::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  int right_index = (msg->ranges.size() * 3) / 4;  // 오른쪽 (약 270도)
  int front_index = 0;                            // 정면 (약 0도)

  float right_distance = msg->ranges[right_index];
  float front_distance = msg->ranges[front_index];

  RCLCPP_INFO(this->get_logger(), "Right distance: %.2f, Front distance: %.2f", right_distance, front_distance);

  geometry_msgs::msg::Twist cmd_vel;

  // 정면의 거리가 너무 가까운 경우 제자리에서 왼쪽으로 회전
  if (front_distance < desired_distance_front_)
  {
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.07 * 2; // 왼쪽 회전
  }
  else
  {
    // 오른쪽 벽의 거리 값에 따라 TurtleBot 조정
    if (right_distance < desired_distance_from_wall_)  // 벽에 너무 가까운 경우
    {
      cmd_vel.linear.x = 0.1;
      cmd_vel.angular.z = 0.07; // 왼쪽으로 회전
    }
    else if (right_distance > desired_distance_from_wall_)  // 벽에서 너무 먼 경우
    {
      cmd_vel.linear.x = 0.1;
      cmd_vel.angular.z = -0.07; // 오른쪽으로 회전
    }
    else  // 벽과 적정 거리 유지
    {
      cmd_vel.linear.x = 0.1;
      cmd_vel.angular.z = 0.0; // 직진
    }
  }

  cmd_vel_pub_->publish(cmd_vel);
  RCLCPP_INFO(this->get_logger(), "Published cmd_vel: linear.x=%.2f, angular.z=%.2f", cmd_vel.linear.x, cmd_vel.angular.z);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtlebotTraceWall>());
  rclcpp::shutdown();
  return 0;
}
