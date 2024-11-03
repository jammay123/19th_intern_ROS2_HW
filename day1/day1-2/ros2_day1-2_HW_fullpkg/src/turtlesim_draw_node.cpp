/* turtlesim_draw_node.cpp */
#include "turtlesim_draw_node.hpp"
#include <iostream>
#include <string>
#include <cmath>

TurtlesimDraw::TurtlesimDraw() : Node("turtlesim_draw")
{
    // Publisher 및 Client 설정
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
    set_pen_client_ = this->create_client<turtlesim::srv::SetPen>("turtle1/set_pen");




    while(rclcpp::ok())
    {
      // CLI 입력 받기
      int option;
      std::cout << "모드 선택:\n1. 도형 그리기\n2. Pen 설정\n : ";
      std::cin >> option;

      if (option == 1)
      {
        int shape;
        std::cout << "원하는 도형 번호 입력\n1. 삼각형\n2. 원\n3. 사각형\n : ";
        std::cin >> shape;

        int size;
        std::cout << "Size 입력(1~10): ";
        std::cin >> size;

        if (shape == 1)
        {
            draw_triangle(size);
        }
        else if (shape == 2)
        {
            draw_circle(size);
        }
        else if (shape == 3)
        {
            draw_square(size);
        }
      }

      else if (option == 2)
      {
      int color_option;
        std::cout << "원하는 펜 색상 번호 입력\n1. 빨강\n2. 초록\n3. 파랑\n : ";
        std::cin >> color_option;

        int r = 0, g = 0, b = 0;
        if (color_option == 1)
        {
            r = 255;
        }
        else if (color_option == 2)
        {
            g = 255;
        }
        else if (color_option == 3)
        {
            b = 255;
        }

        int width;
        std::cout << "Pen 두께 입력: ";
        std::cin >> width;

        set_pen(r, g, b, width);
      }
    }

}

void TurtlesimDraw::set_pen(int r, int g, int b, int width)
{
    auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
    request->r = r;
    request->g = g;
    request->b = b;
    request->width = width;

    while (!set_pen_client_->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_INFO(this->get_logger(), "Waiting for set_pen service...");
    }
    set_pen_client_->async_send_request(request);
}

void TurtlesimDraw::draw_triangle(int size)
{
    auto msg = geometry_msgs::msg::Twist();
    for (int i = 0; i < 3; i++)
    {
        msg.linear.x = size;
        msg.angular.z = 0.0;
        cmd_vel_pub_->publish(msg);
        rclcpp::sleep_for(std::chrono::seconds(1));
        msg.linear.x = 0.0;
        msg.angular.z = 2.094; // 120도
        cmd_vel_pub_->publish(msg);
        rclcpp::sleep_for(std::chrono::seconds(1));
    }
}

void TurtlesimDraw::draw_circle(int size)
{
    auto msg = geometry_msgs::msg::Twist();
    for (int i = 0; i < 12; i++)
    {
        msg.linear.x = size / 2;
        msg.angular.z = 0.5236; // 30도
        cmd_vel_pub_->publish(msg);
        rclcpp::sleep_for(std::chrono::seconds(1));
    }
}

void TurtlesimDraw::draw_square(int size)
{
    auto msg = geometry_msgs::msg::Twist();
    for (int i = 0; i < 4; i++)
    {
        msg.linear.x = size;
        msg.angular.z = 0.0;
        cmd_vel_pub_->publish(msg);
        rclcpp::sleep_for(std::chrono::seconds(1));
        msg.linear.x = 0.0;
        msg.angular.z = 1.5708; // 90도
        cmd_vel_pub_->publish(msg);
        rclcpp::sleep_for(std::chrono::seconds(1));
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtlesimDraw>());
    rclcpp::shutdown();
    return 0;
}
