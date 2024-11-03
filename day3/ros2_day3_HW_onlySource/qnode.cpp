#include "../include/camera_qt_ros2_pkg/qnode.hpp"

QNode::QNode()
{
  node= rclcpp::Node::make_shared("qt_camera_viewer");
  subscription_ = node->create_subscription<sensor_msgs::msg::Image>("/camera1/camera/image_raw", 10,
        std::bind(&QNode::imageCallback, this, std::placeholders::_1));

  this->start();
}

QNode::~QNode()
{
    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }
    this->wait();
}

void QNode::run()
{
  rclcpp::WallRate loop_rate(20);
  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  Q_EMIT rosShutDown();
}

void QNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try
    {
        cv::Mat cv_image = cv_bridge::toCvCopy(msg, "bgr8")->image;

        Q_EMIT imageReceived(cv_image);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(node->get_logger(), "Error converting image: %s", e.what());
    }
}
