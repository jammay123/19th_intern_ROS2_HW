#ifndef MY_QT_ROS2_CAMERA_PKG_QNODE_HPP_
#define MY_QT_ROS2_CAMERA_PKG_QNODE_HPP_

#include <QThread>
#include <QImage>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class QNode : public QThread
{
  Q_OBJECT
public:
  QNode();
  ~QNode();

protected:
  void run() override;

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

  std::shared_ptr<rclcpp::Node> node;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

Q_SIGNALS:
  void rosShutDown();
  void imageReceived(const cv::Mat &image);
};

#endif// MY_QT_ROS2_CAMERA_PKG_QNODE_HPP_
