#ifndef CONTROL_CAM_PARAMS_PROCESSING_QNODE_HPP_
#define CONTROL_CAM_PARAMS_PROCESSING_QNODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rcl_interfaces/msg/parameter_event.hpp>
#include <std_msgs/msg/int64.hpp>
#include <QThread>
#include <QImage>
#include <opencv2/imgproc.hpp>

class ProcessingQNode : public QThread
{
  Q_OBJECT
public:
  ProcessingQNode();
  ~ProcessingQNode();

protected:
  void run() override;

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void parameters_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event);
  void processImage(const cv::Mat &input_image, cv::Mat &output_image, cv::Mat &canny_output);

  std::shared_ptr<rclcpp::Node> node;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  std::string image_topic;

  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameters_subscription_;


  int hue_upper_, hue_lower_, saturation_upper_, saturation_lower_, value_upper_, value_lower_;
  int image_width_, image_height_;
  int erode_cnt_, dilate_cnt_;
  int canny_min_, canny_max_;
  int roi_p1_x_, roi_p1_y_;
  int roi_p2_x_, roi_p2_y_;
  int roi_p3_x_, roi_p3_y_;
  int roi_p4_x_, roi_p4_y_;

Q_SIGNALS:
  void imageProcessed(const QImage &image);
  void imageOriginal(const QImage &image);
  void imageCannyProcessed(const QImage &image);
};

#endif // CONTROL_CAM_PARAMS_PROCESSING_QNODE_HPP_
