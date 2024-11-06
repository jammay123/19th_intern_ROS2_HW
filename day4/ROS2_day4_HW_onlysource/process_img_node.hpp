#ifndef PROCESS_IMG_NODE_HPP
#define PROCESS_IMG_NODE_HPP

#include <QThread>
#include <QImage>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

struct HSVParams {
    int hue_low = 0;
    int hue_upp = 180;
    int satr_low = 0;
    int satr_upp = 255;
    int val_low = 0;
    int val_upp = 255;
    int binary = 0;
};

class ProcessImgNode : public QThread
{
    Q_OBJECT

public:
    ProcessImgNode(rclcpp::Node::SharedPtr node, QObject* parent = nullptr);
    ~ProcessImgNode();

signals:
    void originalImageProcessed(const QImage& image);
    void frame1Processed(const QImage& image);
    void frame2Processed(const QImage& image);
    void hsv1Processed(const QImage& image);
    void hsv2Processed(const QImage& image);
    void hsv3Processed(const QImage& image);

protected:
    void run() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;

    HSVParams hsv_params_[3];  // HSV 파라미터 저장 배열

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void setupParameterEventCallback();
    void declareParameters();
    void setParameter(const std::string& param_name, int value);
};

#endif // PROCESS_IMG_NODE_HPP
