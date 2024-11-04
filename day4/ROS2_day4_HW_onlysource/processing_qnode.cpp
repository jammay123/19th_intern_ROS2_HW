#include "../include/control_cam_params/processing_qnode.hpp"

ProcessingQNode::ProcessingQNode() : QThread()
{
  node = rclcpp::Node::make_shared("processing_qnode");

  image_topic = node->declare_parameter<std::string>("image_topic", "/camera1/camera/image_raw");
  image_topic = node->get_parameter("image_topic").as_string();

  image_subscription_ = node->create_subscription<sensor_msgs::msg::Image>(
    image_topic, 10, std::bind(&ProcessingQNode::image_callback, this, std::placeholders::_1));

  parameters_subscription_ = node->create_subscription<rcl_interfaces::msg::ParameterEvent>(
    "/parameter_events", 10, std::bind(&ProcessingQNode::parameters_callback, this, std::placeholders::_1));

  hue_upper_ = node->declare_parameter<int>("hue_upper", 180);
  hue_lower_ = node->declare_parameter<int>("hue_lower", 0);
  saturation_upper_ = node->declare_parameter<int>("saturation_upper", 255);
  saturation_lower_ = node->declare_parameter<int>("saturation_lower", 0);
  value_upper_ = node->declare_parameter<int>("value_upper", 255);
  value_lower_ = node->declare_parameter<int>("value_lower", 0);

  image_width_ = node->declare_parameter<int>("image_width", 640);
  image_height_ = node->declare_parameter<int>("image_height", 360);

  erode_cnt_ = node->declare_parameter<int>("erode_cnt", 0);
  dilate_cnt_ = node->declare_parameter<int>("dilate_cnt", 0);

  canny_min_ = node->declare_parameter<int>("canny_min", 100);
  canny_max_ = node->declare_parameter<int>("canny_max", 200);

  this->start();
}

ProcessingQNode::~ProcessingQNode()
{
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
}

void ProcessingQNode::run()
{
  rclcpp::spin(node);
}

void ProcessingQNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        // 원본 이미지 frame1
        cv::Mat frame1 = cv_bridge::toCvShare(msg, "bgr8")->image;

        QImage qimage1(frame1.data, frame1.cols, frame1.rows, frame1.step, QImage::Format_BGR888);
        QImage safe_image1 = qimage1.copy(); // 안전하게 복사
        Q_EMIT imageOriginal(safe_image1);
        RCLCPP_INFO(node->get_logger(), "Original image emitted");

        // processimage 함수 사용
        cv::Mat frame2, frame3;
        processImage(frame1, frame2, frame3);

        QImage qimage2(frame2.data, frame2.cols, frame2.rows, frame2.step, QImage::Format_BGR888);
        QImage safe_image2 = qimage2.copy();
        Q_EMIT imageProcessed(safe_image2);

        QImage qimage3(frame3.data, frame3.cols, frame3.rows, frame3.step, QImage::Format_Grayscale8);
        QImage safe_image3 = qimage3.copy();
        Q_EMIT imageCannyProcessed(safe_image3);

    }
    catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(node->get_logger(), "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void ProcessingQNode::processImage(const cv::Mat &input_image, cv::Mat &output_image, cv::Mat &canny_output) {
    cv::Mat resized_image;
    cv::resize(input_image, resized_image, cv::Size(image_width_, image_height_));

    // 검은색 영역 필터링 및 사각형 검출
    cv::Mat gray_image, binary_image;
    cv::cvtColor(resized_image, gray_image, cv::COLOR_BGR2GRAY);
    cv::inRange(gray_image, cv::Scalar(0, 0, 0), cv::Scalar(50, 50, 50), binary_image);

    // 윤곽선 검출
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary_image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // ROI 영역 설정
    cv::Mat mask = cv::Mat::zeros(resized_image.size(), CV_8UC1);
    bool roi_detected = false;
    std::vector<cv::Point> roi_points;

    for (const auto& contour : contours) {
        std::vector<cv::Point> approx;
        cv::approxPolyDP(contour, approx, cv::arcLength(contour, true) * 0.02, true);

        // 꼭짓점이 4개인 컨투어를 사각형으로 간주
        if (approx.size() == 4 && cv::contourArea(approx) > 1000) {
            roi_points = approx;
            roi_detected = true;
            break;
        }
    }

    if (roi_detected) {
        // 검출된 사각형이 있을 경우 ROI 설정
        std::vector<std::vector<cv::Point>> fill_points = {roi_points};
        cv::fillPoly(mask, fill_points, cv::Scalar(255));

        // ROI 경계선
        cv::polylines(resized_image, fill_points, true, cv::Scalar(0, 255, 0), 2);
    }

    else {
        // 검출 실패
        mask = cv::Mat::ones(resized_image.size(), CV_8UC1) * 255;
    }

    // HSV 필터
    cv::Mat hsv_frame, hsv_mask;
    cv::cvtColor(resized_image, hsv_frame, cv::COLOR_BGR2HSV);
    cv::inRange(hsv_frame,
                cv::Scalar(hue_lower_, saturation_lower_, value_lower_),
                cv::Scalar(hue_upper_, saturation_upper_, value_upper_),
                hsv_mask);

    cv::bitwise_and(resized_image, resized_image, output_image, mask & hsv_mask);

    for (int i = 0; i < erode_cnt_; i++) {
        cv::erode(output_image, output_image, cv::Mat());
    }
    for (int i = 0; i < dilate_cnt_; i++) {
        cv::dilate(output_image, output_image, cv::Mat());
    }

    cv::Canny(output_image, canny_output, canny_min_, canny_max_);
}


void ProcessingQNode::parameters_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{
    for (const auto &changed_parameter : event->changed_parameters) {
        if (changed_parameter.name == "hue_lower") {
            hue_lower_ = changed_parameter.value.integer_value;
        }
        else if (changed_parameter.name == "hue_upper") {
            hue_upper_ = changed_parameter.value.integer_value;
        }
        else if (changed_parameter.name == "saturation_lower") {
            saturation_lower_ = changed_parameter.value.integer_value;
        }
        else if (changed_parameter.name == "saturation_upper") {
            saturation_upper_ = changed_parameter.value.integer_value;
        }
        else if (changed_parameter.name == "value_lower") {
            value_lower_ = changed_parameter.value.integer_value;
        }
        else if (changed_parameter.name == "value_upper") {
            value_upper_ = changed_parameter.value.integer_value;
        }

        else if (changed_parameter.name == "image_width") {
            image_width_ = changed_parameter.value.integer_value;
        }
        else if (changed_parameter.name == "image_height") {
            image_height_ = changed_parameter.value.integer_value;
        }

        else if (changed_parameter.name == "erode_cnt"){
            erode_cnt_ = changed_parameter.value.integer_value;
        }
        else if (changed_parameter.name == "dilate_cnt"){
            dilate_cnt_ = changed_parameter.value.integer_value;
        }

        else if (changed_parameter.name == "canny_min") {
            canny_min_ = changed_parameter.value.integer_value;
        }
        else if (changed_parameter.name == "canny_max") {
            canny_max_ = changed_parameter.value.integer_value;
        }

        else if (changed_parameter.name == "roi_p1_x") {
            roi_p1_x_ = changed_parameter.value.integer_value;
        }
        else if (changed_parameter.name == "roi_p1_y") {
            roi_p1_y_ = changed_parameter.value.integer_value;
        }
        else if (changed_parameter.name == "roi_p2_x") {
            roi_p2_x_ = changed_parameter.value.integer_value;
        }
        else if (changed_parameter.name == "roi_p2_y") {
            roi_p2_y_ = changed_parameter.value.integer_value;
        }
        else if (changed_parameter.name == "roi_p3_x") {
            roi_p3_x_ = changed_parameter.value.integer_value;
        }
        else if (changed_parameter.name == "roi_p3_y") {
            roi_p3_y_ = changed_parameter.value.integer_value;
        }
        else if (changed_parameter.name == "roi_p4_x") {
            roi_p4_x_ = changed_parameter.value.integer_value;
        }
        else if (changed_parameter.name == "roi_p4_y") {
            roi_p4_y_ = changed_parameter.value.integer_value;
        }
        RCLCPP_INFO(node->get_logger(), "Parameter %s changed to %ld", changed_parameter.name.c_str(), changed_parameter.value.integer_value);
    }
}

