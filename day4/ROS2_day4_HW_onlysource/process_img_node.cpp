#include "../include/cam_params_pkg/process_img_node.hpp"

ProcessImgNode::ProcessImgNode(rclcpp::Node::SharedPtr node, QObject* parent)
  : QThread(parent), node_(node)
{
    // HSV 파라미터 초기화
    hsv_params_[0] = HSVParams();
    hsv_params_[1] = HSVParams();
    hsv_params_[2] = HSVParams();
    declareParameters();

    setupParameterEventCallback();

    image_subscription_ = node_->create_subscription<sensor_msgs::msg::Image>(
        "/camera1/camera/image_raw", 10,
        std::bind(&ProcessImgNode::imageCallback, this, std::placeholders::_1));
}

void ProcessImgNode::declareParameters()
{
    // HSV1 파라미터
    node_->declare_parameter("hsv1_hue_low", hsv_params_[0].hue_low);
    node_->declare_parameter("hsv1_hue_upp", hsv_params_[0].hue_upp);
    node_->declare_parameter("hsv1_satr_low", hsv_params_[0].satr_low);
    node_->declare_parameter("hsv1_satr_upp", hsv_params_[0].satr_upp);
    node_->declare_parameter("hsv1_val_low", hsv_params_[0].val_low);
    node_->declare_parameter("hsv1_val_upp", hsv_params_[0].val_upp);
    node_->declare_parameter("hsv1_binary", hsv_params_[0].binary);

    // HSV2 파라미터
    node_->declare_parameter("hsv2_hue_low", hsv_params_[1].hue_low);
    node_->declare_parameter("hsv2_hue_upp", hsv_params_[1].hue_upp);
    node_->declare_parameter("hsv2_satr_low", hsv_params_[1].satr_low);
    node_->declare_parameter("hsv2_satr_upp", hsv_params_[1].satr_upp);
    node_->declare_parameter("hsv2_val_low", hsv_params_[1].val_low);
    node_->declare_parameter("hsv2_val_upp", hsv_params_[1].val_upp);
    node_->declare_parameter("hsv2_binary", hsv_params_[1].binary);

    // HSV3 파라미터
    node_->declare_parameter("hsv3_hue_low", hsv_params_[2].hue_low);
    node_->declare_parameter("hsv3_hue_upp", hsv_params_[2].hue_upp);
    node_->declare_parameter("hsv3_satr_low", hsv_params_[2].satr_low);
    node_->declare_parameter("hsv3_satr_upp", hsv_params_[2].satr_upp);
    node_->declare_parameter("hsv3_val_low", hsv_params_[2].val_low);
    node_->declare_parameter("hsv3_val_upp", hsv_params_[2].val_upp);
    node_->declare_parameter("hsv3_binary", hsv_params_[2].binary);

    node_->declare_parameter("roi_p1_x", 160);
    node_->declare_parameter("roi_p1_y", 90);
    node_->declare_parameter("roi_p2_x", 160);
    node_->declare_parameter("roi_p2_y", 270);
    node_->declare_parameter("roi_p3_x", 480);
    node_->declare_parameter("roi_p3_y", 270);
    node_->declare_parameter("roi_p4_x", 480);
    node_->declare_parameter("roi_p4_y", 90);

    node_->declare_parameter("erode_size", 1);
    node_->declare_parameter("dilate_size", 1);

    node_->declare_parameter("width", 640);
    node_->declare_parameter("height", 360);
}

void ProcessImgNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        cv::Mat hsv_frame;
        cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);

        // 원본 이미지 처리
        QImage qimg(frame.data, frame.cols, frame.rows, frame.step, QImage::Format_RGB888);
        emit originalImageProcessed(qimg.rgbSwapped());

        // HSV 마스크 처리 (1, 2, 3) 및 ROI
        for(int i = 0; i < 4; i++) {
            cv::Mat result;
            QImage processed_img;

            if(i == 3) { // ROI 구현
                result = frame.clone();

                // 이미지 크기 조정
                cv::resize(result, result, cv::Size(640, 360));

                std::vector<cv::Point> roi_poly;
                roi_poly.push_back(cv::Point(
                    static_cast<int>(node_->get_parameter("roi_p1_x").as_int()),
                    static_cast<int>(node_->get_parameter("roi_p1_y").as_int())
                ));
                roi_poly.push_back(cv::Point(
                    static_cast<int>(node_->get_parameter("roi_p2_x").as_int()),
                    static_cast<int>(node_->get_parameter("roi_p2_y").as_int())
                ));
                roi_poly.push_back(cv::Point(
                    static_cast<int>(node_->get_parameter("roi_p3_x").as_int()),
                    static_cast<int>(node_->get_parameter("roi_p3_y").as_int())
                ));
                roi_poly.push_back(cv::Point(
                    static_cast<int>(node_->get_parameter("roi_p4_x").as_int()),
                    static_cast<int>(node_->get_parameter("roi_p4_y").as_int())
                ));

                // Polyline 그리기
                cv::polylines(result, roi_poly, true, cv::Scalar(0, 255, 0), 5);

                // ROI 영역 마스크 생성
                cv::Mat mask = cv::Mat::zeros(result.size(), CV_8UC1);
                cv::fillPoly(mask, std::vector<std::vector<cv::Point>>{roi_poly}, cv::Scalar(255));

                // 마스크 적용
                cv::bitwise_and(result, result, result, mask);

                // QImage로 변환
                processed_img = QImage(result.data, result.cols, result.rows,
                                       result.step, QImage::Format_RGB888).rgbSwapped().copy();
            }
            else { // i = 0, 1, 2 (기존 HSV 처리 코드)
                cv::Mat mask;

                int width = node_->get_parameter("width").as_int();
                int height = node_->get_parameter("height").as_int();

                // HSV 범위로 마스크 생성
                cv::inRange(hsv_frame,
                            cv::Scalar(hsv_params_[i].hue_low, hsv_params_[i].satr_low, hsv_params_[i].val_low),
                            cv::Scalar(hsv_params_[i].hue_upp, hsv_params_[i].satr_upp, hsv_params_[i].val_upp),
                            mask);

                // Erosion 및 Dilation 적용
                int erode_size = node_->get_parameter("erode_size").as_int();
                int dilate_size = node_->get_parameter("dilate_size").as_int();
                cv::Mat erode_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(erode_size, erode_size));
                cv::Mat dilate_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(dilate_size, dilate_size));

                cv::erode(mask, mask, erode_kernel);
                cv::dilate(mask, mask, dilate_kernel);

                if (hsv_params_[i].binary) {
                    // 이진화된 마스크를 원래 프레임 크기로 확장
                    cv::resize(mask, mask, cv::Size(width, height));
                    processed_img = QImage(mask.data, mask.cols, mask.rows, mask.step, QImage::Format_Grayscale8).copy();
                }
                else {
                    // 마스크를 원본 이미지에 적용
                    result = cv::Mat::zeros(frame.size(), frame.type());
                    cv::bitwise_and(frame, frame, result, mask);

                    // 이미지 크기 조정
                    cv::resize(result, result, cv::Size(width, height));

                    processed_img = QImage(result.data, result.cols, result.rows, result.step, QImage::Format_RGB888).rgbSwapped().copy();
                }
            }

            // 시그널 발생
            switch(i) {
                case 0:
                    emit hsv1Processed(processed_img);
                    break;
                case 1:
                    emit hsv2Processed(processed_img);
                    break;
                case 2:
                    emit hsv3Processed(processed_img);
                    break;
                case 3:
                    emit frame2Processed(processed_img);
                    break;
            }
        }
    }
    catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
    }
}


void ProcessImgNode::setupParameterEventCallback() {
    auto parameters_callback = [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) {
        for (const auto& changed_parameter : event->changed_parameters) {
            const std::string& name = changed_parameter.name;
            const rclcpp::Parameter param(changed_parameter.name, changed_parameter.value);

            // HSV1 파라미터
            if (name == "hsv1_hue_low") hsv_params_[0].hue_low = param.as_int();
            else if (name == "hsv1_hue_upp") hsv_params_[0].hue_upp = param.as_int();
            else if (name == "hsv1_satr_low") hsv_params_[0].satr_low = param.as_int();
            else if (name == "hsv1_satr_upp") hsv_params_[0].satr_upp = param.as_int();
            else if (name == "hsv1_val_low") hsv_params_[0].val_low = param.as_int();
            else if (name == "hsv1_val_upp") hsv_params_[0].val_upp = param.as_int();
            else if (name == "hsv1_binary") hsv_params_[0].binary = param.as_int() == 1;

            // HSV2 파라미터
            else if (name == "hsv2_hue_low") hsv_params_[1].hue_low = param.as_int();
            else if (name == "hsv2_hue_upp") hsv_params_[1].hue_upp = param.as_int();
            else if (name == "hsv2_satr_low") hsv_params_[1].satr_low = param.as_int();
            else if (name == "hsv2_satr_upp") hsv_params_[1].satr_upp = param.as_int();
            else if (name == "hsv2_val_low") hsv_params_[1].val_low = param.as_int();
            else if (name == "hsv2_val_upp") hsv_params_[1].val_upp = param.as_int();
            else if (name == "hsv2_binary") hsv_params_[1].binary = param.as_int() == 1;

            // HSV3 파라미터
            else if (name == "hsv3_hue_low") hsv_params_[2].hue_low = param.as_int();
            else if (name == "hsv3_hue_upp") hsv_params_[2].hue_upp = param.as_int();
            else if (name == "hsv3_satr_low") hsv_params_[2].satr_low = param.as_int();
            else if (name == "hsv3_satr_upp") hsv_params_[2].satr_upp = param.as_int();
            else if (name == "hsv3_val_low") hsv_params_[2].val_low = param.as_int();
            else if (name == "hsv3_val_upp") hsv_params_[2].val_upp = param.as_int();
            else if (name == "hsv3_binary") hsv_params_[2].binary = param.as_int() == 1;
        }
    };

    parameter_event_sub_ = node_->create_subscription<rcl_interfaces::msg::ParameterEvent>(
        "/parameter_events", 10, parameters_callback);
}

void ProcessImgNode::run() {
    rclcpp::spin(node_);
}

ProcessImgNode::~ProcessImgNode() {
    if (isRunning()) {
        quit();
        wait();
    }
}
