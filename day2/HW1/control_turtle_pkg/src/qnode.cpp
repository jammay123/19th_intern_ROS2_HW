#include "qnode.hpp"

//노드 생성
QNode::QNode() : node(rclcpp::Node::make_shared("control_turtle_node"))
{
    //퍼블리셔 생성 및 초기화
    twist_publisher_ = node->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    clear_client_ = node->create_client<std_srvs::srv::Empty>("clear");
    set_pen_client_ = node->create_client<turtlesim::srv::SetPen>("/turtle1/set_pen");

    twist_subscription_ = node->create_subscription<geometry_msgs::msg::Twist>("/turtle1/cmd_vel",10,[this](const geometry_msgs::msg::Twist::SharedPtr msg) {emit twistReceived(msg->linear.x, msg->angular.z);});
}

QNode::~QNode()
{
    if(this->isRunning()) {
        this->quit();
        this->wait();
    }
    rclcpp::shutdown();
}

void QNode::run()
{
    rclcpp::spin(node);
    emit rosShutDown();
}

void QNode::sendTwist(double linear, double angular)
{
    geometry_msgs::msg::Twist twist;
    twist.linear.x = linear;
    twist.angular.z = angular;
    twist_publisher_->publish(twist);
}

void QNode::set_background_color(const std::string &color) {
    int r = 0, g = 0, b = 0;

    if (color == "r") {
        r = 255;
    } else if (color == "g") {
        g = 255;
    } else if (color == "b") {
        b = 255;
    }

    auto parameter_client = std::make_shared<rclcpp::AsyncParametersClient>(node, "turtlesim");

    if (parameter_client->wait_for_service(std::chrono::seconds(5))) {
        parameter_client->set_parameters({
            rclcpp::Parameter("background_r", r),
            rclcpp::Parameter("background_g", g),
            rclcpp::Parameter("background_b", b)
        });

        auto request = std::make_shared<std_srvs::srv::Empty::Request>();
        auto result = clear_client_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
            std::cout << "배경 색 변경\n";
        }
    }
}

void QNode::setPen(const std::string& color, int width) {
    auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
    if (color == "r") {
        request->r = 255;
        request->g = 0;
        request->b = 0;
    } else if (color == "g") {
        request->r = 0;
        request->g = 255;
        request->b = 0;
    } else if (color == "b") {
        request->r = 0;
        request->g = 0;
        request->b = 255;
    }
    request->width = static_cast<uint8_t>(width);
    set_pen_client_->async_send_request(request);
}


void QNode::drawCircle(double r, double speed) {
    std::thread([this, r, speed]() {
        double circumference = 2 * 3.14 * r; // 원의 둘레
        double duration = circumference / speed; // 전체 이동 시간
        double travel_time = duration / 20; // 20단계로 나누어 이동

        for (int i = 0; i < 20; i++) { // 20단계 반복
            sendTwist(speed, speed / r); // 선속도와 각속도 설정
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(travel_time * 1000))); // 시간 대기
        }
        sendTwist(0, 0); // 멈추기
    }).detach(); // 스레드 분리
}

void QNode::drawSquare(double side_length, double speed) {
    std::thread([this, side_length, speed]() {
        for (int i = 0; i < 4; i++) {
            sendTwist(speed, 0); // 직진
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>((side_length / speed) * 1000))); // 이동 시간 대기
            sendTwist(0, M_PI / 2); // 90도 회전
            std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // 회전 시간 대기
        }
        sendTwist(0, 0); // 멈추기
    }).detach(); // 스레드 분리
}

void QNode::drawTriangle(double side_length, double speed) {
    std::thread([this, side_length, speed]() {
        for (int i = 0; i < 3; i++) {
            sendTwist(speed, 0); // 직진
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>((side_length / speed) * 1000))); // 이동 시간 대기
            sendTwist(0, (2 * 3.14) / 3); // 120도 회전
            std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // 회전 시간 대기
        }
        sendTwist(0, 0); // 멈추기
    }).detach(); // 스레드 분리
}
