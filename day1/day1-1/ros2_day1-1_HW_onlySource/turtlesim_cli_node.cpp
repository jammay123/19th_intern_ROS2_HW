#include "turtlesim_cli/turtlesim_cli_node.hpp"

TurtlesimCliNode::TurtlesimCliNode() : Node("turtlesim_cli_node") {
    control_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    set_pen_client_ = this->create_client<turtlesim::srv::SetPen>("/turtle1/set_pen");
    clear_client_ = this->create_client<std_srvs::srv::Empty>("/clear");
    spawn_client_ = this->create_client<turtlesim::srv::Spawn>("/spawn");
    kill_client_ = this->create_client<turtlesim::srv::Kill>("/kill");

    this->declare_parameter("background_r", 0);
    this->declare_parameter("background_g", 0);
    this->declare_parameter("background_b", 0);
}

void TurtlesimCliNode::start_cli() {
    while (rclcpp::ok()) {
        std::cout << "\n원하는 모드를 선택하세요:\n1: 조종모드\n2: 배경색 설정 모드\n3: 거북이 모양 설정 모드\n4: Pen 설정 모드\n5: 종료\n";
        int mode;
        std::cin >> mode;

        switch (mode) {
            case 1: control_mode();
            break;
            case 2: set_background_color();
            break;
            case 3: set_turtle_shape();
            break;
            case 4: set_pen();
            break;
            default:
            break;
        }
    }
}

void TurtlesimCliNode::control_mode() {
    std::cout << "w,a,s,d 를 눌러 조종화세요, q 입력시 종료\n";
    geometry_msgs::msg::Twist twist;
    char input;
    while ((std::cin >> input) && (input != 'q')) {
        if (input == 'w')
            twist.linear.x = 1.0;

        else if (input == 's')
            twist.linear.x = -1.0;

        else if (input == 'a')
            twist.angular.z = 1.0;

        else if (input == 'd')
            twist.angular.z = -1.0;

        control_publisher_->publish(twist);
        twist.linear.x = 0.0;
        twist.angular.z = 0.0;
    }
}

void TurtlesimCliNode::set_background_color() {
    std::string color_input;
    std::cout << "r,g,b 셋 중 하나를 입력하세요: ";
    std::cin >> color_input;

    int r = 0, g = 0, b = 0;
    if (color_input == "r")
        r = 255;

    else if (color_input == "g")
        g = 255;

    else if (color_input == "b")
        b = 255;

    auto parameter_client = std::make_shared<rclcpp::AsyncParametersClient>(this->shared_from_this(), "turtlesim");
    if (parameter_client->wait_for_service(std::chrono::seconds(5))) {
        parameter_client->set_parameters({
            rclcpp::Parameter("background_r", r),
            rclcpp::Parameter("background_g", g),
            rclcpp::Parameter("background_b", b)
        });

        auto request = std::make_shared<std_srvs::srv::Empty::Request>();
        auto result = clear_client_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(this->shared_from_this(), result) == rclcpp::FutureReturnCode::SUCCESS) {
            std::cout << "배경 색 변경\n";
        }
    }
}

void TurtlesimCliNode::set_turtle_shape() {
    std::cout << "원하는 거북이 모양의 번호를 선택하세요:\n1: ardent\n2: bouncy\n3: crystal\n4: dashing\n5: eloquent\n6: foxy\n7: galactic\n8: humble\n9: rolling\n";

    int choice;
    std::cin >> choice;

    std::string turtle_name;
    switch(choice) {
        case 1: turtle_name = "ardent"; break;
        case 2: turtle_name = "bouncy"; break;
        case 3: turtle_name = "crystal"; break;
        case 4: turtle_name = "dashing"; break;
        case 5: turtle_name = "eloquent"; break;
        case 6: turtle_name = "foxy"; break;
        case 7: turtle_name = "galactic"; break;
        case 8: turtle_name = "humble"; break;
        case 9: turtle_name = "rolling"; break;
        default:
            return;
    }

    auto kill_request = std::make_shared<turtlesim::srv::Kill::Request>();
    kill_request->name = "turtle1";

    if (kill_client_->wait_for_service(std::chrono::seconds(1))) {
        auto kill_result = kill_client_->async_send_request(kill_request);
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), kill_result) == rclcpp::FutureReturnCode::SUCCESS) {
            std::cout << "기존 거북이 제거\n";
        }
    }

    auto spawn_request = std::make_shared<turtlesim::srv::Spawn::Request>();
    spawn_request->x = 5.5;  // 고정된 x 좌표
    spawn_request->y = 5.5;  // 고정된 y 좌표
    spawn_request->theta = 0.0;
    spawn_request->name = "turtle1";

    if (spawn_client_->wait_for_service(std::chrono::seconds(1))) {
        auto spawn_result = spawn_client_->async_send_request(spawn_request);
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), spawn_result) == rclcpp::FutureReturnCode::SUCCESS) {
            std::cout << "새로운 거북이가 생성\n";
        }
    }
}

void TurtlesimCliNode::set_pen() {
    auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
    std::string color_input;
    int width;

    std::cout << "r,g,b 셋 중 하나 입력하세요: ";
    std::cin >> color_input;

    if (color_input == "r") {
        request->r = 255;
        request->g = 0;
        request->b = 0;
    }
    else if (color_input == "g") {
        request->r = 0;
        request->g = 255;
        request->b = 0;
    }
    else if (color_input == "b") {
        request->r = 0; request->g = 0; request->b = 255;
    }

    std::cout << "두꼐(0~10)을 입력하세요: ";
    std::cin >> width;

    request->width = static_cast<uint8_t>(width);
    request->off = 0;

    if (set_pen_client_->wait_for_service(std::chrono::seconds(1))) {
        auto result = set_pen_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), result) == rclcpp::FutureReturnCode::SUCCESS)
            std::cout << "Pen 설정\n";
    }
}
