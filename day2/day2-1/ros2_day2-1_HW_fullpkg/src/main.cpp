#include <QApplication>
#include "main_window.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);

    MainWindow window;
    window.show();      //gui 화면에 나타남

    int result = app.exec();
    rclcpp::shutdown();
    return result;
}
