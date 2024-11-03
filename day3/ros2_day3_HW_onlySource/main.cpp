#include <QApplication>
#include <iostream>
#include "../include/camera_qt_ros2_pkg/main_window.hpp"
#include "../include/camera_qt_ros2_pkg/qnode.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  QApplication a(argc, argv);
  MainWindow w;
  w.show();
  return a.exec();
}
