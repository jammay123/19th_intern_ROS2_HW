#include <QApplication>
#include <iostream>

#include "../include/my_chatter_pkg/main_window.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  QApplication a(argc, argv);
  MainWindow w;
  w.show();

  int result = a.exec();
  rclcpp::shutdown();
  return result;
}
