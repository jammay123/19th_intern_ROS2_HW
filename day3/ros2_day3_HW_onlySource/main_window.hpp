#ifndef MY_QT_ROS2_CAMERA_PKG_MAIN_WINDOW_HPP_
#define MY_QT_ROS2_CAMERA_PKG_MAIN_WINDOW_HPP_

#include <QMainWindow>
#include "QIcon"
#include <QLabel>
#include "qnode.hpp"
#include "ui_mainwindow.h"

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow(QWidget* parent = nullptr);
  ~MainWindow();
  QNode* qnode;

public slots:
  void displayImage(const cv::Mat &image);

private:
  Ui::MainWindowDesign* ui;
  void closeEvent(QCloseEvent* event);
};

#endif// MY_QT_ROS2_CAMERA_PKG_MAIN_WINDOW_HPP_
