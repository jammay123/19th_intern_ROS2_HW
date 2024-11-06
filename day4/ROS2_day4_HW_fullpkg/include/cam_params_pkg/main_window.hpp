#ifndef CAM_PARAMS_PKG_MAIN_WINDOW_H
#define CAM_PARAMS_PKG_MAIN_WINDOW_H

#include <QMainWindow>
#include <QThread>
#include "ui_mainwindow.h"
#include "parameter_img_node.hpp"
#include "process_img_node.hpp"

class ParameterImgNode;  // 전방 선언으로 parameter_img_node 사용

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget* parent = nullptr);
  ~MainWindow();

  private slots:

  void on_hue_low_slider_1_valueChanged(int value);

      void on_hue_upp_slider_1_valueChanged(int value);

  void on_satr_low_slider_1_valueChanged(int value);

      void on_satr_upp_slider_1_valueChanged(int value);

  void on_val_low_slider_1_valueChanged(int value);


  void on_val_upp_slider_1_valueChanged(int value);

  void on_hue_low_slider_2_valueChanged(int value);

  void on_hue_upp_slider_2_valueChanged(int value);

  void on_satr_low_slider_2_valueChanged(int value);

  void on_satr_upp_slider_2_valueChanged(int value);

  void on_val_low_slider_2_valueChanged(int value);

  void on_val_upp_slider_2_valueChanged(int value);

  void on_hue_low_slider_3_valueChanged(int value);

  void on_hue_upp_slider_3_valueChanged(int value);

  void on_satr_low_slider_3_valueChanged(int value);

  void on_satr_upp_slider_3_valueChanged(int value);

  void on_val_low_slider_3_valueChanged(int value);

  void on_val_upp_slider_3_valueChanged(int value);

  void on_canny_upp_slider_valueChanged(int value);

  void on_canny_low_slider_valueChanged(int value);

  void on_width_slider_valueChanged(int value);

  void on_height_slider_valueChanged(int value);

  void on_erode_slider_valueChanged(int value);

  void on_dilate_slider_valueChanged(int value);

  void on_roi_p1_x_slider_valueChanged(int value);

  void on_roi_p1_y_slider_valueChanged(int value);

  void on_roi_p2_x_slider_valueChanged(int value);

  void on_roi_p2_y_slider_valueChanged(int value);

  void on_roi_p3_x_slider_valueChanged(int value);

  void on_roi_p3_y_slider_valueChanged(int value);

  void on_roi_p4_x_slider_valueChanged(int value);

  void on_roi_p4_y_slider_valueChanged(int value);


  void on_hsv_chbox_1_stateChanged(int arg1);

  void on_hsv_chbox_2_stateChanged(int arg1);

  void on_hsv_chbox_3_stateChanged(int arg1);

  void on_canny_chbox_stateChanged(int arg1);

  private:
  Ui::MainWindowDesign* ui;
  ParameterImgNode* parameterImgNode;  // 파라미터 설정 노드 객체
  void closeEvent(QCloseEvent* event);
};

#endif  // CAM_PARAMS_PKG_MAIN_WINDOW_H
