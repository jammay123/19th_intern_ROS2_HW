#ifndef control_cam_params_MAIN_WINDOW_H
#define control_cam_params_MAIN_WINDOW_H

#include <QMainWindow>
#include "QIcon"
#include "QImage"
#include "qnode.hpp"
#include "processing_qnode.hpp"
#include "parameter_qnode.hpp"
#include "ui_mainwindow.h"

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow(QWidget* parent = nullptr);
  ~MainWindow();
  QNode* qnode;
  ParameterQNode* parameter_qnode;
  ProcessingQNode* processing_qnode;

  private slots:
    void updateImageLabel(const QImage &image);
    void updateImageOriginal(const QImage &image);
    void updateImageCannyLabel(const QImage &image);

    void on_hue_upper_slider_valueChanged(int value);

    void on_hue_lower_slider_valueChanged(int value);

    void on_satr_upper_slider_valueChanged(int value);

    void on_satr_lower_slider_valueChanged(int value);

    void on_value_upper_slider_valueChanged(int value);

    void on_value_lower_slider_valueChanged(int value);

    void on_image_width_slider_valueChanged(int value);

    void on_image_height_slider_valueChanged(int value);

    void on_erode_cnt_slider_valueChanged(int value);

    void on_dilate_cnt_slider_valueChanged(int value);


    void on_canny_max_slider_valueChanged(int value);

    void on_canny_min_slider_valueChanged(int value);

private:
  Ui::MainWindowDesign* ui;
  void closeEvent(QCloseEvent* event);
};

#endif  // control_cam_params_MAIN_WINDOW_H
