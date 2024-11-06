#include "../include/cam_params_pkg/main_window.hpp"

MainWindow::MainWindow(QWidget* parent)
  : QMainWindow(parent), ui(new Ui::MainWindowDesign)
{
  ui->setupUi(this);

  QIcon icon(":/images/icon.png");
  this->setWindowIcon(icon);

  // ParameterImgNode 초기화 및 연결 설정
  parameterImgNode = new ParameterImgNode(rclcpp::Node::make_shared("parameter_client_node"), this);
  parameterImgNode->start();

  ui->hue_low_slider_1->setValue(0);
  ui->hue_low_spinbox_1->setValue(0);
  connect(ui->hue_low_slider_1, &QSlider::valueChanged, ui->hue_low_spinbox_1, &QSpinBox::setValue);
  connect(ui->hue_low_spinbox_1, QOverload<int>::of(&QSpinBox::valueChanged), ui->hue_low_slider_1, &QSlider::setValue);

  ui->hue_upp_slider_1->setValue(180);
  ui->hue_upp_spinbox_1->setValue(180);
  connect(ui->hue_upp_slider_1, &QSlider::valueChanged, ui->hue_upp_spinbox_1, &QSpinBox::setValue);
  connect(ui->hue_upp_spinbox_1, QOverload<int>::of(&QSpinBox::valueChanged), ui->hue_upp_slider_1, &QSlider::setValue);

  ui->satr_low_slider_1->setValue(0);
  ui->satr_low_spinbox_1->setValue(0);
  connect(ui->satr_low_slider_1, &QSlider::valueChanged, ui->satr_low_spinbox_1, &QSpinBox::setValue);
  connect(ui->satr_low_spinbox_1, QOverload<int>::of(&QSpinBox::valueChanged), ui->satr_low_slider_1, &QSlider::setValue);

  ui->satr_upp_slider_1->setValue(255);
  ui->satr_upp_spinbox_1->setValue(255);
  connect(ui->satr_upp_slider_1, &QSlider::valueChanged, ui->satr_upp_spinbox_1, &QSpinBox::setValue);
  connect(ui->satr_upp_spinbox_1, QOverload<int>::of(&QSpinBox::valueChanged), ui->satr_upp_slider_1, &QSlider::setValue);

  ui->val_low_slider_1->setValue(0);
  ui->val_low_spinbox_1->setValue(0);
  connect(ui->val_low_slider_1, &QSlider::valueChanged, ui->val_low_spinbox_1, &QSpinBox::setValue);
  connect(ui->val_low_spinbox_1, QOverload<int>::of(&QSpinBox::valueChanged), ui->val_low_slider_1, &QSlider::setValue);

  ui->val_upp_slider_1->setValue(255);
  ui->val_upp_spinbox_1->setValue(255);
  connect(ui->val_upp_slider_1, &QSlider::valueChanged, ui->val_upp_spinbox_1, &QSpinBox::setValue);
  connect(ui->val_upp_spinbox_1, QOverload<int>::of(&QSpinBox::valueChanged), ui->val_upp_slider_1, &QSlider::setValue);

  // Repeat for each set of sliders and spinboxes based on the image
  ui->hue_low_slider_2->setValue(0);
  ui->hue_low_spinbox_2->setValue(0);
  connect(ui->hue_low_slider_2, &QSlider::valueChanged, ui->hue_low_spinbox_2, &QSpinBox::setValue);
  connect(ui->hue_low_spinbox_2, QOverload<int>::of(&QSpinBox::valueChanged), ui->hue_low_slider_2, &QSlider::setValue);

  ui->hue_upp_slider_2->setValue(180);
  ui->hue_upp_spinbox_2->setValue(180);
  connect(ui->hue_upp_slider_2, &QSlider::valueChanged, ui->hue_upp_spinbox_2, &QSpinBox::setValue);
  connect(ui->hue_upp_spinbox_2, QOverload<int>::of(&QSpinBox::valueChanged), ui->hue_upp_slider_2, &QSlider::setValue);

  ui->satr_low_slider_2->setValue(0);
  ui->satr_low_spinbox_2->setValue(0);
  connect(ui->satr_low_slider_2, &QSlider::valueChanged, ui->satr_low_spinbox_2, &QSpinBox::setValue);
  connect(ui->satr_low_spinbox_2, QOverload<int>::of(&QSpinBox::valueChanged), ui->satr_low_slider_2, &QSlider::setValue);

  ui->satr_upp_slider_2->setValue(255);
  ui->satr_upp_spinbox_2->setValue(255);
  connect(ui->satr_upp_slider_2, &QSlider::valueChanged, ui->satr_upp_spinbox_2, &QSpinBox::setValue);
  connect(ui->satr_upp_spinbox_2, QOverload<int>::of(&QSpinBox::valueChanged), ui->satr_upp_slider_2, &QSlider::setValue);

  ui->val_low_slider_2->setValue(0);
  ui->val_low_spinbox_2->setValue(0);
  connect(ui->val_low_slider_2, &QSlider::valueChanged, ui->val_low_spinbox_2, &QSpinBox::setValue);
  connect(ui->val_low_spinbox_2, QOverload<int>::of(&QSpinBox::valueChanged), ui->val_low_slider_2, &QSlider::setValue);

  ui->val_upp_slider_2->setValue(255);
  ui->val_upp_spinbox_2->setValue(255);
  connect(ui->val_upp_slider_2, &QSlider::valueChanged, ui->val_upp_spinbox_2, &QSpinBox::setValue);
  connect(ui->val_upp_spinbox_2, QOverload<int>::of(&QSpinBox::valueChanged), ui->val_upp_slider_2, &QSlider::setValue);

  ui->hue_low_slider_3->setValue(0);
  ui->hue_low_spinbox_3->setValue(0);
  connect(ui->hue_low_slider_3, &QSlider::valueChanged, ui->hue_low_spinbox_3, &QSpinBox::setValue);
  connect(ui->hue_low_spinbox_3, QOverload<int>::of(&QSpinBox::valueChanged), ui->hue_low_slider_3, &QSlider::setValue);

  ui->hue_upp_slider_3->setValue(180);
  ui->hue_upp_spinbox_3->setValue(180);
  connect(ui->hue_upp_slider_3, &QSlider::valueChanged, ui->hue_upp_spinbox_3, &QSpinBox::setValue);
  connect(ui->hue_upp_spinbox_3, QOverload<int>::of(&QSpinBox::valueChanged), ui->hue_upp_slider_3, &QSlider::setValue);

  ui->satr_low_slider_3->setValue(0);
  ui->satr_low_spinbox_3->setValue(0);
  connect(ui->satr_low_slider_3, &QSlider::valueChanged, ui->satr_low_spinbox_3, &QSpinBox::setValue);
  connect(ui->satr_low_spinbox_3, QOverload<int>::of(&QSpinBox::valueChanged), ui->satr_low_slider_3, &QSlider::setValue);

  ui->satr_upp_slider_3->setValue(255);
  ui->satr_upp_spinbox_3->setValue(255);
  connect(ui->satr_upp_slider_3, &QSlider::valueChanged, ui->satr_upp_spinbox_3, &QSpinBox::setValue);
  connect(ui->satr_upp_spinbox_3, QOverload<int>::of(&QSpinBox::valueChanged), ui->satr_upp_slider_3, &QSlider::setValue);

  ui->val_low_slider_3->setValue(0);
  ui->val_low_spinbox_3->setValue(0);
  connect(ui->val_low_slider_3, &QSlider::valueChanged, ui->val_low_spinbox_3, &QSpinBox::setValue);
  connect(ui->val_low_spinbox_3, QOverload<int>::of(&QSpinBox::valueChanged), ui->val_low_slider_3, &QSlider::setValue);

  ui->val_upp_slider_3->setValue(255);
  ui->val_upp_spinbox_3->setValue(255);
  connect(ui->val_upp_slider_3, &QSlider::valueChanged, ui->val_upp_spinbox_3, &QSpinBox::setValue);
  connect(ui->val_upp_spinbox_3, QOverload<int>::of(&QSpinBox::valueChanged), ui->val_upp_slider_3, &QSlider::setValue);

  ui->canny_low_slider->setValue(50);
  ui->canny_low_spinbox->setValue(50);
  connect(ui->canny_low_slider, &QSlider::valueChanged, ui->canny_low_spinbox, &QSpinBox::setValue);
  connect(ui->canny_low_spinbox, QOverload<int>::of(&QSpinBox::valueChanged), ui->canny_low_slider, &QSlider::setValue);

  ui->canny_upp_slider->setValue(150);
  ui->canny_upp_spinbox->setValue(150);
  connect(ui->canny_upp_slider, &QSlider::valueChanged, ui->canny_upp_spinbox, &QSpinBox::setValue);
  connect(ui->canny_upp_spinbox, QOverload<int>::of(&QSpinBox::valueChanged), ui->canny_upp_slider, &QSlider::setValue);

  ui->erode_slider->setValue(1);
  ui->erode_spinbox->setValue(1);
  connect(ui->erode_slider, &QSlider::valueChanged, ui->erode_spinbox, &QSpinBox::setValue);
  connect(ui->erode_spinbox, QOverload<int>::of(&QSpinBox::valueChanged), ui->erode_slider, &QSlider::setValue);

  ui->dilate_slider->setValue(1);
  ui->dilate_spinbox->setValue(1);
  connect(ui->dilate_slider, &QSlider::valueChanged, ui->dilate_spinbox, &QSpinBox::setValue);
  connect(ui->dilate_spinbox, QOverload<int>::of(&QSpinBox::valueChanged), ui->dilate_slider, &QSlider::setValue);

  // ROI initial values and connections
  ui->roi_p1_x_slider->setValue(160);
  ui->roi_p1_x_spinbox->setValue(160);
  connect(ui->roi_p1_x_slider, &QSlider::valueChanged, ui->roi_p1_x_spinbox, &QSpinBox::setValue);
  connect(ui->roi_p1_x_spinbox, QOverload<int>::of(&QSpinBox::valueChanged), ui->roi_p1_x_slider, &QSlider::setValue);

  ui->roi_p1_y_slider->setValue(90);
  ui->roi_p1_y_spinbox->setValue(90);
  connect(ui->roi_p1_y_slider, &QSlider::valueChanged, ui->roi_p1_y_spinbox, &QSpinBox::setValue);
  connect(ui->roi_p1_y_spinbox, QOverload<int>::of(&QSpinBox::valueChanged), ui->roi_p1_y_slider, &QSlider::setValue);

  ui->roi_p2_x_slider->setValue(160);
  ui->roi_p2_x_spinbox->setValue(160);
  connect(ui->roi_p2_x_slider, &QSlider::valueChanged, ui->roi_p2_x_spinbox, &QSpinBox::setValue);
  connect(ui->roi_p2_x_spinbox, QOverload<int>::of(&QSpinBox::valueChanged), ui->roi_p2_x_slider, &QSlider::setValue);

  ui->roi_p2_y_slider->setValue(270);
  ui->roi_p2_y_spinbox->setValue(270);
  connect(ui->roi_p2_y_slider, &QSlider::valueChanged, ui->roi_p2_y_spinbox, &QSpinBox::setValue);
  connect(ui->roi_p2_y_spinbox, QOverload<int>::of(&QSpinBox::valueChanged), ui->roi_p2_y_slider, &QSlider::setValue);

  ui->roi_p3_x_slider->setValue(480);
  ui->roi_p3_x_spinbox->setValue(480);
  connect(ui->roi_p3_x_slider, &QSlider::valueChanged, ui->roi_p3_x_spinbox, &QSpinBox::setValue);
  connect(ui->roi_p3_x_spinbox, QOverload<int>::of(&QSpinBox::valueChanged), ui->roi_p3_x_slider, &QSlider::setValue);

  ui->roi_p3_y_slider->setValue(270);
  ui->roi_p3_y_spinbox->setValue(270);
  connect(ui->roi_p3_y_slider, &QSlider::valueChanged, ui->roi_p3_y_spinbox, &QSpinBox::setValue);
  connect(ui->roi_p3_y_spinbox, QOverload<int>::of(&QSpinBox::valueChanged), ui->roi_p3_y_slider, &QSlider::setValue);

  ui->roi_p4_x_slider->setValue(480);
  ui->roi_p4_x_spinbox->setValue(480);
  connect(ui->roi_p4_x_slider, &QSlider::valueChanged, ui->roi_p4_x_spinbox, &QSpinBox::setValue);
  connect(ui->roi_p4_x_spinbox, QOverload<int>::of(&QSpinBox::valueChanged), ui->roi_p4_x_slider, &QSlider::setValue);

  ui->roi_p4_y_slider->setValue(90);
  ui->roi_p4_y_spinbox->setValue(90);
  connect(ui->roi_p4_y_slider, &QSlider::valueChanged, ui->roi_p4_y_spinbox, &QSpinBox::setValue);
  connect(ui->roi_p4_y_spinbox, QOverload<int>::of(&QSpinBox::valueChanged), ui->roi_p4_y_slider, &QSlider::setValue);

  ui->width_slider->setValue(640);
  ui->height_slider->setValue(360);
  connect(ui->width_slider, &QSlider::valueChanged, ui->width_spinbox, &QSpinBox::setValue);
  connect(ui->width_spinbox, QOverload<int>::of(&QSpinBox::valueChanged), ui->width_slider, &QSlider::setValue);
  connect(ui->height_slider, &QSlider::valueChanged, ui->height_spinbox, &QSpinBox::setValue);
  connect(ui->height_spinbox, QOverload<int>::of(&QSpinBox::valueChanged), ui->height_slider, &QSlider::setValue);

  // ROS 종료 시그널 처리
  connect(parameterImgNode, &ParameterImgNode::finished, this, &MainWindow::close);
}

MainWindow::~MainWindow()
{
  delete ui;
}

void MainWindow::closeEvent(QCloseEvent* event)
{
  QMainWindow::closeEvent(event);
}

void MainWindow::on_hue_low_slider_1_valueChanged(int value)
{
    parameterImgNode->setParameter("hsv1_hue_low", value);
}

void MainWindow::on_hue_upp_slider_1_valueChanged(int value)
{
    parameterImgNode->setParameter("hsv1_hue_upp", value);
}

void MainWindow::on_satr_low_slider_1_valueChanged(int value)
{
    parameterImgNode->setParameter("hsv1_satr_low", value);
}

void MainWindow::on_satr_upp_slider_1_valueChanged(int value)
{
    parameterImgNode->setParameter("hsv1_satr_upp", value);
}

void MainWindow::on_val_low_slider_1_valueChanged(int value)
{
    parameterImgNode->setParameter("hsv1_val_low", value);
}

void MainWindow::on_val_upp_slider_1_valueChanged(int value)
{
    parameterImgNode->setParameter("hsv1_val_upp", value);
}

void MainWindow::on_hue_low_slider_2_valueChanged(int value)
{
    parameterImgNode->setParameter("hsv2_hue_low", value);
}

void MainWindow::on_hue_upp_slider_2_valueChanged(int value)
{
    parameterImgNode->setParameter("hsv2_hue_upp", value);
}

void MainWindow::on_satr_low_slider_2_valueChanged(int value)
{
    parameterImgNode->setParameter("hsv2_satr_low", value);
}

void MainWindow::on_satr_upp_slider_2_valueChanged(int value)
{
    parameterImgNode->setParameter("hsv2_satr_upp", value);
}

void MainWindow::on_val_low_slider_2_valueChanged(int value)
{
    parameterImgNode->setParameter("hsv2_val_low", value);
}

void MainWindow::on_val_upp_slider_2_valueChanged(int value)
{
    parameterImgNode->setParameter("hsv2_val_upp", value);
}

void MainWindow::on_hue_low_slider_3_valueChanged(int value)
{
    parameterImgNode->setParameter("hsv3_hue_low", value);
}

void MainWindow::on_hue_upp_slider_3_valueChanged(int value)
{
    parameterImgNode->setParameter("hsv3_hue_upp", value);
}

void MainWindow::on_satr_low_slider_3_valueChanged(int value)
{
    parameterImgNode->setParameter("hsv3_satr_low", value);
}

void MainWindow::on_satr_upp_slider_3_valueChanged(int value)
{
    parameterImgNode->setParameter("hsv3_satr_upp", value);
}

void MainWindow::on_val_low_slider_3_valueChanged(int value)
{
    parameterImgNode->setParameter("hsv3_val_low", value);
}

void MainWindow::on_val_upp_slider_3_valueChanged(int value)
{
    parameterImgNode->setParameter("hsv3_val_upp", value);
}

void MainWindow::on_canny_low_slider_valueChanged(int value)
{
    parameterImgNode->setParameter("canny_low", value);
}

void MainWindow::on_canny_upp_slider_valueChanged(int value)
{
    parameterImgNode->setParameter("canny_upp", value);
}

void MainWindow::on_erode_slider_valueChanged(int value)
{
    parameterImgNode->setParameter("erode_size", value);
}

void MainWindow::on_dilate_slider_valueChanged(int value)
{
    parameterImgNode->setParameter("dilate_size", value);
}

void MainWindow::on_roi_p1_x_slider_valueChanged(int value)
{
    parameterImgNode->setParameter("roi_p1_x", value);
}

void MainWindow::on_roi_p1_y_slider_valueChanged(int value)
{
    parameterImgNode->setParameter("roi_p1_y", value);
}

void MainWindow::on_roi_p2_x_slider_valueChanged(int value)
{
    parameterImgNode->setParameter("roi_p2_x", value);
}

void MainWindow::on_roi_p2_y_slider_valueChanged(int value)
{
    parameterImgNode->setParameter("roi_p2_y", value);
}

void MainWindow::on_roi_p3_x_slider_valueChanged(int value)
{
    parameterImgNode->setParameter("roi_p3_x", value);
}

void MainWindow::on_roi_p3_y_slider_valueChanged(int value)
{
    parameterImgNode->setParameter("roi_p3_y", value);
}

void MainWindow::on_roi_p4_x_slider_valueChanged(int value)
{
    parameterImgNode->setParameter("roi_p4_x", value);
}

void MainWindow::on_roi_p4_y_slider_valueChanged(int value)
{
    parameterImgNode->setParameter("roi_p4_y", value);
}

void MainWindow::on_width_slider_valueChanged(int value)
{
    parameterImgNode->setParameter("width", value);
}

void MainWindow::on_height_slider_valueChanged(int value)
{
    parameterImgNode->setParameter("height", value);
}

void MainWindow::on_hsv_chbox_1_stateChanged(int arg1)
{
    parameterImgNode->setParameter("hsv1_binary", arg1 == Qt::Checked ? 1 : 0);
}

void MainWindow::on_hsv_chbox_2_stateChanged(int arg1)
{
    parameterImgNode->setParameter("hsv2_binary", arg1 == Qt::Checked ? 1 : 0);
}

void MainWindow::on_hsv_chbox_3_stateChanged(int arg1)
{
    parameterImgNode->setParameter("hsv3_binary", arg1 == Qt::Checked ? 1 : 0);
}

void MainWindow::on_canny_chbox_stateChanged(int arg1)
{
    parameterImgNode->setParameter("all_edge", arg1 == Qt::Checked ? 1 : 0);
}

