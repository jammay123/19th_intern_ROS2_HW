#include "../include/control_cam_params/main_window.hpp"

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::MainWindowDesign)
{
  ui->setupUi(this);

  QIcon icon("://ros-icon.png");
  this->setWindowIcon(icon);

  qnode = new QNode();
  parameter_qnode = new ParameterQNode();
  processing_qnode = new ProcessingQNode();

  QObject::connect(qnode, SIGNAL(rosShutDown()), this, SLOT(close()));
  connect(processing_qnode, &ProcessingQNode::imageOriginal, this, &MainWindow::updateImageOriginal, Qt::QueuedConnection);
  connect(processing_qnode, &ProcessingQNode::imageProcessed, this, &MainWindow::updateImageLabel, Qt::QueuedConnection);
  connect(processing_qnode, &ProcessingQNode::imageCannyProcessed, this, &MainWindow::updateImageCannyLabel, Qt::QueuedConnection); // Canny 신호 연결

  // 슬라이더 및 스핀박스 초기화 및 파라미터 설정
  ui->hue_upper_slider->setValue(180);
  ui->hue_upper_spinbox->setValue(180);
  connect(ui->hue_upper_slider, &QSlider::valueChanged, this, &MainWindow::on_hue_upper_slider_valueChanged);
  connect(ui->hue_upper_slider, &QSlider::valueChanged, ui->hue_upper_spinbox, &QSpinBox::setValue);

  connect(ui->hue_upper_spinbox, QOverload<int>::of(&QSpinBox::valueChanged), ui->hue_upper_slider, &QSlider::setValue);

  ui->hue_lower_slider->setValue(0);
  ui->hue_lower_spinbox->setValue(180);
  connect(ui->hue_lower_slider, &QSlider::valueChanged, this, &MainWindow::on_hue_lower_slider_valueChanged);
  connect(ui->hue_lower_slider, &QSlider::valueChanged, ui->hue_lower_spinbox, &QSpinBox::setValue);
  connect(ui->hue_lower_spinbox, QOverload<int>::of(&QSpinBox::valueChanged), ui->hue_lower_slider, &QSlider::setValue);

  ui->satr_upper_slider->setValue(255);
  ui->satr_upper_spinbox->setValue(255);
  connect(ui->satr_upper_slider, &QSlider::valueChanged, this, &MainWindow::on_satr_upper_slider_valueChanged);
  connect(ui->satr_upper_slider, &QSlider::valueChanged, ui->satr_upper_spinbox, &QSpinBox::setValue);
  connect(ui->satr_upper_spinbox, QOverload<int>::of(&QSpinBox::valueChanged), ui->satr_upper_slider, &QSlider::setValue);

  ui->satr_lower_slider->setValue(0);
  ui->satr_lower_spinbox->setValue(0);
  connect(ui->satr_lower_slider, &QSlider::valueChanged, this, &MainWindow::on_satr_lower_slider_valueChanged);
  connect(ui->satr_lower_slider, &QSlider::valueChanged, ui->satr_lower_spinbox, &QSpinBox::setValue);
  connect(ui->satr_lower_spinbox, QOverload<int>::of(&QSpinBox::valueChanged), ui->satr_lower_slider, &QSlider::setValue);

  ui->value_upper_slider->setValue(255);
  ui->value_upper_spinbox->setValue(255);
  connect(ui->value_upper_slider, &QSlider::valueChanged, this, &MainWindow::on_value_upper_slider_valueChanged);
  connect(ui->value_upper_slider, &QSlider::valueChanged, ui->value_upper_spinbox, &QSpinBox::setValue);
  connect(ui->value_upper_spinbox, QOverload<int>::of(&QSpinBox::valueChanged), ui->value_upper_slider, &QSlider::setValue);

  ui->value_lower_slider->setValue(0);
  ui->value_lower_spinbox->setValue(0);
  connect(ui->value_lower_slider, &QSlider::valueChanged, this, &MainWindow::on_value_lower_slider_valueChanged);
  connect(ui->value_lower_slider, &QSlider::valueChanged, ui->value_lower_spinbox, &QSpinBox::setValue);
  connect(ui->value_lower_spinbox, QOverload<int>::of(&QSpinBox::valueChanged), ui->value_lower_slider, &QSlider::setValue);

  ui->image_width_slider->setValue(640);
  ui->img_width_spinbox->setValue(640);
  connect(ui->image_width_slider, &QSlider::valueChanged, this, &MainWindow::on_image_width_slider_valueChanged);
  connect(ui->image_width_slider, &QSlider::valueChanged, ui->img_width_spinbox, &QSpinBox::setValue);
  connect(ui->img_width_spinbox, QOverload<int>::of(&QSpinBox::valueChanged), ui->image_width_slider, &QSlider::setValue);

  ui->image_height_slider->setValue(360);
  ui->img_height_spinbox->setValue(640);
  connect(ui->image_height_slider, &QSlider::valueChanged, this, &MainWindow::on_image_height_slider_valueChanged);
  connect(ui->image_height_slider, &QSlider::valueChanged, ui->img_height_spinbox, &QSpinBox::setValue);
  connect(ui->img_height_spinbox, QOverload<int>::of(&QSpinBox::valueChanged), ui->image_height_slider, &QSlider::setValue);

  ui->erode_cnt_slider->setValue(0);
  ui->erode_cnt_spinbox->setValue(0);
  connect(ui->erode_cnt_slider, &QSlider::valueChanged, this, &MainWindow::on_erode_cnt_slider_valueChanged);
  connect(ui->erode_cnt_slider, &QSlider::valueChanged, ui->erode_cnt_spinbox, &QSpinBox::setValue);
  connect(ui->erode_cnt_spinbox, QOverload<int>::of(&QSpinBox::valueChanged), ui->erode_cnt_slider, &QSlider::setValue);


  ui->dilate_cnt_slider->setValue(0);
  ui->dilate_cnt_spinbox->setValue(0);
  connect(ui->dilate_cnt_slider, &QSlider::valueChanged, this, &MainWindow::on_dilate_cnt_slider_valueChanged);
  connect(ui->dilate_cnt_slider, &QSlider::valueChanged, ui->dilate_cnt_spinbox, &QSpinBox::setValue);
  connect(ui->dilate_cnt_spinbox, QOverload<int>::of(&QSpinBox::valueChanged), ui->dilate_cnt_slider, &QSlider::setValue);

  ui->canny_max_slider->setValue(200);
  ui->canny_max_spinbox->setValue(200);
  connect(ui->canny_max_slider, &QSlider::valueChanged, this, &MainWindow::on_canny_max_slider_valueChanged);
  connect(ui->canny_max_slider, &QSlider::valueChanged, ui->canny_max_spinbox, &QSpinBox::setValue);
  connect(ui->canny_max_spinbox, QOverload<int>::of(&QSpinBox::valueChanged), ui->canny_max_slider, &QSlider::setValue);

  ui->canny_min_slider->setValue(100);
  ui->canny_min_spinbox->setValue(100);
  connect(ui->canny_min_slider, &QSlider::valueChanged, this, &MainWindow::on_canny_min_slider_valueChanged);
  connect(ui->canny_min_slider, &QSlider::valueChanged, ui->canny_min_spinbox, &QSpinBox::setValue);
  connect(ui->canny_min_spinbox, QOverload<int>::of(&QSpinBox::valueChanged), ui->canny_min_slider, &QSlider::setValue);
}

MainWindow::~MainWindow()
{
  delete ui;
  delete qnode;
  delete parameter_qnode;
  delete processing_qnode;  // processing_qnode 삭제 추가
}

void MainWindow::closeEvent(QCloseEvent* event)
{
    QMainWindow::closeEvent(event);  // 기본 close 이벤트 처리 호출
}

void MainWindow::updateImageLabel(const QImage &image)
{
    QPixmap pixmap = QPixmap::fromImage(image);
    ui->img_label_2->setPixmap(pixmap);
    ui->img_label_2->setAlignment(Qt::AlignCenter);
}

void MainWindow::updateImageCannyLabel(const QImage &image)
{
    QPixmap pixmap = QPixmap::fromImage(image);
    ui->img_label_3->setPixmap(pixmap);
    ui->img_label_3->setAlignment(Qt::AlignCenter);
}

void MainWindow::updateImageOriginal(const QImage &image)
{
    QPixmap pixmap = QPixmap::fromImage(image);
    ui->img_label_1->setPixmap(pixmap.scaled(ui->img_label_1->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    ui->img_label_1->setAlignment(Qt::AlignCenter);
}

void MainWindow::on_hue_upper_slider_valueChanged(int value)
{
  parameter_qnode->setHueUpper(value);
}

void MainWindow::on_hue_lower_slider_valueChanged(int value)
{
  parameter_qnode->setHueLower(value);
}

void MainWindow::on_satr_upper_slider_valueChanged(int value)
{
  parameter_qnode->setSatrUpper(value);
}

void MainWindow::on_satr_lower_slider_valueChanged(int value)
{
  parameter_qnode->setSatrLower(value);
}

void MainWindow::on_value_upper_slider_valueChanged(int value)
{
  parameter_qnode->setValUpper(value);
}

void MainWindow::on_value_lower_slider_valueChanged(int value)
{
  parameter_qnode->setValLower(value);
}

void MainWindow::on_image_width_slider_valueChanged(int value)
{
  parameter_qnode->setImgWidth(value);
}

void MainWindow::on_image_height_slider_valueChanged(int value)
{
  parameter_qnode->setImgHeight(value);
}


void MainWindow::on_erode_cnt_slider_valueChanged(int value)
{
  parameter_qnode->setErodeCnt(value);
}


void MainWindow::on_dilate_cnt_slider_valueChanged(int value)
{
  parameter_qnode->setDilateCnt(value);
}



void MainWindow::on_canny_max_slider_valueChanged(int value)
{
  parameter_qnode->setCannyMax(value);
}


void MainWindow::on_canny_min_slider_valueChanged(int value)
{
  parameter_qnode->setCannyMax(value);
}

