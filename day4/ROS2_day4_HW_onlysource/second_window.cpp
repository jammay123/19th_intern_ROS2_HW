#include "../include/cam_params_pkg/second_window.hpp"

SecondWindow::SecondWindow(QWidget* parent)
  : QMainWindow(parent), ui(new Ui::SecondWindowDesign)
{
  ui->setupUi(this);

  QIcon icon(":/images/icon.png");
  this->setWindowIcon(icon);

  // ProcessImgNode 초기화 및 시작
  processImgNode = new ProcessImgNode(rclcpp::Node::make_shared("image_processing_node"), this);
  processImgNode->start();

  // 각 이미지 처리 결과를 해당하는 UI 라벨에 연결
  connect(processImgNode, &ProcessImgNode::originalImageProcessed, this, &SecondWindow::updateOriginalImage);
  connect(processImgNode, &ProcessImgNode::frame1Processed, this, &SecondWindow::updateFrame1);
  connect(processImgNode, &ProcessImgNode::frame2Processed, this, &SecondWindow::updateFrame2);
  connect(processImgNode, &ProcessImgNode::hsv1Processed, this, &SecondWindow::updateHSV1);
  connect(processImgNode, &ProcessImgNode::hsv2Processed, this, &SecondWindow::updateHSV2);
  connect(processImgNode, &ProcessImgNode::hsv3Processed, this, &SecondWindow::updateHSV3);
}

SecondWindow::~SecondWindow()
{
  delete processImgNode;
  delete ui;
}

void SecondWindow::updateOriginalImage(const QImage& image)
{
    if (!image.isNull()) {
        // 이미지 크기 조정 (비율 유지, 중앙 정렬)
        ui->frame_org->setPixmap(QPixmap::fromImage(image).scaled(ui->frame_org->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
        ui->frame_org->setAlignment(Qt::AlignCenter);
    }
}

void SecondWindow::updateFrame1(const QImage& image)
{
    if (!image.isNull()) {
        // 이미지 크기 조정 (비율 유지, 중앙 정렬)
        ui->frame_1->setPixmap(QPixmap::fromImage(image).scaled(ui->frame_1->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
        ui->frame_1->setAlignment(Qt::AlignCenter);
    }
}

void SecondWindow::updateFrame2(const QImage& image)
{
    if (!image.isNull()) {
        // 이미지 크기 조정 (비율 유지, 중앙 정렬)
        ui->frame_2->setPixmap(QPixmap::fromImage(image).scaled(ui->frame_2->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
        ui->frame_2->setAlignment(Qt::AlignCenter);
    }
}

void SecondWindow::updateHSV1(const QImage& image)
{
    if (!image.isNull()) {
        QPixmap pixmap = QPixmap::fromImage(image);
        ui->frame_3->setPixmap(pixmap);
        ui->frame_3->setAlignment(Qt::AlignCenter);
    }
}

void SecondWindow::updateHSV2(const QImage& image)
{
    if (!image.isNull()) {
        QPixmap pixmap = QPixmap::fromImage(image);
        ui->frame_4->setPixmap(pixmap);
        ui->frame_4->setAlignment(Qt::AlignCenter);
    }
}

void SecondWindow::updateHSV3(const QImage& image)
{
    if (!image.isNull()) {
        QPixmap pixmap = QPixmap::fromImage(image);
        ui->frame_5->setPixmap(pixmap);
        ui->frame_5->setAlignment(Qt::AlignCenter);
    }
}

void SecondWindow::closeEvent(QCloseEvent* event)
{
    QMainWindow::closeEvent(event);
}
