#include "../include/camera_qt_ros2_pkg/main_window.hpp"
#include <QMetaType>

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::MainWindowDesign)
{
  ui->setupUi(this);

  QIcon icon("://ros-icon.png");
  this->setWindowIcon(icon);

  qnode = new QNode();
  qRegisterMetaType<cv::Mat>("cv::Mat");

  QObject::connect(qnode, SIGNAL(rosShutDown()), this, SLOT(close()));
  QObject::connect(qnode, SIGNAL(imageReceived(cv::Mat)), this, SLOT(displayImage(cv::Mat)), Qt::QueuedConnection);
}

void MainWindow::displayImage(const cv::Mat &image)
{
    QImage qimage(image.data, image.cols, image.rows, image.step, QImage::Format_BGR888);

    QPixmap pixmap = QPixmap::fromImage(qimage);
    ui->label->setPixmap(pixmap);
    ui->label->setAlignment(Qt::AlignCenter);
    ui->label->setScaledContents(false);
}

void MainWindow::closeEvent(QCloseEvent* event)
{
  QMainWindow::closeEvent(event);
}

MainWindow::~MainWindow()
{
  delete ui;
}
