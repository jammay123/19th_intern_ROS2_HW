/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date August 2024
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include "../include/control_turtle_pkg/main_window.hpp"

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::MainWindowDesign)
{
    ui->setupUi(this);

    QIcon icon("://ros-icon.png");
    this->setWindowIcon(icon);

    qnode = new QNode();            //ros2 통신을 위한 qnode 객체 생성

    QObject::connect(qnode, SIGNAL(rosShutDown()), this, SLOT(close()));        //ros2 종료시 gui도 함께 종료
    connect(qnode, &QNode::twistReceived, this, &MainWindow::updateTextBrowser);

    qnode->start();
}

void MainWindow::closeEvent(QCloseEvent* event)
{
    QMainWindow::closeEvent(event);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_w_btn_clicked()
{
    qnode->sendTwist(1.0, 0.0);
}


void MainWindow::on_s_btn_clicked()
{
    qnode->sendTwist(-1.0, 0.0);
}


void MainWindow::on_a_btn_clicked()
{
    qnode->sendTwist(0.0, 1.0);
}


void MainWindow::on_d_btn_clicked()
{
    qnode->sendTwist(0.0, -1.0);
}

void MainWindow::on_line_b_btn_clicked()
{
    int width = ui->pen_width->value();
    lastColor = "b";
    qnode->setPen(lastColor, width);
}


void MainWindow::on_line_g_btn_clicked()
{
    int width = ui->pen_width->value();
    lastColor = "g";
    qnode->setPen(lastColor, width);
}


void MainWindow::on_line_r_btn_clicked()
{
    int width = ui->pen_width->value();
    lastColor = "r";
    qnode->setPen(lastColor, width);
}

void MainWindow::on_backgnd_r_btn_clicked()
{
    qnode->set_background_color("r");
}


void MainWindow::on_backgnd_g_btn_clicked()
{
    qnode->set_background_color("g");
}


void MainWindow::on_backgnd_b_btn_clicked()
{
    qnode->set_background_color("b");
}

void MainWindow::on_circle_btn_clicked() {
    int radius = ui->draw_slider->value(); // 슬라이더 값으로 반지름 설정
    qnode->drawCircle(radius, 2.0); // 반지름과 속도 설정
}

void MainWindow::on_square_btn_clicked() {
    int side_length = ui->draw_slider->value(); // 슬라이더 값으로 변의 길이 설정
    qnode->drawSquare(side_length, 2.0); // 변의 길이와 속도 설정
}

void MainWindow::on_triangle_btn_clicked() {
    int side_length = ui->draw_slider->value(); // 슬라이더 값으로 변의 길이 설정
    qnode->drawTriangle(side_length, 2.0); // 변의 길이와 속도 설정
}

void MainWindow::on_pen_width_actionTriggered(int action)
{
    int width = ui->pen_width->value();
    qnode->setPen(lastColor, width);
}


void MainWindow::on_draw_slider_actionTriggered(int action)
{

}

void MainWindow::updateTextBrowser(double linear, double angular) {
    QString text = QString("Linear: %1, Angular: %2").arg(linear).arg(angular);
    ui->textBrowser->append(text);
}

