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

#include "../include/my_chatter_pkg/main_window.hpp"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::MainWindowDesign), talker_node_(new TalkerNode(this)), listener_node_(new ListenerNode(this))
{
  ui->setupUi(this);

  connect(ui->pub_btn, &QPushButton::clicked, this, &MainWindow::on_pub_btn_clicked);
  connect(listener_node_, &ListenerNode::messageReceived, ui->label, &QLabel::setText);
}

void MainWindow::closeEvent(QCloseEvent* event)
{
  QMainWindow::closeEvent(event);
}

MainWindow::~MainWindow()
{
  delete ui;
  delete talker_node_;
  delete listener_node_;
}

void MainWindow::on_pub_btn_clicked()
{
  QString message = ui->textEdit->toPlainText();
  talker_node_->publishMessage(message);
}

