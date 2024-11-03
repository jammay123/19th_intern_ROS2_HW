/**
 * @file /include/my_chatter_pkg/main_window.hpp
 *
 * @brief Qt based gui for %(package)s.
 *
 * @date August 2024
 **/

#ifndef my_chatter_pkg_MAIN_WINDOW_H
#define my_chatter_pkg_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QMainWindow>
#include "QIcon"
#include "ui_mainwindow.h"
#include "talker_node.hpp"
#include "listener_node.hpp"

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow(QWidget* parent = nullptr);
  ~MainWindow();

  private slots:
  void on_pub_btn_clicked();

  private:
  Ui::MainWindowDesign* ui;
  TalkerNode* talker_node_;
  ListenerNode* listener_node_;
  void closeEvent(QCloseEvent* event);
};

#endif  // my_chatter_pkg_MAIN_WINDOW_H
