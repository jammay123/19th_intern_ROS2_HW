/**
 * @file /include/control_turtle_pkg/main_window.hpp
 *
 * @brief Qt based gui for %(package)s.
 *
 * @date August 2024
 **/

#ifndef control_turtle_pkg_MAIN_WINDOW_H
#define control_turtle_pkg_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QMainWindow>
#include "QIcon"
#include "qnode.hpp"
#include "ui_mainwindow.h"

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
    QNode* qnode;

private slots:
    void on_w_btn_clicked();

    void on_s_btn_clicked();

    void on_a_btn_clicked();

    void on_d_btn_clicked();

    void on_line_b_btn_clicked();

    void on_line_g_btn_clicked();

    void on_line_r_btn_clicked();

    void on_backgnd_r_btn_clicked();

    void on_backgnd_g_btn_clicked();

    void on_backgnd_b_btn_clicked();

    void on_circle_btn_clicked();

    void on_square_btn_clicked();

    void on_triangle_btn_clicked();

    void on_pen_width_actionTriggered(int action);

    void on_draw_slider_actionTriggered(int action);

    void updateTextBrowser(double linear, double angular);

private:
    Ui::MainWindowDesign* ui;
    std::string lastColor;
    void closeEvent(QCloseEvent* event);
};

#endif  // control_turtle_pkg_MAIN_WINDOW_H
