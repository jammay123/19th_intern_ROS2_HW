#ifndef CAM_PARAMS_PKG_SECOND_WINDOW_H
#define CAM_PARAMS_PKG_SECOND_WINDOW_H

#include <QMainWindow>
#include <QThread>
#include <QPixmap>
#include "ui_secondwindow.h"
#include "process_img_node.hpp"

class ProcessImgNode;  // 전방 선언으로 process_img_node 사용

class SecondWindow : public QMainWindow
{
  Q_OBJECT

public:
    explicit SecondWindow(QWidget* parent = nullptr);
    ~SecondWindow();

private slots:
    void updateOriginalImage(const QImage& image);
    void updateFrame1(const QImage& image);
    void updateFrame2(const QImage& image);
    void updateHSV1(const QImage& image);
    void updateHSV2(const QImage& image);
    void updateHSV3(const QImage& image);

protected:
    void closeEvent(QCloseEvent* event) override;

private:
    Ui::SecondWindowDesign* ui;
    ProcessImgNode* processImgNode;
};

#endif  // CAM_PARAMS_PKG_SECOND_WINDOW_H
