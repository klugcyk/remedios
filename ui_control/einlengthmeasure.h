/*
    文件等级：密一
    author:klug
    献给我的弟弟奥雷里亚诺第二
    start:230724
    last:230724
*/

#ifndef EINLENGTHMEASURE_H
#define EINLENGTHMEASURE_H

#include <QDockWidget>
#include "img_process/zenturmExtract.hpp"
#include "img_process/laserLengthMeasure.hpp"
#include <opencv2/opencv.hpp>
#include "camera/camera.hpp"

#define imgShowDivide 2 //显示在窗口上图像缩小比例

namespace Ui {
class einLengthMeasure;
}

class einLengthMeasure : public QDockWidget,
        public lengthMeasure::laserLengthMeasure,
        public basler_camera
{
    Q_OBJECT

public:
    explicit einLengthMeasure(QWidget *parent = nullptr);
    ~einLengthMeasure();

private slots:
    void on_grab_clicked();
    void on_save_clicked();
    void on_measure_clicked();

    void on_func_clicked();

private:
    Ui::einLengthMeasure *ui;
    void imgShow(cv::Mat img);
    void imgShow();
    cv::Mat grabImg;

};

#endif // EINLENGTHMEASURE_H
