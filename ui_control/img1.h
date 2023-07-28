/*
    文件等级:密一
    author:klug
    献给我的师弟长发的阿开奥斯人
    start：230418
    last:230420
*/

#ifndef IMG1_H
#define IMG1_H

#include <QWidget>
#include "img_process/construct_cal.hpp"
#include "ui_control/mainwindow.h"

#define img1_save

namespace Ui {
class img1;
}

class img1 : public QWidget,public construct_cal
{
    Q_OBJECT

public:
    explicit img1(QWidget *parent = nullptr);
    ~img1();

private slots:
    void on_img_process_clicked();

private:
    Ui::img1 *ui;
    void img_show(cv::Mat img);

};

#endif // IMG1_H
