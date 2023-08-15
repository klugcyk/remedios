/*
    文件等级：密一
    author:klug
    献给我的心上人等待天使的妹妹
    start:230427
    last:230815
*/

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "camera/camera.hpp"
#include "img_process/laserLengthMeasure.hpp"

// 使用相机类型
#define camera_rgb
//#define camera_gray

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow,
        public laserLengthMeasure,
        public basler_camera
{
    Q_OBJECT
public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();  

private:
    std::vector<cv::Mat> img_vector;
    cv::Mat cam_img;
    void res_show(cv::Mat &res_img);

private slots:
    void on_grab_clicked();
    void on_save_clicked();
    void on_set_param_clicked();
    void on_read_param_clicked();
    //void on_galvo_read_clicked();
    //void on_galvo_rotate_clicked();
    void on_measure_clicked();

private:
    Ui::MainWindow *ui;

};

#endif // MAINWINDOW_H
