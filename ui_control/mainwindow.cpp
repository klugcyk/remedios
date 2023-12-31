/*
    文件等级：密一
    author:klug
    献给美人儿蕾梅黛丝
    start:230427
    last:230818
*/

#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include "img_process/laserLengthMeasure.hpp"
#include "camera/camera.hpp"
#include "ui_control/einlengthmeasure.h"
#include "source.hpp"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setWindowTitle("laser_length_measure");

    //calDone=1;

    if(!calDone)
    {
        //system calibration
        std::vector<cv::Mat> cal_img;
        std::vector<cv::Mat> laser_img;

        //load the image from the path 20 for camera 10 for laser line
        /*int cal_img_cnt=30;//加载图片的数量
        for(int i=1;i<=cal_img_cnt;i++)
        {
            std::string path=cal_img_path;
            path+=std::to_string(i);
            path+=".png";
            cv::Mat temp_img=imread(path);
            if(!temp_img.empty())
            {
                cal_img.push_back(temp_img);
            }
        }*/

        for(int i=1;i<=4;i++)
        {
            std::string path=cal_img_path;
            path+=std::to_string(i);
            path+=".png";
            cv::Mat _img=imread(path);
            if(!_img.empty())
            {
                laser_img.push_back(_img);
            }
            else
            {
                std::cout<<"img "
                         << i
                         <<" not find"
                         <<std::endl;
            }
        }

        systemCalibrate(laser_img);
        calDone=1;
    }
}

MainWindow::~MainWindow()
{
    delete ui;
}

#ifdef camera_gray

void MainWindow::on_grab_clicked()
{
    camera_grab_gray();
    img_show();
}

#endif

#ifdef camera_rgb

void MainWindow::on_grab_clicked()
{
    camera_grab_rgb();
    cv::Mat img=cv::imread("/home/klug/img/cam_img.png");
    res_show(img);
}

#endif

void MainWindow::on_set_param_clicked()
{
    set_exposure_time=ui->exposure_time_set->text().toInt();
    camera_set_parameter();
    printf("set the parameter...\n");
}

void MainWindow::on_read_param_clicked()
{
    camera_read_parameter();

    std::string temp=std::to_string(read_exposure_time);
    QString et=QString::fromStdString(temp);
    ui->exposure_time_set->setText(et);
}

void MainWindow::on_save_clicked()
{
    int i=1;

    i=ui->img_name->text().toInt();
    std::string num=std::to_string(i);
    std::string n;
    QString name;
    QString num_;
    num_=QString::fromStdString(num);
    n="/home/klug/img/lengthMeasure/";
    if(ui->forCal->isChecked())
    {
        n+="cal/";
    }
    else
    {
        n+="measure/";
    }
    n+=std::to_string(i);
    n+=".png";

    cv::Mat img=cv::imread("/home/klug/img/cam_img.png");
    if(!img.empty())
    {
        cv::imwrite(n,img);
    }
}

void MainWindow::on_measure_clicked()
{
    std::string img_path="/home/klug/img/lengthMeasure/measure/";
    std::string img_num=ui->img_name->text().toStdString();
    img_path+=img_num;
    img_path+=".png";

    cv::Mat img=imread(img_path);

    if(!img.empty())
    {
        //double length=lengthMeasure(img);
        double length=lengthMeasureCrossRatio(img);
        QString str = QString::number(length,'f',3);
        ui->distance->setText(str);
    }
}
/*
void MainWindow::on_galvo_read_clicked()
{
    galvo_read();
    std::string temp=std::to_string(ein_galvo_angle);
    QString angle=QString::fromStdString(temp);
    ui->galvo_1->setText(angle);
    temp=std::to_string(zwei_galvo_angle);
    angle=QString::fromStdString(temp);
    ui->galvo_2->setText(angle);
}

void MainWindow::on_galvo_rotate_clicked()
{
    float angle1,angle2;
    angle1=ui->galvo_1->text().toFloat();
    angle2=ui->galvo_2->text().toFloat();
    galvo_rotate(angle1,angle2);
}*/

void MainWindow::res_show(cv::Mat &res_img)
{
    cvtColor(res_img,res_img,cv::COLOR_BGR2RGB);
    QImage img((const unsigned char*)res_img.data,res_img.cols,res_img.rows,res_img.step,QImage::Format_RGB888);
    QImage qimg=img.scaled(800,600).scaled(640,512,Qt::IgnoreAspectRatio,Qt::SmoothTransformation);
    ui->img_continue->setPixmap(QPixmap::fromImage(qimg));
    ui->img_continue->resize(qimg.size());
    ui->img_continue->show();
}

