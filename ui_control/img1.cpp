/*
    文件等级:密一
    author:klug
    献给被妓女骗上床的皮埃特罗克雷斯皮
    start：230418
    last:230420
*/

#include "img1.h"
#include "ui_img1.h"
#include "img_process/construct_cal.hpp"

img1::img1(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::img1)
{
    ui->setupUi(this);
    setWindowTitle("img1");
}

img1::~img1()
{
    delete ui;
}

void img1::img_show(cv::Mat img)
{
    cvtColor(img,img,cv::COLOR_BGR2RGB);
    QImage img1((const unsigned char*)img.data,img.cols,img.rows,img.step,QImage::Format_RGB888);
    QImage qimg=img1.scaled(800,600).scaled(640,512,Qt::IgnoreAspectRatio,Qt::SmoothTransformation);
    ui->imgshow->setPixmap(QPixmap::fromImage(qimg));
    ui->imgshow->resize(qimg.size());
    ui->imgshow->show();
}

void img1::on_img_process_clicked()
{
    int ein=ui->img_name->text().toUInt();

    std::string name="/home/klug/img/construct/";
    name+=std::to_string(ein);
    name+=".png";

    cv::Mat img1=cv::imread(name);
    if(img1.empty())
    {
        return ;
    }

    cv::Mat img2(img1.rows,img1.cols,CV_8UC1);

    int p1,p2;

    p1=ui->param1->text().toInt();
    p2=ui->param2->text().toInt();

    grid_extract_preprocess(img1,img2,p1,p2);

    img_show(img2);
#ifdef img1_save
    cv::imwrite("/home/klug/img/construct/grid_extract_res.png",img2);
#endif
}
