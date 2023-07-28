/*
    文件等级：密一
    author:klug
    献给我的弟弟奥雷里亚诺第二
    start:230724
    last:230724
*/

#include "einlengthmeasure.h"
#include "ui_einlengthmeasure.h"
#include "source.hpp"

einLengthMeasure::einLengthMeasure(QWidget *parent):
    QDockWidget(parent),
    ui(new Ui::einLengthMeasure)
{
    ui->setupUi(this);
    setWindowTitle("ein_measure");
}

einLengthMeasure::~einLengthMeasure()
{
    delete ui;
}

void einLengthMeasure::imgShow(cv::Mat img)
{
    QImage qqimg((const unsigned char*)img.data,img.cols,img.rows,img.step,QImage::Format_RGB888);
    QImage qimg=qqimg.scaled(img.cols/imgShowDivide,img.rows/imgShowDivide).scaled(img.cols/imgShowDivide,img.rows/imgShowDivide,Qt::IgnoreAspectRatio,Qt::SmoothTransformation);
    ui->img_continue->setPixmap(QPixmap::fromImage(qimg));
    ui->img_continue->resize(qimg.size());
    ui->img_continue->show();
}

void einLengthMeasure::imgShow()
{
    QImage qqimg((const unsigned char*)grabImg.data,grabImg.cols,grabImg.rows,grabImg.step,QImage::Format_RGB888);
    QImage qimg=qqimg.scaled(grabImg.cols/imgShowDivide,grabImg.rows/imgShowDivide).scaled(grabImg.cols/imgShowDivide,grabImg.rows/imgShowDivide,Qt::IgnoreAspectRatio,Qt::SmoothTransformation);
    ui->img_continue->setPixmap(QPixmap::fromImage(qimg));
    ui->img_continue->resize(qimg.size());
    ui->img_continue->show();
}

void einLengthMeasure::on_grab_clicked()
{
    grabImg=camera_grab_rgb_return();
    imgShow();
}

void einLengthMeasure::on_save_clicked()
{
    if(!grabImg.empty())
    {
        std::string name=write_img_path;
        if(ui->forCal->isChecked())
        {
            name+="cal/";
        }
        name+=std::to_string(ui->imgName->text().toInt());
        name+=".png";

        cv::imwrite(name,grabImg);
    }
}

void einLengthMeasure::on_measure_clicked()
{
    grabImg=camera_grab_rgb_return();
    float l=lengthCalculate(grabImg);
    ui->length->setText(QString::number(l,'f',5));
}

void einLengthMeasure::on_func_clicked()
{
    cv::Mat img=cv::imread("/home/klug/img/length_measure/1.png");
    imgShow(img);
}

