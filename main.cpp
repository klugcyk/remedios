/*
    文件等级：密一
    author:klug
    献给我的心上人等待天使的妹妹
    start:221129
    last:230814
*/

#include "ui_control/mainwindow.h"
#include "ui_control/einlengthmeasure.h"
#include "source.hpp"
#include <QApplication>

bool camera_continue_switch=0; //开启相机连续采集图像
bool calDone=1;

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    MainWindow w;
    w.show();

    return a.exec();
}
