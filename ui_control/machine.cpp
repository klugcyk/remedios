/*
    文件等级：密一
    author:klug
    献给我的心上人等待天使的妹妹
    start:230425
    last:230425
*/

#include "machine.h"
#include "ui_machine.h"

machine::machine(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::machine)
{
    ui->setupUi(this);
}

machine::~machine()
{
    delete ui;
}
