/*
    文件等级：密一
    author:klug
    献给我的心上人等待天使的妹妹
    start:230425
    last:230425
*/

#ifndef MACHINE_H
#define MACHINE_H

#include <QWidget>

namespace Ui {
class machine;
}

class machine : public QWidget
{
    Q_OBJECT

public:
    explicit machine(QWidget *parent = nullptr);
    ~machine();

private:
    Ui::machine *ui;
};

#endif // MACHINE_H
