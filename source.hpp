/*
    文件等级：密一
    author:klug
    献给我的心上人等待天使的妹妹，她的眼神常带月光
    start:230724
    last:230815
*/

#pragma once // 只编译一次

#ifndef source_hpp
#define source_hpp

#include <string>
#include <mutex>

#define cal_img_path "/home/klug/img/lengthMeasure/cal/"
#define undistort_img_path "/home/klug/img/lengthMeasure/undistort/"

#define cal_img_num 30 //加载标定图片的数量
#define laserLineCnt 16 //结构光硬件上光线的条数

extern bool camera_continue_switch;
extern bool link_update;
extern bool richt_update;

#endif //source_hpp
