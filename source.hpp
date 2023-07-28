/*
    文件等级：密一
    author:klug
    献给我的心上人等待天使的妹妹，她的眼神常带月光
    start:230724
    last:230724
*/

#pragma once // 只编译一次

#ifndef source_hpp
#define source_hpp

#include <string>
#include <mutex>

extern bool link_update;
extern bool richt_update;

#define read_img_path "/home/klug/img/lengthMeasure/"
#define read_img_path_cal "/home/klug/img/lengthMeasure/cal/"
#define write_img_path "/home/klug/img/lengthMeasure/"
#define write_img_path_undistort "/home/klug/img/lengthMeasure/undistort/"
#define save_file_path "/home/klug/"
#define undistort_img_path "/home/klug/img/lengthMeasure/undistort/"
#define read_json_path "/home/klug/"
#define write_json_path "/home/klug/"

#define cal_img_num 30 //加载标定图片的数量
#define laserLineCnt 16 //结构光硬件上光线的条数

extern bool camera_continue_switch;

#endif //source_hpp
