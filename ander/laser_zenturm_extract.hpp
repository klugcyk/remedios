/*
    文件等级：密一
    author:klug
    献给不喜欢我的弗雷德里希冯海因洛特
    start:221230
    last:230328
*/

#ifndef IMG_PROCESS_HPP
#define IMG_PROCESS_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <fstream>
#include <math.h>

#define laser_zenturm_save
#define laser_zenturm_print_msg_info
#define laser_zenturm_circle_mark
#define laser_zenturm_print_data_info

using namespace cv;

class laser_zenturm_extract
{
public:
    laser_zenturm_extract(int r,int g,int b);
    laser_zenturm_extract();
    ~laser_zenturm_extract();

    cv::Mat laser_zenturm_nach(cv::Mat img,cv::Point2f &laser_center);
    cv::Mat laser_zenturm_fern(cv::Mat img,cv::Point2f &laser_center);

public:

private:
    std::vector<cv::Point2f> laser_zenturm_almost_find(cv::Mat img,cv::Mat &gray_img);
    bool judge_length(cv::Mat img);

private:
    int r_threshold;
    int g_threshold;
    int b_threshold;
    int up_width=15;
    int down_width=15;
    int link_width=15;
    int richt_width=15;
    int almost_roi=80; // 粗定位图像大小
    int almost_find_length=100; // 粗定位点距离阈值

};

#endif // IMG_PROCESS_HPP
