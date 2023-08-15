/*
    文件等级：密一
    author:klug
    献给我亲爱的好友何塞阿尔卡迪奥
    start:230222
    last:230605
*/

#ifndef laser_length_measure_HPP
#define laser_length_measure_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <fstream>
#include <math.h>
#include "galvo/galvo.hpp"
#include "math/geometry.hpp"

#define laser_length_measure_print_msg_info
#define laser_length_measure_print_data_info
#define laser_length_measure_print_error_info
#define laser_length_measure_save_process
//#define laser_length_measure_cheak
#define calibrate_img_required 30 //标定所需图片数量

class laser_length_measure//:public galvo,public laser_zenturm_extract
{
public:
    laser_length_measure();
    ~laser_length_measure();

public:

protected:   
    int system_calibrate(std::vector<cv::Mat> cal_img);
    int system_calibrate(std::vector<cv::Mat> cal_img,std::vector<cv::Mat> laser_point_img);
    int base_calibrate(std::vector<cv::Mat> cal_img,std::vector<cv::Mat> laser_point_img);
    void laser_zenturm_caltest(cv::Mat &src_img,cv::Point2f &zenturm);
    double length_measure(cv::Mat src_img,double angle_ein,double angle_zwei); // 根据振镜的角度计算距离
    double length_measure_base(cv::Mat src_img);
    cv::Mat cameraMatrix=cv::Mat(3,3,CV_32FC1,cv::Scalar::all(0));
    cv::Mat distCoeffs=cv::Mat(1,5,CV_32FC1,cv::Scalar::all(0));

protected:
    cv::Point3f zenturm_coordinate; //激光中心坐标
    mathGeometry::geoLineParam laser_line; // 标定出射激光线在相机坐标系中的方程参数

private:
    void calculate_line(double point_array[10][3]);
    void calculate_line(double point_array[10][3],mathGeometry::geoLineParam &line);
    std::vector<cv::Mat> camera_calibrate(std::vector<cv::Mat> img_vector);

private:
    std::vector<cv::Mat> extrinsic_matrix;
    cv::Size board_size=cv::Size(6,9);
    cv::Size2f square_size=cv::Size2f(28,28);
    std::vector<cv::Mat> rvecsMat;
    std::vector<cv::Mat> rotation_matrix;
    std::vector<cv::Mat> tvecsMat;
};

#endif // laser_length_measure_HPP
