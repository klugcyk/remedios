/*
    文件等级：密一
    author:klug
    献给我亲爱的师弟头盔闪亮的赫克托尔
    start:230228
    last:230605
*/

#ifndef galvo_H
#define galvo_H

#include <iostream>
#include "camera/camera.hpp"
#include "galvo_control/galvo_control.hpp"
#include "math/geometry.hpp"

#define galvo_msg_print_info
#define galvo_data_print_info
#define galvo_error_print_info

class galvo
{
public:
    galvo();
    ~galvo();

public:

protected:
    void galvo_calibrate(mathGeometry::geoLineParam shoot); //振镜标定
    mathGeometry::geoLineParam laser_line_shoot_galvo(float angle_ein,float angle_zwei);//计算在振镜坐标系下的直线方程
    mathGeometry::geoLineParam laser_line_shoot_camera(float angle_ein,float angle_zwei);//计算在相机坐标系下的直线方程
    void galvo2camera(std::vector<cv::Point3f> point,std::vector<mathGeometry::geoLineParam> line);

protected:
    cv::Point3f tran;
    float deviation_angel_ein; //振镜角度偏差
    float deviation_angel_zwei;
    mathGeometry::geoLineParam laser_source; //光源入射到第一振镜激光线的方向向量，振镜坐标系下，标定时用
    mathGeometry::geoLineParam laser_transmit; //激光线在两个振镜传递的方向向量，振镜坐标系下，标定时用
    mathGeometry::geoLineParam laser_shoot; //光源出射到到外部的方向向量,只在标定中使用，振镜坐标系下，标定时用

private:
    void get_panel(cv::Point3f p1,cv::Point3f p2,cv::Point3f p3,float &a,float &b,float &c,float &d);
    cv::Point3f line_plane_intersection(mathGeometry::geoLineParam line,mathGeometry::geoPlaneParam plane);
    mathGeometry::geoLineParam laser_reflect(mathGeometry::geoLineParam ld,mathGeometry::geoPlaneParam plane);
    mathGeometry::geoPlaneParam plane_rotate(mathGeometry::geoPlaneParam plane,mathGeometry::geoLineParam line,float angle);

private:
    mathGeometry::geoLineParam axia_ein; //第一振镜旋转轴线
    mathGeometry::geoLineParam axia_zwei; //第二振镜旋转轴线
    mathGeometry::geoPlaneParam ein_galvo_plane; //第一振镜平面参数，0度
    mathGeometry::geoPlaneParam zwei_galvo_plane; //第二振镜平面参数，0度

};

#endif // galvo_H
