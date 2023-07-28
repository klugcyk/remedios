/*
    文件等级：密一
    author:klug
    献给我亲爱的师弟头盔闪亮的赫克托尔
    start:230724
    last:230724
*/

#ifndef laserLengthMeasure_HPP
#define laserLengthMeasure_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <fstream>
#include <math.h>
#include "math/geometry.hpp"
#include "cameraGene/cameraGene.hpp"
#include "img_process/zenturmExtract.hpp"

#define laserLengthMeasurePrintMsgInfo
#define laserLengthMeasurePrintDataInfo
#define laserLengthMeasurePrintErrorInfo
#define laserLengthMeasureSaveProcess

namespace lengthMeasure
{

// 单点激光测距不加振镜
class laserLengthMeasure:private cameraGene
{
public:
    laserLengthMeasure();
    ~laserLengthMeasure();

public:

protected:   
    void system_cal(std::vector<cv::Mat> imgArray);
    float lengthCalculate(cv::Mat srcImg);

protected:

private:

private:
    math_geometry::geo_line_param laser_line; //激光线在相机坐标系下直线方程
    cv::Point3f zenturm_coordinate; //提取的激光中线点在相机坐标系下的三维坐标
};

};

#endif // laserLengthMeasure_HPP
