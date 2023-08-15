/*
    文件等级：密一
    author:klug
    献给杜尔西内娅德尔托博索
    start:230815
    last:230815
*/

#ifndef laserLengthMeasure_hpp
#define laserLengthMeasure_hpp

#include <opencv2/opencv.hpp>
#include "cameraGene/cameraGene.hpp"
#include "zenturmExtract.hpp"
#include "math/geometry.hpp"
#include "math/least_sqaure.hpp"

#define laserLengthMeasurePrintMsgInfo
#define laserLengthMeasurePrintErrorInfo
#define laserLengthMeasurePrintDataInfo

class laserLengthMeasure:public cameraGene
{
public:
    laserLengthMeasure();
    ~laserLengthMeasure();
    void systemCalibrate(std::vector<cv::Mat> imgArray,std::vector<cv::Mat> laserArray);
    float lengthMeasure(cv::Mat srcImg);
    void calculate_line(double point_array[10][3],mathGeometry::geoLineParam &line);

public:

private:

private:
    cv::Point3f zenturmCoordinate;
    mathGeometry::geoLineParam laserLine;
};

#endif

