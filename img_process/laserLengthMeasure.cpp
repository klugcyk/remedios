/*
    文件等级：密一
    author:klug
    献给我亲爱的好友尼姆教授
    start:230724
    last:230724
*/

#include "img_process/laserLengthMeasure.hpp"

namespace lengthMeasure
{

laserLengthMeasure::laserLengthMeasure()
{
#ifdef laserLengthMeasurePrintMsgInfo
    printf("start the laser length measure...\n");
#endif

}

laserLengthMeasure::~laserLengthMeasure()
{
#ifdef laserLengthMeasurePrintMsgInfo
    printf("end the laser length measure...\n");
#endif

}

/*
    激光线方程标定
    @imgArray:标定用图片，前25张用于相机标定后10张用于直线方程计算
*/
void laserLengthMeasure::system_cal(std::vector<cv::Mat> imgArray)
{
    if(imgArray.size()<35)
    {
#ifdef laserLengthMeasurePrintErrorInfo
        printf("img not enough...");
#endif
        return ;
    }

    //相机标定
    cameraCalibrate(imgArray);

    //激光线标定
    std::vector<cv::Point3f> zenturm_array;
    for(size_t imgCnt=25;imgCnt<imgArray.size();imgCnt++)
    {
        cv::Point2f zt;
        cv::Point3f zt3d;
        zenturmExtract(imgArray[imgCnt],zt);

        zt3d.z=extrinsic[imgCnt].at<double>(2,3);
        zenturm_array.push_back(zt3d); //记录点在图像上的位置
    }
}

float laserLengthMeasure::lengthCalculate(cv::Mat srcImg)
{
    float length=0.1;

    double fx=cameraMatrix.at<double>(0,0);
    double fy=cameraMatrix.at<double>(1,1);
    double u0=cameraMatrix.at<double>(0,2);
    double v0=cameraMatrix.at<double>(1,2);

    //1 计算激光中心点位置
    cv::Point2f zenturm;
    zenturmExtract(srcImg,zenturm);
    //2 计算距离
    double temp1=laser_line.b*laser_line.x0-laser_line.a*laser_line.y0;
    double temp2=(zenturm.x-u0)*laser_line.b/fx;
    double temp3=(zenturm.y-v0)*laser_line.a/fy;

    length=temp1/(temp2-temp3);

    zenturm_coordinate.z=length;
    zenturm_coordinate.x=laser_line.a*(length-laser_line.z0)/laser_line.c+laser_line.x0;
    zenturm_coordinate.y=laser_line.b*(length-laser_line.z0)/laser_line.c+laser_line.y0;

    length=sqrt(zenturm_coordinate.x*zenturm_coordinate.x+
                zenturm_coordinate.y*zenturm_coordinate.y+
                zenturm_coordinate.z*zenturm_coordinate.z);

    return length;
}

};
