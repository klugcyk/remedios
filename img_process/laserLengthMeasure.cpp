/*
    文件等级：密一
    author:klug
    献给善良的黑魔导女孩
    start:230815
    last:230818
*/

#include "img_process/laserLengthMeasure.hpp"

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
    计算出射激光线方程，利用最小二乘法，计算出的结果为两个平面的交线
    @point_array:点坐标
*/
void laserLengthMeasure::calculate_line(double point_array[10][3],mathGeometry::geoLineParam &line)
{
    int n=10;
    double sum_z=0;
    double sum_y=0;
    double sum_x=0;
    double sum_z2=0;
    double sum_xz=0;
    double sum_yz=0;

    for(size_t i=0;i<n;i++)
    {
        sum_x+=point_array[i][0];
        sum_y+=point_array[i][1];
        sum_z+=point_array[i][2];

        sum_xz+=point_array[i][0]*point_array[i][2];
        sum_yz+=point_array[i][1]*point_array[i][2];
        sum_z2+=point_array[i][2]*point_array[i][2];
    }

    line.k1=(n*sum_xz-sum_x*sum_z)/(n*sum_z2-sum_z*sum_z);
    line.b1=(sum_x-line.k1*sum_z)/n;
    line.k2=(n*sum_yz-sum_y*sum_z)/(n*sum_z2-sum_z*sum_z);
    line.b2=(sum_y-line.k2*sum_z)/n;

    //vector ein
    double ax=1;
    double ay=0;
    double az=-line.k1;

    //vector zwei
    double bx=0;
    double by=1;
    double bz=-line.k2;

    line.x0=-line.b1;
    line.y0=-line.b2;
    line.z0=0;
    line.a=ay*bz-az*by;
    line.b=az*bx-ax*bz;
    line.c=ax*by-ay*bx;
}

/*
    激光线方程标定
    @imgArray:标定用图片，前25张用于相机标定,后10张用于直线方程计算
*/
void laserLengthMeasure::systemCalibrate(std::vector<cv::Mat> imgArray,std::vector<cv::Mat> laserArray)
{
    if(imgArray.size()<30)
    {
#ifdef laserLengthMeasurePrintErrorInfo
        printf("ERROR:img not enough...");
#endif
        return;
    }

    double fx=cameraMatrix.at<double>(0,0);
    double fy=cameraMatrix.at<double>(1,1);
    double u0=cameraMatrix.at<double>(0,2);
    double v0=cameraMatrix.at<double>(1,2);

    //相机标定
    board_size=cv::Size(8,11);
    square_size=cv::Size2f(5.00,5.00);
    cameraCalibrate(imgArray);

    //激光线标定
    //1 激光点坐标计算
    double pointArray[10][3];
    for(size_t imgCnt=0;imgCnt<laserArray.size();imgCnt++)
    {
        cv::Point2f zt;
        Eigen::Vector3d zt3d;
        zenturmExtractCal(laserArray[imgCnt],zt);

        double z=extrinsic[imgCnt+25].at<double>(2,3);
        pointArray[imgCnt][2]=z;
        pointArray[imgCnt][0]=(zt.x-u0)/fx*z;
        pointArray[imgCnt][1]=(zt.y-v0)/fy*z;
    }

    //2 最小二乘法拟合直线
    calculate_line(pointArray,laserLine);

#ifdef laserLengthMeasurePrintMsgInfo
    printf("Achieve the system calibrate...\n");
#endif
}

/*
    交比方程标定
    @imgArray:标定用图片，3张，0，10，20，30
*/
void laserLengthMeasure::systemCalibrate(std::vector<cv::Mat> laserArray)
{
    if(laserArray.size()!=4)
    {
#ifdef laserLengthMeasurePrintErrorInfo
        printf("ERROR:img num not correct...");
#endif
        return;
    }

    //交比计算
    for(size_t imgCnt=0;imgCnt<laserArray.size();imgCnt++)
    {
        cv::Point2f zt;
        zenturmExtractCal(laserArray[imgCnt],zt);
        pointArray[imgCnt][0]=zt.x;
        pointArray[imgCnt][1]=zt.y;
    }

#ifdef laserLengthMeasurePrintMsgInfo
    printf("Achieve the system calibrate...\n");
#endif
}

/*
    根据激光中心点测量距离
    @srcImg:激光中心点图像
*/
float laserLengthMeasure::lengthMeasure(cv::Mat srcImg)
{
    float length=-1;

    double fx=cameraMatrix.at<double>(0,0);
    double fy=cameraMatrix.at<double>(1,1);
    double u0=cameraMatrix.at<double>(0,2);
    double v0=cameraMatrix.at<double>(1,2);

    //1 计算激光中心点位置
    cv::Point2f zenturm;
    //cv::undistort(srcImg,srcImg,cameraMatrix,distCoeffs);
    zenturmExtract(srcImg,zenturm);
    //2 计算距离
    double t1=(zenturm.x-u0)/(laserLine.a*fx);
    double t2=(zenturm.y-v0)/(laserLine.b*fy);
    double t3=laserLine.x0/laserLine.a;
    double t4=laserLine.y0/laserLine.b;
    length=(t3-t4)/(t1-t2);
    double temp1=laserLine.b*laserLine.x0-laserLine.a*laserLine.y0;
    double temp2=(zenturm.x-u0)*laserLine.b/fx;
    double temp3=(zenturm.y-v0)*laserLine.a/fy;

    length=temp1/(temp2-temp3);

    zenturmCoordinate.z=length;
    zenturmCoordinate.x=laserLine.a*(length-laserLine.z0)/laserLine.c+laserLine.x0;
    zenturmCoordinate.y=laserLine.b*(length-laserLine.z0)/laserLine.c+laserLine.y0;

    length=sqrt(zenturmCoordinate.x*zenturmCoordinate.x+
                zenturmCoordinate.y*zenturmCoordinate.y+
                zenturmCoordinate.z*zenturmCoordinate.z);

    return length;
}

/*
    交比不变计算距离
    @srcImg:激光中心点图像
*/
float laserLengthMeasure::lengthMeasureCrossRatio(cv::Mat srcImg)
{
    cv::Point2f zenturm;
    //zenturmExtractCal(srcImg,zenturm);
    zenturmExtract(srcImg,zenturm);

    //超过标定量程
    if(zenturm.y>pointArray[2][1])
    {
#ifdef laserLengthMeasurePrintErrorInfo
        printf("overload...\n");
#endif
        return -1;
    }

    float t1=pointArray[0][1];//先前算出的点坐标
    float t2=zenturm.y;
    float t3=pointArray[2][1];
    float t4=pointArray[3][1];

    float l1=abs(t1-t2);
    float l2=abs(t2-t3);
    float l3=abs(t3-t4);

    crossRatio=(l2*(l1+l2+l3))/((l1+l2)*(l2+l3));//交比从图像中算出

    t1=Viel-Ein;
    t2=Drei-Ein;
    t3=Drei*t1-crossRatio*Viel*t2;
    t4=t1-crossRatio*t2;

    float length=t3/t4;//(60-60*crossRatio)/(3-2*crossRatio);

    return length;
}
