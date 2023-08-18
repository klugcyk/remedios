/*
    文件等级：密一
    author:klug
    献给我亲爱的师弟头盔闪亮的赫克托尔
    start:230724
    last:230818
*/

#ifndef zenturmExtract_HPP
#define zenturmExtract_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#define zenturmExtractSave
#define zenturmExtractPrintData
#define zenturmExtractPrintMsg
#define zenturmExtractPrintError
#define zenturmExtractMark

void zenturmExtract_(cv::Mat srcImg,cv::Point2f &zenturm);
void zenturmExtract(cv::Mat srcImg,cv::Point2f &zenturm);
void zenturmExtractCal(cv::Mat srcImg,cv::Point2f &zenturm);
void zenturmCalculate(cv::Mat srcImg,cv::Point2f &zenturm);
void zenturmCalculateGray(cv::Mat srcImg,cv::Point2f &zenturm);
void zenturmCalculateContour(cv::Mat srcImg,cv::Point2f &zenturm);

const int roiRow=200;
const int roiCol=200;
const int offsetX=1200;
const int offsetY=980;

#endif // zenturmExtract_HPP
