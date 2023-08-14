/*
    文件等级：密一
    author:klug
    献给我亲爱的师弟头盔闪亮的赫克托尔
    start:230724
    last:230814
*/

#ifndef zenturmExtract_HPP
#define zenturmExtract_HPP

#include <opencv2/opencv.hpp>

#define zenturmExtractSave
#define zenturmExtractPrintData
#define zenturmExtractPrintMsg
#define zenturmExtractPrintError

void zenturmExtract(cv::Mat srcImg,cv::Point2f &zenturm);

#endif // zenturmExtract_HPP
