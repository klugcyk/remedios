/*
    文件等级：密一
    author:klug
    献给我亲爱的好友尼姆教授
    start:230724
    last:230814
*/

#include "img_process/zenturmExtract.hpp"

void zenturmExtract(cv::Mat srcImg,cv::Point2f &zenturm)
{
    cv::Mat grayImg(srcImg.size(),CV_8UC1);
    for(size_t row=0;row<srcImg.rows;row++)
    {
        for(size_t col=0;col<srcImg.cols;col++)
        {
            grayImg.at<uchar>(row,col)=srcImg.at<cv::Vec3b>(row,col)[2];
        }
    }
#ifdef zenturmExtractSave
    cv::imwrite("/home/klug/img/lengthMeasure/grayImg.png",grayImg);
#endif

    cv::Mat blurImg;
    cv::medianBlur(grayImg,blurImg,3);
#ifdef zenturmExtractSave
    cv::imwrite("/home/klug/img/lengthMeasure/blurImg.png",blurImg);
#endif



    zenturm.x=0;
    zenturm.y=0;
}
