/*
    文件等级：密一
    author:klug
    献给我亲爱的好友尼姆教授
    start:230724
    last:230818
*/

#include "img_process/zenturmExtract.hpp"

void zenturmExtractCal(cv::Mat srcImg,cv::Point2f &zenturm)
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
    cv::imwrite("/home/klug/img/lengthMeasure/res/grayImg.png",grayImg);
#endif

    cv::Mat blurImg;
    cv::medianBlur(grayImg,blurImg,3);
#ifdef zenturmExtractSave
    cv::imwrite("/home/klug/img/lengthMeasure/res/blurImg.png",blurImg);
#endif

    cv::Mat binImg;
    cv::threshold(blurImg,binImg,100,250,cv::THRESH_BINARY);
#ifdef zenturmExtractSave
    cv::imwrite("/home/klug/img/lengthMeasure/res/binImg.png",binImg);
#endif

    //形态学
    cv::Mat morImg;
    cv::Mat kernel=getStructuringElement(cv::MORPH_RECT,cv::Size(9,9),cv::Point(-1,-1));
    morphologyEx(binImg,morImg,CV_MOP_OPEN,kernel);
#ifdef zenturmExtractSave
    cv::imwrite("/home/klug/img/lengthMeasure/res/morImg.png",morImg);
#endif

    //连通域roi提取
    cv::Mat labels,lab;
    cv::Mat connect_info,connect_centor;
    int line_cnt=cv::connectedComponentsWithStats(morImg,lab,connect_info,connect_centor,8,CV_16U);
#ifdef zenturmExtractPrintData
    std::cout<<"connected cnt:="<<line_cnt<<std::endl;
#endif

    std::vector<cv::Point2f> zenturmSet;
    for(int i=0;i<line_cnt;i++)
    {
        int x=connect_info.at<int>(i,0);
        int y=connect_info.at<int>(i,1);
        int h=connect_info.at<int>(i,2);
        int w=connect_info.at<int>(i,3);

        //连通域为最大区域不进行处理
        if(h==morImg.cols&&w==morImg.rows)
        {
            continue;
        }

        cv::Mat roi_img=morImg(cv::Rect(x,y,h,w));
        cv::Point2f z;
        zenturmCalculateGray(roi_img,z);
        z.x+=x;
        z.y+=y;
        zenturmSet.push_back(z);
    }

    if(zenturmSet.size()<2)
    {
        zenturm=zenturmSet[0];
    }
    else
    {
#ifdef zenturmExtractPrintError
        printf("ERROR:zenturm extract failed...\n");
#endif
    }

#ifdef zenturmExtractMark
    cv::Mat mark;
    srcImg.copyTo(mark);
    cv::circle(mark,zenturm,10,cv::Scalar(0,255,0));
    cv::imwrite("/home/klug/img/lengthMeasure/res/markImg.png",mark);
#endif
}

void zenturmExtract(cv::Mat srcImg,cv::Point2f &zenturm)
{
    cv::Mat grayImg(roiRow,roiCol,CV_8UC1);
    for(size_t row=0;row<roiRow;row++)
    {
        for(size_t col=0;col<roiCol;col++)
        {
            grayImg.at<uchar>(row,col)=srcImg.at<cv::Vec3b>(row+offsetY,col+offsetX)[2];
        }
    }
#ifdef zenturmExtractSave
    cv::imwrite("/home/klug/img/lengthMeasure/res/grayImg.png",grayImg);
#endif

    cv::Mat blurImg;
    cv::medianBlur(grayImg,blurImg,3);
#ifdef zenturmExtractSave
    cv::imwrite("/home/klug/img/lengthMeasure/res/blurImg.png",blurImg);
#endif

    cv::Mat binImg;
    cv::threshold(blurImg,binImg,0,100,cv::THRESH_OTSU);
    //cv::threshold(blurImg,binImg,150,250,cv::THRESH_BINARY);
#ifdef zenturmExtractSave
    cv::imwrite("/home/klug/img/lengthMeasure/res/binImg.png",binImg);
#endif

    //形态学
    cv::Mat morImg;
    cv::Mat kernel=getStructuringElement(cv::MORPH_RECT,cv::Size(9,9),cv::Point(-1,-1));
    morphologyEx(binImg,morImg,CV_MOP_OPEN,kernel);
#ifdef zenturmExtractSave
    cv::imwrite("/home/klug/img/lengthMeasure/res/morImg.png",morImg);
#endif

    //连通域roi提取
    cv::Mat labels,lab;
    cv::Mat connect_info,connect_centor;
    int line_cnt=cv::connectedComponentsWithStats(morImg,lab,connect_info,connect_centor,8,CV_16U);
#ifdef zenturmExtractPrintData
    std::cout<<"connected cnt:="<<line_cnt<<std::endl;
#endif

    std::vector<cv::Point2f> zenturmSet;
    for(int i=0;i<line_cnt;i++)
    {
        int x=connect_info.at<int>(i,0);
        int y=connect_info.at<int>(i,1);
        int h=connect_info.at<int>(i,2);
        int w=connect_info.at<int>(i,3);

        //连通域为最大区域不进行处理
        if(h==morImg.cols&&w==morImg.rows)
        {
            continue;
        }

        cv::Mat roi_img=morImg(cv::Rect(x,y,h,w));
        cv::Point2f z;
        zenturmCalculateGray(roi_img,z);
        z.x+=x;
        z.x+=offsetX;
        z.y+=y;
        z.y+=offsetY;
        zenturmSet.push_back(z);
    }

    if(zenturmSet.size()<2)
    {
        zenturm=zenturmSet[0];
    }
    else if(zenturmSet.size()==2)
    {
        double y1=zenturmSet[0].y;
        double y2=zenturmSet[1].y;
        if(y1>y2)
        {
            zenturm=zenturmSet[0];
        }
        else
        {
            zenturm=zenturmSet[1];
        }
    }
    else
    {
#ifdef zenturmExtractPrintError
        printf("ERROR:zenturm extract failed...\n");
#endif
    }

#ifdef zenturmExtractMark
    cv::Mat mark;
    srcImg.copyTo(mark);
    cv::circle(mark,zenturm,1,cv::Scalar(0,255,0));
    cv::imwrite("/home/klug/img/lengthMeasure/res/markImg.png",mark);
#endif
}

/*
    中心点坐标计算
    @srcImg:中线点闭包
    @zenturm:中心点坐标
*/
void zenturmCalculate(cv::Mat srcImg,cv::Point2f &zenturm)
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
    cv::imwrite("/home/klug/img/lengthMeasure/res/grayImg.png",grayImg);
#endif

    cv::Mat blurImg;
    cv::medianBlur(grayImg,blurImg,3);
#ifdef zenturmExtractSave
    cv::imwrite("/home/klug/img/lengthMeasure/res/blurImg.png",blurImg);
#endif

    cv::Mat binImg;
    cv::threshold(blurImg,binImg,100,250,cv::THRESH_BINARY);
#ifdef zenturmExtractSave
    cv::imwrite("/home/klug/img/lengthMeasure/res/binImg.png",binImg);
#endif

    //形态学
    cv::Mat morImg;
    cv::Mat kernel=getStructuringElement(cv::MORPH_RECT,cv::Size(9,9),cv::Point(-1,-1));
    morphologyEx(binImg,morImg,CV_MOP_OPEN,kernel);
#ifdef zenturmExtractSave
    cv::imwrite("/home/klug/img/lengthMeasure/res/morImg.png",morImg);
#endif

    //连通域roi提取
    cv::Mat labels,lab;
    cv::Mat connect_info,connect_centor;
    int line_cnt=cv::connectedComponentsWithStats(morImg,lab,connect_info,connect_centor,8,CV_16U);
#ifdef zenturmExtractPrintData
    std::cout<<"connected cnt:="<<line_cnt<<std::endl;
#endif

    std::vector<cv::Point2f> zenturmSet;
    for(int i=0;i<line_cnt;i++)
    {
        int x=connect_info.at<int>(i,0);
        int y=connect_info.at<int>(i,1);
        int h=connect_info.at<int>(i,2);
        int w=connect_info.at<int>(i,3);

        //连通域为最大区域不进行处理
        if(h==morImg.cols&&w==morImg.rows)
        {
            continue;
        }

        cv::Mat roi_img=morImg(cv::Rect(x,y,h,w));
        cv::Point2f z;
        zenturmCalculate(roi_img,z);
        z.x+=x;
        z.y+=y;
        zenturmSet.push_back(z);
    }

    if(zenturmSet.size()<2)
    {
        zenturm=zenturmSet[0];
    }
    else
    {
#ifdef zenturmExtractPrintError
        printf("ERROR:zenturm extract failed...\n");
#endif
    }

#ifdef zenturmExtractMark
    cv::Mat mark;
    srcImg.copyTo(mark);
    cv::circle(mark,zenturm,10,cv::Scalar(0,255,0));
    cv::imwrite("/home/klug/img/lengthMeasure/res/markImg.png",mark);
#endif
}

void zenturmExtract_(cv::Mat srcImg,cv::Point2f &zenturm)
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
    cv::imwrite("/home/klug/img/lengthMeasure/res/grayImg.png",grayImg);
#endif

    cv::Mat blurImg;
    cv::medianBlur(grayImg,blurImg,3);
#ifdef zenturmExtractSave
    cv::imwrite("/home/klug/img/lengthMeasure/res/blurImg.png",blurImg);
#endif

    cv::Mat binImg;
    cv::threshold(blurImg,binImg,0,100,cv::THRESH_OTSU);
    //cv::threshold(blurImg,binImg,150,250,cv::THRESH_BINARY);
#ifdef zenturmExtractSave
    cv::imwrite("/home/klug/img/lengthMeasure/res/binImg.png",binImg);
#endif

    //形态学
    cv::Mat morImg;
    cv::Mat kernel=getStructuringElement(cv::MORPH_RECT,cv::Size(9,9),cv::Point(-1,-1));
    morphologyEx(binImg,morImg,CV_MOP_OPEN,kernel);
#ifdef zenturmExtractSave
    cv::imwrite("/home/klug/img/lengthMeasure/res/morImg.png",morImg);
#endif

    //连通域roi提取
    cv::Mat labels,lab;
    cv::Mat connect_info,connect_centor;
    int line_cnt=cv::connectedComponentsWithStats(morImg,lab,connect_info,connect_centor,8,CV_16U);
#ifdef zenturmExtractPrintData
    std::cout<<"connected cnt:="<<line_cnt<<std::endl;
#endif

    std::vector<cv::Point2f> zenturmSet;
    for(int i=0;i<line_cnt;i++)
    {
        int x=connect_info.at<int>(i,0);
        int y=connect_info.at<int>(i,1);
        int h=connect_info.at<int>(i,2);
        int w=connect_info.at<int>(i,3);

        //连通域为最大区域不进行处理
        if(h==morImg.cols&&w==morImg.rows)
        {
            continue;
        }

        cv::Mat roi_img=morImg(cv::Rect(x,y,h,w));
        cv::Point2f z;
        zenturmCalculate(roi_img,z);
        z.x+=x;
        z.y+=y;
        zenturmSet.push_back(z);
    }

    if(zenturmSet.size()<2)
    {
        zenturm=zenturmSet[0];
    }
    else if(zenturmSet.size()==2)
    {
        double y1=zenturmSet[0].y;
        double y2=zenturmSet[1].y;
        if(y1>y2)
        {
            zenturm=zenturmSet[0];
        }
        else
        {
            zenturm=zenturmSet[1];
        }
    }
    else
    {
#ifdef zenturmExtractPrintError
        printf("ERROR:zenturm extract failed...\n");
#endif
    }

#ifdef zenturmExtractMark
    cv::Mat mark;
    srcImg.copyTo(mark);
    cv::circle(mark,zenturm,1,cv::Scalar(0,255,0));
    cv::imwrite("/home/klug/img/lengthMeasure/res/markImg.png",mark);
#endif
}

/*
    灰度质心法计算中心点坐标计算
    @srcImg:中线点闭包
    @zenturm:中心点坐标
*/
void zenturmCalculateGray(cv::Mat srcImg,cv::Point2f &zenturm)
{
    double x=0,y=0;
    int pCnt=0;

    for(size_t row=0;row<srcImg.rows;row++)
    {
        for(size_t col=0;col<srcImg.cols;col++)
        {
            int gray=srcImg.at<uchar>(row,col);
            if(gray>0)
            {
                x+=col;
                y+=row;
                pCnt++;
            }
        }
    }

    zenturm.x=x/pCnt;
    zenturm.y=y/pCnt;
}

/*
    边缘质心法计算中心点坐标计算
    @srcImg:中线点闭包
    @zenturm:中心点坐标
*/
void zenturmCalculateContour(cv::Mat srcImg,cv::Point2f &zenturm)
{

}
