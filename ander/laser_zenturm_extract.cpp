/*
    文件等级：密一
    author:klug
    献给不喜欢我的弗雷德里希冯海因洛特
    start:221230
    last:230329
*/

#include "laser_zenturm_extract.hpp"

using namespace cv;

laser_zenturm_extract::laser_zenturm_extract(int r,int g,int b)
{
#ifdef laser_zenturm_print_msg_info
    printf("start laser_zenturm_extract...\n");
#endif
    b_threshold=b;
    g_threshold=g;
    r_threshold=r;

}

laser_zenturm_extract::laser_zenturm_extract()
{
#ifdef laser_zenturm_print_msg_info
    printf("start laser_zenturm_extract...\n");
#endif

}

laser_zenturm_extract::~laser_zenturm_extract()
{
#ifdef laser_zenturm_print_msg_info
    printf("end laser_zenturm_extract...\n");
#endif

}

/*
    粗提取激光中心点，反光膜
    @img 包含激光中心点的图像
    @almost_roi 提取的ROI区域
    @almost_find_point 返回值，包含提取到可能为中心点的的坐标
*/
std::vector<cv::Point2f> laser_zenturm_extract::laser_zenturm_almost_find(cv::Mat img,cv::Mat &gray_img)
{
    cv::Mat ein_channel(img.rows,img.cols,CV_8UC1);
    int rgb_array[img.rows*img.cols];

    //根据颜色阈值生成单通道灰度图
    int pixel_cnt=0;
    int col_max=0,row_max=0;
    int col_min=11111,row_min=11111;
    for(int row=up_width;row<img.rows-down_width;row++)
    {
        for(int col=link_width;col<img.cols-richt_width;col++)
        {
            int gray=0;
            gray+=-1*img.at<Vec3b>(row,col)[0];//b
            gray+=-1*img.at<Vec3b>(row,col)[1];//g
            gray+=2*img.at<Vec3b>(row,col)[2];//r
            if(gray<0)
            {
                gray=0;
            }
            if(gray>255)
            {
                gray=255;
                pixel_cnt++;
            }
            ein_channel.at<uchar>(row,col)=gray;
        }
    }

#ifdef laser_zenturm_save
    cv::imwrite("almostfind_ein_channel.jpg",ein_channel);
#endif
    cv::Mat edge;
    cv::threshold(ein_channel,ein_channel,10,255,cv::THRESH_BINARY);

#ifdef laser_zenturm_save
    cv::imwrite("almostfind_bin.jpg",ein_channel);
#endif

    cv::Mat element;
    element = getStructuringElement(MORPH_RECT,Size(17,17));
    morphologyEx(ein_channel,ein_channel,MORPH_CLOSE,element);

#ifdef laser_zenturm_save
    cv::imwrite("almostfind_bin_close.jpg",ein_channel);
#endif

    Canny(ein_channel,edge,150,100,3);
    gray_img=edge;
#ifdef laser_zenturm_save
    cv::imwrite("almostfind_edge.jpg",edge);
#endif
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    findContours(edge, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE, Point(0, 0));
    std::vector<cv::Point2f> point_save;
    cv::RotatedRect box;

    for(size_t c_size=0;c_size<contours.size();c_size++)
    {
        box=minAreaRect(Mat(contours[c_size]));
        cv::Point2f p(box.center.x,box.center.y);
        point_save.push_back(p);
    }

#ifdef laser_zenturm_print_data_info
    std::cout<<"almost find point size is "<<point_save.size()<<std::endl;
#endif
    return point_save;
}

/*
    近距离激光中心点提取算法，反光膜
    @img 提取中心点的图片
    @laser_center 中心点坐标
    @centor_mark 返回值，标记中心点的图片
*/
cv::Mat laser_zenturm_extract::laser_zenturm_nach(cv::Mat img,cv::Point2f &laser_center)
{
    std::vector<cv::Point2f> laser_zenturm_find;
    cv::Mat bin_img;
    cv::Mat centor_mark=img;
    laser_zenturm_find=laser_zenturm_almost_find(img,bin_img);

    cv::Mat roi,roi_;
    int x_decline;
    int y_decline;

    for(size_t roi_cnt=0;roi_cnt<laser_zenturm_find.size();roi_cnt++)
    {
        x_decline=0;
        y_decline=0;

        int p_x=laser_zenturm_find[roi_cnt].x-almost_roi/2;
        int p_y=laser_zenturm_find[roi_cnt].y-almost_roi/2;
        if(p_x<0)
        {
            p_x=0;
        }
        if(p_y<0)
        {
            p_y=0;
        }

        if(p_x+almost_roi>img.cols)
        {
            x_decline=p_x+almost_roi-img.cols;
        }
        if(p_y+almost_roi>img.rows)
        {
            y_decline=p_y+almost_roi-img.rows;
        }

        cv::Point2f p(p_x,p_y);
        roi=bin_img(cv::Rect(p.x,p.y,almost_roi-x_decline,almost_roi-y_decline));
        roi_=img(cv::Rect(p.x,p.y,almost_roi-x_decline,almost_roi-y_decline));
#ifdef laser_zenturm_save
        imwrite("almost_roi_bin.png",roi);
#endif

        std::vector<std::vector<Point>> contours;
        std::vector<Vec4i> hierarchy;
        findContours(roi,contours,hierarchy,RETR_TREE,CHAIN_APPROX_SIMPLE,Point());

        float u_sum=0;
        float v_sum=0;

        for(size_t i=0;i<contours.size();i++)
        {
            int v_min=10000;
            int v_max=0;
            int u_min=10000;
            int u_max=0;
            int pixel_cnt=0;
            std::vector<cv::Point> con=contours[i];
            for(size_t j=0;j<con.size();j++)
            {
                if(con[j].x<u_min)
                {
                    u_min=con[j].x;
                }
                else if(con[j].x>u_max)
                {
                    u_max=con[j].x;
                }

                if(con[j].y<v_min)
                {
                    v_min=con[j].y;
                }
                else if(con[j].y>v_max)
                {
                    v_max=con[j].y;
                }
            }

            for(int row=v_min;row<v_max;row++)
            {
                for(int col=u_min;col<u_max;col++)
                {
                    int blue=roi_.at<Vec3b>(row,col)[0];
                    int green=roi_.at<Vec3b>(row,col)[1];
                    int red=roi_.at<Vec3b>(row,col)[2];
                    if(blue>b_threshold&&green>g_threshold&&red>r_threshold)
                    {
                        pixel_cnt++;
                        u_sum+=row;
                        v_sum+=col;
                    }
                }
            }

            if(pixel_cnt<50)
            {
                continue;
            }
            else
            {
#ifdef laser_zenturm_print_data_info
                printf("pixel_cnt %d\n",pixel_cnt);
#endif
            }

            v_sum/=pixel_cnt;
            u_sum/=pixel_cnt;

            circle(centor_mark,cv::Point(p_x+v_sum+1,p_y+u_sum+1),10,Scalar(0,255,0),1,5,0);
            laser_center.x=p_x+v_sum+1;
            laser_center.y=p_y+u_sum+1;
        }
    }

    return centor_mark;
}

/*
    远距离激光中心点提取算法，反光膜
    @img 提取中心点的图片
    @laser_center 中心点坐标
    @centor_mark 返回值，标记中心点的图片
*/
cv::Mat laser_zenturm_extract::laser_zenturm_fern(cv::Mat img,cv::Point2f &laser_center)
{
    std::vector<cv::Point2f> laser_zenturm_find;
    cv::Mat bin_img;
    cv::Mat centor_mark=img;
    laser_zenturm_find=laser_zenturm_almost_find(img,bin_img);

    cv::Mat roi,roi_;
    int x_decline;
    int y_decline;

    for(size_t roi_cnt=0;roi_cnt<laser_zenturm_find.size();roi_cnt++)
    {
        x_decline=0;
        y_decline=0;

        int p_x=laser_zenturm_find[roi_cnt].x-almost_roi/2;
        int p_y=laser_zenturm_find[roi_cnt].y-almost_roi/2;
        if(p_x<0)
        {
            p_x=0;
        }
        if(p_y<0)
        {
            p_y=0;
        }

        if(p_x+almost_roi>img.cols)
        {
            x_decline=p_x+almost_roi-img.cols;
        }
        if(p_y+almost_roi>img.rows)
        {
            y_decline=p_y+almost_roi-img.rows;
        }

        cv::Point2f p(p_x,p_y);
        roi=bin_img(cv::Rect(p.x,p.y,almost_roi-x_decline,almost_roi-y_decline));
        roi_=img(cv::Rect(p.x,p.y,almost_roi-x_decline,almost_roi-y_decline));
#ifdef laser_zenturm_save
        imwrite("almost_roi_bin.png",roi);
#endif

        std::vector<std::vector<Point>> contours;
        std::vector<Vec4i> hierarchy;
        findContours(roi,contours,hierarchy,RETR_TREE,CHAIN_APPROX_SIMPLE,Point());

        float u_sum=0;
        float v_sum=0;

        for(size_t i=0;i<contours.size();i++)
        {
            int v_min=10000;
            int v_max=0;
            int u_min=10000;
            int u_max=0;
            int pixel_cnt=0;
            std::vector<cv::Point> con=contours[i];
            for(size_t j=0;j<con.size();j++)
            {
                if(con[j].x<u_min)
                {
                    u_min=con[j].x;
                }
                else if(con[j].x>u_max)
                {
                    u_max=con[j].x;
                }

                if(con[j].y<v_min)
                {
                    v_min=con[j].y;
                }
                else if(con[j].y>v_max)
                {
                    v_max=con[j].y;
                }
            }

            for(int row=v_min;row<v_max;row++)
            {
                for(int col=u_min;col<u_max;col++)
                {
                    int blue=roi_.at<Vec3b>(row,col)[0];
                    int green=roi_.at<Vec3b>(row,col)[1];
                    int red=roi_.at<Vec3b>(row,col)[2];
                    if(blue>b_threshold&&green>g_threshold&&red>r_threshold)
                    {
                        pixel_cnt++;
                        u_sum+=row;
                        v_sum+=col;
                    }
                }
            }

            if(pixel_cnt<10)
            {
                continue;
            }
            else
            {
#ifdef laser_zenturm_print_data_info
                printf("pixel_cnt %d\n",pixel_cnt);
#endif
            }

            v_sum/=pixel_cnt;
            u_sum/=pixel_cnt;

            circle(centor_mark,cv::Point(p_x+v_sum+1,p_y+u_sum+1),10,Scalar(0,255,0),1,5,0);
            laser_center.x=p_x+v_sum+1;
            laser_center.y=p_y+u_sum+1;
        }
    }

    return centor_mark;
}

/*
    判断距离远近
    @img:输入图像
    @judge:返回值，判断结果，0 表示远距离，1 表示近距离
*/
bool laser_zenturm_extract::judge_length(cv::Mat img)
{
    bool judge;



    return judge;
}
