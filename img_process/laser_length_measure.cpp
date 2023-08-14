/*
    文件等级：密一
    author:klug
    献给我亲爱的好友尼姆教授
    start:230222
    last:230605
*/

#include "img_process/laser_length_measure.hpp"
#include "galvo/galvo.hpp"

laser_length_measure::laser_length_measure()
{
#ifdef laser_length_measure_print_msg_info
    printf("start the laser length measure node...\n");
#endif

}

laser_length_measure::~laser_length_measure()
{
#ifdef laser_length_measure_print_msg_info
    printf("end the laser length measure node...\n");
#endif

}

/*
    相机标定程序，用于三维重建和激光点测距,使用类中默认的参数
    @img_vector:图像容器,30张,前20张用于相机标定，后10张用于激光线方程计算，激光点测距时
    @img_vector:图像容器,20张,三维重建相机标定时
    @transform_matrix:返回值，外参矩阵
*/
std::vector<cv::Mat> laser_length_measure::camera_calibrate(std::vector<cv::Mat> img_vector)
{
#ifdef laser_length_measure_print_msg_info
    printf("start the calibrate from img_array...\n");
#endif

    int image_count=0;
    cv::Size image_size;

    std::vector<cv::Point2f> image_points;
    std::vector<std::vector<cv::Point2f>> image_points_seq;

    for(size_t i=0;i<img_vector.size();i++)
    {
        image_count++;
#ifdef laser_length_measure_print_msg_info
        std::cout << "image_count=" << image_count << std::endl;
#endif
        cv::Mat imageInput = img_vector[i];
        if(image_count==1)
        {
            image_size.width = imageInput.cols;
            image_size.height = imageInput.rows;
        }

        bool bRes=findChessboardCorners(imageInput,board_size,image_points,0);

        if(bRes)
        {
            cv::Mat view_gray;
            cvtColor(imageInput, view_gray, cv::COLOR_RGB2GRAY);
            cv::cornerSubPix(view_gray, image_points, cv::Size(11, 11), cv::Size(-1, -1),cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.01));
            image_points_seq.push_back(image_points);
            drawChessboardCorners(view_gray, board_size, image_points, true);
        }
        else
        {
#ifdef laser_length_measure_print_error_info
            printf("img fail...\n");
#endif
        }
    }

    std::vector<std::vector<cv::Point3f>> object_points_seq;

    for(int t=0;t<image_count;t++)
    {
        std::vector<cv::Point3f> object_points;
        for (int i = 0; i < board_size.height; i++)
        {
            for (int j = 0; j < board_size.width; j++)
            {
                cv::Point3f realPoint;
                realPoint.x = i * square_size.width;
                realPoint.y = j * square_size.height;
                realPoint.z = 0;
                object_points.push_back(realPoint);
            }
        }
        object_points_seq.push_back(object_points);
    }

    double rms;
    if(object_points_seq.size()==image_points_seq.size())
    {
        rms=calibrateCamera(object_points_seq,image_points_seq,image_size,cameraMatrix,distCoeffs,rvecsMat,tvecsMat, cv::CALIB_FIX_K3+cv::CALIB_ZERO_TANGENT_DIST);
    }

#ifdef laser_length_measure_print_data_info
    std::cout<<"cameraMatrix:="
            <<cameraMatrix
           <<std::endl;
#endif

#ifdef laser_length_measure_print_data_info
    std::cout << "RMS:" << rms << "pixel" << std::endl << std::endl;
    for(size_t i=0;i<tvecsMat.size();i++)
    {
        std::cout<<"transform "
              <<i
              <<" := "
              <<tvecsMat[i]
              <<std::endl;
    }
#endif
    rotation_matrix.clear();
    for(size_t i=0;i<rvecsMat.size();i++)
    {
#ifdef laser_length_measure_print_data_info
        std::cout<<"rotate vector:="
              <<i
              <<" :="
              <<rvecsMat[i]
              <<std::endl;
#endif
        cv::Mat rm = cv::Mat(3,3,CV_64FC1,cv::Scalar::all(0));
        Rodrigues(rvecsMat[i],rm);
        rotation_matrix.push_back(rm);
#ifdef laser_length_measure_print_data_info
        std::cout<<"rotate matriz "
              <<i
              <<" :="
              <<rm
              <<std::endl;
#endif
    }

    extrinsic_matrix.clear();
    for(size_t i=0;i<rotation_matrix.size();i++)
    {
        cv::Mat tm=cv::Mat(4,4,CV_64FC1,cv::Scalar::all(0));
        for(int c=0;c<3;c++)
        {
            for(int r=0;r<3;r++)
            {
                tm.at<double>(c,r)=rotation_matrix[i].at<double>(c,r);
            }
        }
        tm.at<double>(0,3)=tvecsMat[i].at<double>(0,0);
        tm.at<double>(1,3)=tvecsMat[i].at<double>(0,1);
        tm.at<double>(2,3)=tvecsMat[i].at<double>(0,2);
        tm.at<double>(3,3)=1;
        extrinsic_matrix.push_back(tm);
#ifdef laser_length_measure_print_data_info
        std::cout<<"extrinsic matrix "
                <<i
                <<":="
                <<std::endl
                <<tm
                <<std::endl;
#endif
    }
#ifdef laser_length_measure_print_msg_info
    printf("calibrate done from img array...\n");
#endif
    return extrinsic_matrix;
}

/*
    标定激光测距系统，振镜标定
    @cal_img:输入图片1-20张用于标定相机内参，后21-30张用于标定距离,31，32用于相机振镜参数偏差标定
    @返回值，标定计算结果状态
*/
int laser_length_measure::system_calibrate(std::vector<cv::Mat> cal_img)
{
    //标定相机
    if(cal_img.size()<=25)
    {
#ifdef laser_length_measure_print_error_info
        printf("the img is not enough!!!\n");
#endif
        return -1;
    }

    std::vector<cv::Mat> extrinsics=camera_calibrate(cal_img);
#ifdef laser_length_measure_print_data_info
    for(int i=20;i<extrinsics.size();i++)
    {
        printf("z(%d):=%f\n",i,extrinsics[i].at<double>(2,3));
    }
#endif
    double cal_u=cameraMatrix.at<double>(0,2);
    double cal_v=cameraMatrix.at<double>(1,2);
    double cal_fx=cameraMatrix.at<double>(0,0);
    double cal_fy=cameraMatrix.at<double>(1,1);

#ifdef laser_length_measure_print_data_info
    printf("img_size.col:=%d\n",cal_img[1].cols);
    printf("img_size.row:=%d\n",cal_img[1].rows);
    printf("cal_u:=%f\n",cal_u);
    printf("cal_v:=%f\n",cal_v);
    printf("cal_fx:=%f\n",cal_fx);
    printf("cal_fy:=%f\n",cal_fy);
#endif

    // 读取包含激光中心点的图像
    std::vector<cv::Mat> laser_point_img;
    for(int i=101;i<=100+cal_img.size()-18;i++)
    {
        std::string path="/home/klug/img/length_measure/";
        path+=std::to_string(i);
        path+=".png";
        cv::Mat _img=imread(path);
        cv::Mat undistort_img;
        cv::undistort(_img,undistort_img,cameraMatrix,distCoeffs);
#ifdef laser_length_measure_save_process
        std::string path_="undistort"+std::to_string(i);
        path_+=".png";
        cv::imwrite(path_,undistort_img);
#endif
        if(!_img.empty())
        {
            laser_point_img.push_back(_img);
        }
        else
        {
            std::cout<<"img "
                     << i
                     <<".png not find"
                     <<std::endl;
        }
    }

    if(laser_point_img.size()!=12)
    {
        return -2;
    }

    double xyz_array[10][3]; //保存距离信息
    //cal_img中第20-30张图片用于距离标定
    for(size_t i=20;i<30;i++)
    {
        cv::Point2f p;
        laser_zenturm_caltest(laser_point_img[i-20],p);//计算激光中心点
#ifdef laser_length_measure_save_process
        std::string path="zenturm_";
        path+=std::to_string(i-20);
        path+=".png";
        cv::imwrite(path,laser_point_img[i-20]);
#endif
        xyz_array[i-20][0]=(p.x-cal_u)/cal_fx*extrinsics[i].at<double>(2,3); // x
        xyz_array[i-20][1]=(p.y-cal_v)/cal_fy*extrinsics[i].at<double>(2,3); // y
        xyz_array[i-20][2]=extrinsics[i].at<double>(2,3); // 从外参中读取标定板到相机坐标系的距离Z
    }

    // 计算激光线在相机坐标系下的方程
    calculate_line(xyz_array);
#ifdef laser_length_measure_print_data_info
    printf("line in camera coordinate a %f\n",laser_line.a);
    printf("line in camera coordinate b %f\n",laser_line.b);
    printf("line in camera coordinate c %f\n",laser_line.c);
    printf("line in camera coordinate x %f\n",laser_line.x0);
    printf("line in camera coordinate y %f\n",laser_line.y0);
    printf("line in camera coordinate z %f\n",laser_line.y0);
#endif
    // 标定振镜,计算角度偏差
    galvo_calibrate(laser_line);

    // 标定振镜坐标系和相机坐标系之间的偏差
    std::vector<cv::Point3f> galvo_camera_point;
    std::vector<mathGeometry::geoLineParam> galvo_camera_line;

    //标定振镜和相机偏差计算点坐标
    cv::Point2f p2;
    cv::Point3f p3;
    laser_zenturm_caltest(laser_point_img[10],p2);//计算激光中心点
#ifdef laser_length_measure_save_process
    cv::imwrite("zenturm_11.png",laser_point_img[10]);
#endif
    p3.x=(p2.x-cal_u)/cal_fx*extrinsics[29].at<double>(2,3); //根据最后一张标定板计算x u*z=fx*x+u0*z
    p3.y=(p2.y-cal_v)/cal_fy*extrinsics[29].at<double>(2,3); //根据最后一张标定板计算y
    p3.z=extrinsics[29].at<double>(2,3); //从外参中读取标定板到相机坐标系的距离Z
    galvo_camera_point.push_back(p3);

    laser_zenturm_caltest(laser_point_img[11],p2);
#ifdef laser_length_measure_save_process
    cv::imwrite("zenturm_12.png",laser_point_img[11]);
#endif
    p3.x=(p2.x-cal_u)/cal_fx*extrinsics[29].at<double>(2,3);
    p3.y=(p2.y-cal_v)/cal_fy*extrinsics[29].at<double>(2,3);
    p3.z=extrinsics[29].at<double>(2,3);
    galvo_camera_point.push_back(p3);

    //标定振镜和相机偏差计算出射激光线方程
    mathGeometry::geoLineParam l1=laser_line_shoot_galvo(-1/57.32,1/57.32);
    galvo_camera_line.push_back(l1);
    l1=laser_line_shoot_galvo(-1/57.32,0);
    galvo_camera_line.push_back(l1);

    galvo2camera(galvo_camera_point,galvo_camera_line);

/*
    cv::Point2f p2;
    cv::Point3f p3;
    laser_zenturm_caltest(laser_point_img[10],p2);//计算激光中心点
    p3.x=(p2.x-cal_u)/cal_fx*extrinsics[29].at<double>(2,3); //根据最后一张标定板计算x u*z=fx*x+u0*z
    p3.y=(p2.y-cal_v)/cal_fy*extrinsics[29].at<double>(2,3); //根据最后一张标定板计算y
    p3.z=extrinsics[29].at<double>(2,3); //从外参中读取标定板到相机坐标系的距离Z
    geo_line_param l1=laser_line_shoot_galvo(-1/57.32,1/57.32);

    double fx=cameraMatrix.at<double>(0,0);
    double fy=cameraMatrix.at<double>(1,1);
    double u0=cameraMatrix.at<double>(0,2);
    double v0=cameraMatrix.at<double>(1,2);

    galvo2camera(p3,l1,u0,v0,fx,fy);*/
#ifdef laser_length_measure_print_msg_info
    printf("Achieve the system calibrate...\n");
#endif

    return 1;
}

/*
    标定激光测距系统，振镜标定
    @cal_img:输入图片1-20张用于标定相机内参，后21-30张用于标定距离,31，32用于相机振镜参数偏差标定
    @laser_point_img:1-10用于计算振镜安装误差，11，12用于标定相机和振镜坐标系之间的距离
    @返回值，标定计算结果状态
*/
int laser_length_measure::system_calibrate(std::vector<cv::Mat> cal_img,std::vector<cv::Mat> laser_point_img)
{
    //标定相机
    if(cal_img.size()<=calibrate_img_required)
    {
#ifdef laser_length_measure_print_error_info
        printf("the img is not enough!!!\n");
#endif
        return -1;
    }

    std::vector<cv::Mat> extrinsics=camera_calibrate(cal_img);
#ifdef laser_length_measure_print_data_info
    for(int i=20;i<extrinsics.size();i++)
    {
        printf("z(%d):=%f\n",i,extrinsics[i].at<double>(2,3));
    }
#endif
    double cal_u=cameraMatrix.at<double>(0,2);
    double cal_v=cameraMatrix.at<double>(1,2);
    double cal_fx=cameraMatrix.at<double>(0,0);
    double cal_fy=cameraMatrix.at<double>(1,1);

#ifdef laser_length_measure_print_data_info
    printf("img_size.col:=%d\n",cal_img[1].cols);
    printf("img_size.row:=%d\n",cal_img[1].rows);
    printf("cal_u:=%f\n",cal_u);
    printf("cal_v:=%f\n",cal_v);
    printf("cal_fx:=%f\n",cal_fx);
    printf("cal_fy:=%f\n",cal_fy);
#endif

    if(laser_point_img.size()!=12)
    {
        return -2;
    }

    double xyz_array[10][3]; //保存距离信息
    //cal_img中第20-30张图片用于距离标定
    for(size_t i=20;i<30;i++)
    {
        cv::Point2f p;
        laser_zenturm_caltest(laser_point_img[i-20],p);//计算激光中心点
#ifdef laser_length_measure_save_process
        std::string path="/home/klug/img/lengthMeasure/res/zenturm_";
        path+=std::to_string(i-20);
        path+=".png";
        cv::imwrite(path,laser_point_img[i-20]);
#endif
        xyz_array[i-20][0]=(p.x-cal_u)/cal_fx*extrinsics[i].at<double>(2,3); // x
        xyz_array[i-20][1]=(p.y-cal_v)/cal_fy*extrinsics[i].at<double>(2,3); // y
        xyz_array[i-20][2]=extrinsics[i].at<double>(2,3); // 从外参中读取标定板到相机坐标系的距离Z
    }

    // 计算激光线在相机坐标系下的方程
    calculate_line(xyz_array);
#ifdef laser_length_measure_print_data_info
    printf("line in camera coordinate a %f\n",laser_line.a);
    printf("line in camera coordinate b %f\n",laser_line.b);
    printf("line in camera coordinate c %f\n",laser_line.c);
    printf("line in camera coordinate x %f\n",laser_line.x0);
    printf("line in camera coordinate y %f\n",laser_line.y0);
    printf("line in camera coordinate z %f\n",laser_line.y0);
#endif
    // 标定振镜,计算角度偏差
    galvo_calibrate(laser_line);

    // 标定振镜坐标系和相机坐标系之间的偏差
    std::vector<cv::Point3f> galvo_camera_point;
    std::vector<mathGeometry::geoLineParam> galvo_camera_line;

    //标定振镜和相机偏差计算点坐标
    cv::Point2f p2;
    cv::Point3f p3;
    laser_zenturm_caltest(laser_point_img[10],p2);//计算激光中心点
#ifdef laser_length_measure_save_process
    cv::imwrite("/home/klug/img/lengthMeasure/res/zenturm_11.png",laser_point_img[10]);
#endif
    p3.x=(p2.x-cal_u)/cal_fx*extrinsics[29].at<double>(2,3); //根据最后一张标定板计算x u*z=fx*x+u0*z
    p3.y=(p2.y-cal_v)/cal_fy*extrinsics[29].at<double>(2,3); //根据最后一张标定板计算y
    p3.z=extrinsics[29].at<double>(2,3); //从外参中读取标定板到相机坐标系的距离Z
    galvo_camera_point.push_back(p3);

    laser_zenturm_caltest(laser_point_img[11],p2);
#ifdef laser_length_measure_save_process
    cv::imwrite("/home/klug/img/lengthMeasure/res/zenturm_12.png",laser_point_img[11]);
#endif
    p3.x=(p2.x-cal_u)/cal_fx*extrinsics[29].at<double>(2,3);
    p3.y=(p2.y-cal_v)/cal_fy*extrinsics[29].at<double>(2,3);
    p3.z=extrinsics[29].at<double>(2,3);
    galvo_camera_point.push_back(p3);

    //标定振镜和相机偏差计算出射激光线方程
    mathGeometry::geoLineParam l1=laser_line_shoot_galvo(-1/57.32,1/57.32);
    galvo_camera_line.push_back(l1);
    l1=laser_line_shoot_galvo(-1/57.32,0);
    galvo_camera_line.push_back(l1);

    galvo2camera(galvo_camera_point,galvo_camera_line);

#ifdef laser_length_measure_print_msg_info
    printf("Achieve the system calibrate...\n");
#endif

    return 1;
}

/*
    计算出射激光线方程，利用最小二乘法，计算出的结果为两个平面的交线
    @point_array:点坐标
*/
void laser_length_measure::calculate_line(double point_array[10][3])
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

    laser_line.k1=(n*sum_xz-sum_x*sum_z)/(n*sum_z2-sum_z*sum_z);
    laser_line.b1=(sum_x-laser_line.k1*sum_z)/n;
    laser_line.k2=(n*sum_yz-sum_y*sum_z)/(n*sum_z2-sum_z*sum_z);
    laser_line.b2=(sum_y-laser_line.k2*sum_z)/n;

    //vector ein
    double ax=1;
    double ay=0;
    double az=-laser_line.k1;

    //vector zwei
    double bx=0;
    double by=1;
    double bz=-laser_line.k2;

    laser_line.x0=-laser_line.b1;
    laser_line.y0=-laser_line.b2;
    laser_line.z0=0;
    laser_line.a=ay*bz-az*by;
    laser_line.b=az*bx-ax*bz;
    laser_line.c=ax*by-ay*bx;
#ifdef laser_length_measure_print_data_info
    printf("plane.x:=%f\n",laser_line.x0);
    printf("plane.y:=%f\n",laser_line.y0);
    printf("plane.z:=%f\n",laser_line.z0);
    printf("plane.a:=%f\n",laser_line.a);
    printf("plane.b:=%f\n",laser_line.b);
    printf("plane.c:=%f\n",laser_line.c);
#endif

#ifdef laser_length_measure_cheak
    double xx=-laser_line.a*laser_line.z0/laser_line.c+laser_line.x0;
    double yy=-laser_line.b*laser_line.z0/laser_line.c+laser_line.y0;
    printf("xx:=%f\n",xx);
    printf("yy:=%f\n",yy);
#endif
}

/*
    计算出射激光线方程，利用最小二乘法，计算出的结果为两个平面的交线
    @point_array:点坐标
*/
void laser_length_measure::calculate_line(double point_array[10][3],mathGeometry::geoLineParam &line)
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
    line.b1=(sum_x-laser_line.k1*sum_z)/n;
    line.k2=(n*sum_yz-sum_y*sum_z)/(n*sum_z2-sum_z*sum_z);
    line.b2=(sum_y-laser_line.k2*sum_z)/n;

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
#ifdef laser_length_measure_print_data_info
    printf("plane.x:=%f\n",line.x0);
    printf("plane.y:=%f\n",line.y0);
    printf("plane.z:=%f\n",line.z0);
    printf("plane.a:=%f\n",line.a);
    printf("plane.b:=%f\n",line.b);
    printf("plane.c:=%f\n",line.c);
#endif

#ifdef laser_length_measure_cheak
    double xx=-laser_line.a*laser_line.z0/laser_line.c+laser_line.x0;
    double yy=-laser_line.b*laser_line.z0/laser_line.c+laser_line.y0;
    printf("xx:=%f\n",xx);
    printf("yy:=%f\n",yy);
#endif
}

/*
    计算到基线的距离，计算方法：u=fx*x+u0*z，v=fy*y+v0*z,结合直线方程建立方程组求解
    @src_img:包含激光中心点的图像
    @length:z方向距离
*/
double laser_length_measure::length_measure_base(cv::Mat src_img)
{
    double length=0.0;

    cv::Point2f pz;
    laser_zenturm_caltest(src_img,pz);

    double u=pz.x;
    double v=pz.y;
    double fx=cameraMatrix.at<double>(0,0);
    double fy=cameraMatrix.at<double>(1,1);
    double u0=cameraMatrix.at<double>(0,2);
    double v0=cameraMatrix.at<double>(1,2);

    double temp1=laser_line.b*laser_line.x0-laser_line.a*laser_line.y0;
    double temp2=(u-u0)*laser_line.b/fx;
    double temp3=(v-v0)*laser_line.a/fy;

    length=temp1/(temp2-temp3);

    zenturm_coordinate.z=length;
    zenturm_coordinate.x=laser_line.a*(length-laser_line.z0)/laser_line.c+laser_line.x0;
    zenturm_coordinate.y=laser_line.b*(length-laser_line.z0)/laser_line.c+laser_line.y0;
#ifdef laser_length_measure_print_data_info
    printf("k1:=%f\n",laser_line.k1);
    printf("b1:=%f\n",laser_line.b1);
    printf("k2:=%f\n",laser_line.k2);
    printf("b2:=%f\n",laser_line.b2);

    printf("a:=%f\n",laser_line.a);
    printf("b:=%f\n",laser_line.b);
    printf("c:=%f\n",laser_line.c);
    printf("x0:=%f\n",laser_line.x0);
    printf("y0:=%f\n",laser_line.y0);
    printf("z0:=%f\n",laser_line.z0);
#endif
    return length;
}

/*
    激光中心点提取算法用于标定测试，不用于实际使用
    @src_img:包含激光中心点的图片
    @zenturm:提取的激光中心点
*/
void laser_length_measure::laser_zenturm_caltest(cv::Mat &src_img,cv::Point2f &zenturm)
{
    cv::Mat bin_img,edge_img;
    cv::cvtColor(src_img,src_img,cv::COLOR_BGR2GRAY);
    threshold(src_img,bin_img,150,255,cv::THRESH_BINARY);
#ifdef laser_length_measure_save_process
    imwrite("/home/klug/img/lengthMeasure/res/bin.png",bin_img);
#endif
    int u_sum=0;
    int v_sum=0;
    int pixel_cnt=0;

    for(size_t row=0;row<src_img.rows;row++)
    {
        for(size_t col=0;col<src_img.cols;col++)
        {
            int gray=bin_img.at<uchar>(row,col);
            if(gray!=0)
            {
                pixel_cnt++;
                u_sum+=row;
                v_sum+=col;
            }
        }
    }

    if(pixel_cnt!=0)
    {
        zenturm.x=v_sum/pixel_cnt;
        zenturm.y=u_sum/pixel_cnt;
        circle(src_img,zenturm,10,Scalar(255,255,255),1,5,0);
    }
    else
    {
        return;
    }

#ifdef laser_length_measure_save_process
    imwrite("/home/klug/img/lengthMeasure/res/mark_src.png",src_img);
#endif
}

/*
    根据振镜角度计算距离，计算方法：u=fx*x+u0*z，v=fy*y+v0*z,结合直线方程建立方程组求解
    @src_img:输入包含激光中心点的图像
    @angle_ein:第一振镜角度
    @angle_zwei:第二振镜角度
    @返回值：到相机坐标系远点的距离
*/
double laser_length_measure::length_measure(cv::Mat src_img,double angle_ein,double angle_zwei)
{
    double length=0.0;

    mathGeometry::geoLineParam line;
    //line=laser_line_shoot(angle_ein,angle_zwei);
    line=laser_line_shoot_camera(angle_ein,angle_zwei);

    cv::Point2f pz;
    laser_zenturm_caltest(src_img,pz); //计算激光中心点位置

    double u=pz.x;
    double v=pz.y;
    double fx=cameraMatrix.at<double>(0,0);
    double fy=cameraMatrix.at<double>(1,1);
    double u0=cameraMatrix.at<double>(0,2);
    double v0=cameraMatrix.at<double>(1,2);

    double temp1=line.b*line.x0-line.a*line.y0;
    double temp2=(u-u0)*line.b/fx;
    double temp3=(v-v0)*line.a/fy;

    length=temp1/(temp2-temp3);
    zenturm_coordinate.z=length;
    zenturm_coordinate.x=line.a*(length-line.z0)/line.c+line.x0;
    zenturm_coordinate.y=line.b*(length-line.z0)/line.c+line.y0;

    length=zenturm_coordinate.z;

    return length;
}
