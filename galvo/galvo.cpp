/*
    文件等级：密一
    author:klug
    献给沙朗克里斯温雅德
    start:230228
    last:230426
*/

#include "galvo.hpp"
#include <Eigen/Eigen>
#include <Eigen/Core>

galvo::galvo()
{
    cv::Point3f temp(-4,-31.01,45);
    laser_source.a=0; //初始化原始光源出射激光，建立激光出射坐标系
    laser_source.b=-1;
    laser_source.c=0;
    laser_source.x0=0;
    laser_source.y0=0;
    laser_source.z0=0;

    axia_ein.a=4.53;//初始化旋转轴1向量参数，从模型中获得
    axia_ein.b=0;
    axia_ein.c=-16.9;
    axia_ein.x0=3.65-temp.x;
    axia_ein.y0=-89.7-temp.y;
    axia_ein.z0=19.02-temp.z;

    axia_zwei.a=0; //初始化旋转轴2向量x参数，从模型中获得
    axia_zwei.b=-1;
    axia_zwei.c=0;
    axia_zwei.x0=-32.95-temp.x;
    axia_zwei.y0=-71.5-temp.y;
    axia_zwei.z0=37.06-temp.z;

    get_panel(cv::Point3f(1.65-temp.x,-82.47-temp.y,49.09-temp.z),//初始化振镜平面1参数，从模型中获得
              cv::Point3f(-10.99-temp.x,-95.55-temp.y,45.71-temp.z),
              cv::Point3f(3.03-temp.x,-82.47-temp.y,43.94-temp.z),
              ein_galvo_plane.A,
              ein_galvo_plane.B,
              ein_galvo_plane.C,
              ein_galvo_plane.D);

    get_panel(cv::Point3f(-26.69-temp.x,-104.2-temp.y,34.14-temp.z),//初始化振镜平面2参数，从模型中获得
              cv::Point3f(-40.57-temp.x,-80.09-temp.y,44.79-temp.z),
              cv::Point3f(-23.51-temp.x,-80.09-temp.y,31.7-temp.z),
              zwei_galvo_plane.A,
              zwei_galvo_plane.B,
              zwei_galvo_plane.C,
              zwei_galvo_plane.D);
#ifdef galvo_data_print_info
    printf("plane_1_A %f\n",ein_galvo_plane.A);
    printf("plane_1_B %f\n",ein_galvo_plane.B);
    printf("plane_1_C %f\n",ein_galvo_plane.C);
    printf("plane_1_D %f\n",ein_galvo_plane.D);
    printf("plane_2_A %f\n",zwei_galvo_plane.A);
    printf("plane_2_B %f\n",zwei_galvo_plane.B);
    printf("plane_2_C %f\n",zwei_galvo_plane.C);
    printf("plane_2_D %f\n",zwei_galvo_plane.D);
#endif

}

galvo::~galvo()
{
#ifdef galvo_msg_print_info
    printf("close the galvo...");
#endif

}

/*
    迭代寻找最优的角度偏移值
    @shoot:出射光线直线方程参数，用标定板计算得到的
*/
void galvo::galvo_calibrate(math_geometry::geo_line_param shoot)
{
    float step=0.0001;
    math_geometry::geo_plane_param ein_reflect_plane,zwei_reflect_plane; //旋转后的两个振镜平面
    float error_buf=1000;
    float shoot_length=sqrt(shoot.a*shoot.a+shoot.b*shoot.b+shoot.c*shoot.c);

    for(int ein=-870;ein<870;ein++)
    {
        float ein_angle=step*ein;
        ein_reflect_plane=plane_rotate(ein_galvo_plane,axia_ein,ein_angle); //计算第1个振镜旋转平面
        laser_transmit=laser_reflect(laser_source,ein_reflect_plane);

        for(int zwei=-870;zwei<870;zwei++)
        {
            float zwei_angle=step*zwei;
            zwei_reflect_plane=plane_rotate(zwei_galvo_plane,axia_zwei,zwei_angle); //计算第2个振镜旋转平面
            laser_shoot=laser_reflect(laser_transmit,zwei_reflect_plane);
            float laser_shoot_length=sqrt(laser_shoot.a*laser_shoot.a+laser_shoot.b*laser_shoot.b+laser_shoot.c*laser_shoot.c);

            float error=0;
            error=(shoot.a/shoot_length-laser_shoot.a/laser_shoot_length)*(shoot.a/shoot_length-laser_shoot.a/laser_shoot_length);
            error+=(shoot.b/shoot_length-laser_shoot.b/laser_shoot_length)*(shoot.b/shoot_length-laser_shoot.b/laser_shoot_length);
            error+=(shoot.c/shoot_length-laser_shoot.c/laser_shoot_length)*(shoot.c/shoot_length-laser_shoot.c/laser_shoot_length);

            if(error<error_buf)
            {
                error_buf=error;
                deviation_angel_ein=ein_angle;
                deviation_angel_zwei=zwei_angle;
            }
        }
    }

#ifdef galvo_data_print_info
    // 验证出射光线方程
    ein_reflect_plane=plane_rotate(ein_galvo_plane,axia_ein,deviation_angel_ein);
    zwei_reflect_plane=plane_rotate(zwei_galvo_plane,axia_zwei,deviation_angel_zwei);
    laser_transmit=laser_reflect(laser_source,ein_reflect_plane);
    laser_shoot=laser_reflect(laser_transmit,zwei_reflect_plane);

    printf("laser_shoot a %f\n",laser_shoot.a);
    printf("laser_shoot b %f\n",laser_shoot.b);
    printf("laser_shoot c %f\n",laser_shoot.c);
    printf("deviation_angel_ein %f\n",deviation_angel_ein);
    printf("deviation_angel_zwei %f\n",deviation_angel_zwei);
#endif
}

/*
    计算直线和平面的交点
    @line:输入直线方程的参数
    @plane:输入平面方程的参数
    @intersection:返回值，交点的坐标
*/
cv::Point3f galvo::line_plane_intersection(math_geometry::geo_line_param line,math_geometry::geo_plane_param plane)
{
    cv::Point3f intersection;

    float temp1=plane.A*line.a+plane.B*line.b+plane.C*line.c;
    float temp2=-(plane.A*line.x0+plane.B*line.y0+plane.C*line.z0+plane.D);
    float temp3=temp2/temp1;

    intersection.x=line.a*temp3+line.x0;
    intersection.y=line.b*temp3+line.y0;
    intersection.z=line.c*temp3+line.z0;

    return intersection;
}

/*
    计算经过镜面反射的光线的直线方程
    @ld:入射光直线方程
    @plane:反射平面
    @lr:返回值，出射光直线方程方向
*/
math_geometry::geo_line_param galvo::laser_reflect(math_geometry::geo_line_param ld,math_geometry::geo_plane_param plane)
{
    math_geometry::geo_line_param lr;

    float length_ld=sqrt(ld.a*ld.a+ld.b*ld.b+ld.c*ld.c);
    float length_plane=sqrt(plane.A*plane.A+plane.B*plane.B+plane.C*plane.C);

    float cos_theta=ld.a*plane.A+ld.b*plane.B+ld.c*plane.C;

    if(cos_theta<0)
    {
        float ld_a=ld.a/length_ld; //归一化向量
        float ld_b=ld.b/length_ld;
        float ld_c=ld.c/length_ld;
        float plane_A=-plane.A/length_plane; //归一化向量
        float plane_B=-plane.B/length_plane;
        float plane_C=-plane.C/length_plane;

        float length_sum=2*abs(ld_a*plane_A+ld_b*plane_B+ld_c*plane_C); //2*cos（）

        plane_A*=length_sum;
        plane_B*=length_sum;
        plane_C*=length_sum;

        lr.a=plane_A-ld_a;
        lr.b=plane_B-ld_b;
        lr.c=plane_C-ld_c;
    }
    else
    {
        float ld_a=ld.a/length_ld; // 归一化向量
        float ld_b=ld.b/length_ld;
        float ld_c=ld.c/length_ld;
        float plane_A=plane.A/length_plane; // 归一化向量
        float plane_B=plane.B/length_plane;
        float plane_C=plane.C/length_plane;

        float length_sum=2*abs(ld_a*plane_A+ld_b*plane_B+ld_c*plane_C); //2*cos（）

        plane_A*=length_sum;
        plane_B*=length_sum;
        plane_C*=length_sum;

        lr.a=plane_A-ld_a;
        lr.b=plane_B-ld_b;
        lr.c=plane_C-ld_c;
    }

    cv::Point3f intersection=line_plane_intersection(ld,plane);
    lr.x0=intersection.x;
    lr.y0=intersection.y;
    lr.z0=intersection.z;

    return lr;
}

/*
    平面绕定轴旋转固定角度，要求平面平行于轴
    @plane:平面参数
    @line:旋转轴直线方程
    @angle:旋转角度
    @res_plane:返回值，旋转后平面
*/
math_geometry::geo_plane_param galvo::plane_rotate(math_geometry::geo_plane_param plane,math_geometry::geo_line_param line,float angle)
{
    math_geometry::geo_plane_param res_plane;

    float judge=(plane.A*line.a+plane.B*line.b+plane.C*line.c)/(sqrt(line.a*line.a+line.b*line.b+line.c*line.c)*sqrt(plane.A*plane.A+plane.B*plane.B+plane.C*plane.C));
    if(judge>0.001)
    {
#ifdef galvo_error_print_info
        std::cout<<"pl:="<<judge<<std::endl;
        printf("das plane und das axis is not parallel...\n");
#endif
        return res_plane;
    }

    Eigen::Vector3d plane_vector;
    Eigen::AngleAxisd rotate_vector;

    judge=plane.A*line.x0+plane.B*line.y0+plane.C*line.z0+plane.D; //符号判断点在平面的侧
    float temp=sqrt(line.a*line.a+line.b*line.b+line.c*line.c);

    plane_vector.x()=plane.A;
    plane_vector.y()=plane.B;
    plane_vector.z()=plane.C;
    rotate_vector.axis().x()=line.a/temp;
    rotate_vector.axis().y()=line.b/temp;
    rotate_vector.axis().z()=line.c/temp;
    rotate_vector.angle()=angle;

    Eigen::Matrix3d rotate_matrix=rotate_vector.toRotationMatrix();

    plane_vector=rotate_matrix*plane_vector;

    res_plane.A=plane_vector.x();
    res_plane.B=plane_vector.y();
    res_plane.C=plane_vector.z();

    float distance; //旋转轴上的点到平面的距离
    distance=abs(plane.A*line.x0+plane.B*line.y0+plane.C*line.z0+plane.D);
    distance/=sqrt(plane.A*plane.A+plane.B*plane.B+plane.C*plane.C);

    temp=sqrt(res_plane.A*res_plane.A+res_plane.B*res_plane.B+res_plane.C*res_plane.C);

    float temp1=distance*temp;
    float temp2=-distance*temp;

    float d1=temp1-res_plane.A*line.x0-res_plane.B*line.y0-res_plane.C*line.z0;
    float d2=temp2-res_plane.A*line.x0-res_plane.B*line.y0-res_plane.C*line.z0;

    temp1=res_plane.A*line.x0+res_plane.B*line.y0+res_plane.C*line.z0+d1;
    //temp2=res_plane.A*line.x0+res_plane.B*line.y0+res_plane.C*line.z0+d2;

    //判断与原始平面是否同号，同号为直线上的点在同侧
    if(temp1*judge>0)
    {
        res_plane.D=d1;
    }
    else
    {
        res_plane.D=d2;
    }

    return res_plane;
}

/*
    根据两个振镜角度计算出射激光线的方向，在振镜坐标系下
    @angle_ein:第一振镜角度
    @angle_zwei:第二振镜角度
    @laser_line:返回值，出射激光线的直线方程
*/
math_geometry::geo_line_param galvo::laser_line_shoot_galvo(float angle_ein,float angle_zwei)
{
    math_geometry::geo_plane_param plane_temp1=plane_rotate(ein_galvo_plane,axia_ein,angle_ein+deviation_angel_ein);
    math_geometry::geo_plane_param plane_temp2=plane_rotate(zwei_galvo_plane,axia_zwei,angle_zwei+deviation_angel_zwei);

    laser_transmit=laser_reflect(laser_source,plane_temp1);
    laser_shoot=laser_reflect(laser_transmit,plane_temp2);

#ifdef galvo_data_print_info
    printf("galvo shoot.a:=%f\n",laser_shoot.a);
    printf("galvo shoot.b:=%f\n",laser_shoot.b);
    printf("galvo shoot.c:=%f\n",laser_shoot.c);
    printf("galvo shoot.x0:=%f\n",laser_shoot.x0);
    printf("galvo shoot.y0:=%f\n",laser_shoot.y0);
    printf("galvo shoot.z0:=%f\n",laser_shoot.z0);
#endif

    return laser_shoot;
}

/*
    *根据两个振镜角度计算出射激光线的方向，在相机坐标系下
    @angle_ein:第一振镜角度
    @angle_zwei:第二振镜角度
    @laser_line:返回值，出射激光线的直线方程
*/
math_geometry::geo_line_param galvo::laser_line_shoot_camera(float angle_ein,float angle_zwei)
{
    math_geometry::geo_line_param shoot_line,line_temp;

    // 计算旋转后的出射光线
    math_geometry::geo_plane_param plane_temp1=plane_rotate(ein_galvo_plane,axia_ein,angle_ein+deviation_angel_ein);
    math_geometry::geo_plane_param plane_temp2=plane_rotate(zwei_galvo_plane,axia_zwei,angle_zwei+deviation_angel_zwei);

    line_temp=laser_reflect(laser_source,plane_temp1);
    shoot_line=laser_reflect(line_temp,plane_temp2);

    //振镜坐标系下，第二振镜上的点转换到相机坐标系下
    shoot_line.x0-=tran.x;
    shoot_line.y0-=tran.y;
    shoot_line.z0-=tran.z;

#ifdef galvo_data_print_info
    printf("camera shoot.a:=%f\n",shoot_line.a);
    printf("camera shoot.b:=%f\n",shoot_line.b);
    printf("camera shoot.c:=%f\n",shoot_line.c);
    printf("camera shoot.x0:=%f\n",shoot_line.x0);
    printf("camera shoot.y0:=%f\n",shoot_line.y0);
    printf("camera shoot.z0:=%f\n",shoot_line.z0);
#endif

    return shoot_line;
}

/*
    计算平面方程
    @p1:平面上的点1
    @p2:平面上的点2
    @p3:平面上的点3
    @a:平面参数1
    @b:平面参数2
    @c:平面参数3
    @d:平面参数4
*/
void galvo::get_panel(cv::Point3f p1,cv::Point3f p2,cv::Point3f p3,float &a,float &b,float &c,float &d)
{
    a=(p2.y-p1.y)*(p3.z-p1.z)-(p2.z-p1.z)*(p3.y-p1.y);
    b=(p2.z-p1.z)*(p3.x-p1.x)-(p2.x-p1.x)*(p3.z-p1.z);
    c=(p2.x-p1.x)*(p3.y-p1.y)-(p2.y-p1.y)*(p3.x-p1.x);
    d=-(a*p1.x+b*p1.y+c*p1.z);
}

/*
    计算振镜坐标系和相机坐标系的偏差，
    算出两个点，利用直线方程算出三个点的坐标，
    @point:在相机坐标系下，两个点的坐标，根据标定板算出
    @line:振镜坐标系下，两条直线的方程，根据振镜角度算出
    @tran:两个坐标系之间的偏差，定义在振镜的类中
*/
void galvo::galvo2camera(std::vector<cv::Point3f> point,std::vector<math_geometry::geo_line_param> line)
{
    //首先判断容器中数据量是否正确
    if(point.size()!=2&&line.size()!=2)
    {
#ifdef galvo_error_print_info
        printf("point line size is not correct...\n");
#endif
        return ;
    }
    else
    {
#ifdef galvo_control_data_print_info
        printf("p1.x:=%f\n",point[0].x);
        printf("p1.y:=%f\n",point[0].y);
        printf("p1.z:=%f\n",point[0].z);
        printf("p2.x:=%f\n",point[1].x);
        printf("p2.y:=%f\n",point[1].y);
        printf("p2.z:=%f\n",point[1].z);
#endif
    }

    float t11=(point[0].x-line[0].x0)/line[0].a;
    float t12=(point[0].y-line[0].y0)/line[0].b;
    float t13=(point[0].z-line[0].z0)/line[0].c;
    float t14=1/line[0].a;
    float t15=1/line[0].b;
    float t16=1/line[0].c;

    float t21=(point[1].x-line[1].x0)/line[1].a;
    float t22=(point[1].y-line[1].y0)/line[1].b;
    float t23=(point[1].z-line[1].z0)/line[1].c;
    float t24=1/line[1].a;
    float t25=1/line[1].b;
    float t26=1/line[1].c;

    float t1=t25*t11-t15*t21;
    float t2=t25*t14-t15*t24;
    float t3=t25*t12-t15*t22;

    tran.x=(t3-t1)/t2;
    tran.y=(t11+t14*tran.x-t12)/t15;
    tran.z=(t11+t14*tran.x-t13)/t16;

#ifdef galvo_control_data_print_info
    printf("tran.x:=%f\n",tran.x);
    printf("tran.y:=%f\n",tran.y);
    printf("tran.z:=%f\n",tran.z);
#endif
}
