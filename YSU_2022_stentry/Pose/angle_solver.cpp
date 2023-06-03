#include "Main/headfiles.h"
#include "Pose/angle_solver.h"
#include <Kalman/kalman.h>
// #define DEBUG // 打开调试模式，输出yam轴pitch轴误差

AngleSolver::AngleSolver():p_forecast_(std::make_unique<Forecast>())
{

}

void AngleSolver::InitAngle()
{
    cam=Mat(3,3,CV_32FC1,Scalar::all(0));
    disCoeffD=Mat(1,5,CV_32FC1,Scalar::all(0));

    string file_path="../xml_path/camera.xml";
    FileStorage fr;
    fr.open(file_path,FileStorage::READ);
    while(!fr.isOpened()){
        cout<<"camera xml floading failed..."<<endl;
        fr=FileStorage(file_path,FileStorage::READ);
    }
    fr["camera-matrix"]>>cam;
    fr["distortion"]>>disCoeffD;
    fr.release();

    obj = vector<Point3f>{
            cv::Point3f(-61.5f,-30.0f,0),
            cv::Point3f(61.5f,-30.0f,0),
            cv::Point3f(61.5f,30.0f,0),
            cv::Point3f(-61.5f,30.0f,0)
    };
    rVec = cv::Mat::zeros(3,1,CV_64FC1);
    tVec = cv::Mat::zeros(3,1,CV_64FC1);
    // 迭代法重力补偿初始化
    gaf_solver = std::make_shared<rmoss_projectile_motion::GafProjectileSolver>(25, 0.01);
    projectile_tansformoss_tool = std::make_shared<rmoss_projectile_motion::GimbalTransformTool>(gaf_solver);
}

inline void getMiddle(const std::vector<Point2f>& obj,Point2f& res){
    res = Point2f((obj[0].x+obj[1].x+obj[2].x+obj[3].x)/4,
            (obj[0].y+obj[1].y+obj[2].y+obj[3].y)/4);
}
inline void getTranslationObj(const Point2f& oldMiddle,const Point2f& newMiddle,
                              const vector<Point2f>& OldObj, vector<Point2f> &newObj){
    newObj.clear();
    for (auto &point:OldObj){
        newObj.emplace_back(
                    Point2f(point.x-oldMiddle.x+newMiddle.x,
                            point.y-oldMiddle.y+newMiddle.y));
    }
}

inline void showForecast(cv::Point2f& src,std::vector<cv::Point2f>&src_obj,
                         cv::Point2f& forecast,std::vector<cv::Point2f>&forecast_obj,
                         cv::Mat &img){
    circle(img,src,5,Scalar(0,0,255));
    circle(img,forecast,5,Scalar(0,255,0));
    for(int i=0; i<4 ; i++){
        line(img,src_obj[i],src_obj[(i+1)%4],Scalar(0,0,255),3);
        line(img,forecast_obj[i],forecast_obj[(i+1)%4],Scalar(0,255,0),3);
    }
    imshow("forecast",img);
    waitKey(1);
}

double * AngleSolver::SolveAngle(vector<Point2f>& observation_obj){
    if(!(observation_obj[0] == Point2f(0, 0)
         && observation_obj[1] == Point2f(0, 0)
         && observation_obj[2] == Point2f(0, 0)
         && observation_obj[3] == Point2f(0, 0)))
    {

     getMiddle(observation_obj,middle);
     p_forecast_ ->getReal(middle);
     auto &forecast_middel = p_forecast_ -> getNextForecast();
     getTranslationObj(middle,forecast_middel , observation_obj, object_armor_points_ );


//    cv::Mat img;
//    showForecast(middle,observation_obj,forecast_middel,object_armor_points_,img);


    cv::solvePnP(obj,object_armor_points_,cam,disCoeffD,rVec,tVec,false,SOLVEPNP_ITERATIVE);
    _xErr = atan(tVec.at<double>(0, 0) / tVec.at<double>(2, 0)) / 2 / CV_PI * 360;//+5
    _yErr = atan(tVec.at<double>(1, 0) / tVec.at<double>(2, 0)) / 2 / CV_PI * 360;


    if(_yErr < 18 && _yErr > -18)
    {
        p_y_err[0] = _xErr;
        p_y_err[1] = _yErr;
    }
    else
    {
        p_y_err[0] = 0;
        p_y_err[1] = 0;
    }

    p_y_err[0] = _xErr;
    p_y_err[1] = _yErr;
    //求距离
    double x_pos=tVec.at<double>(0,0)/1000;
    double y_pos=tVec.at<double>(1,0)/1000;
    double z_pos=tVec.at<double>(2,0)/1000;
    distance_3d=sqrt(x_pos*x_pos+y_pos*y_pos+z_pos*z_pos);
    gra_t=distance_3d/BULLETFIRE_V;

    //_yErr = _yErr-(CAM_SUPPORT_DIS+4.9*gra_t*gra_t)*0.001;
    std::cout << "pitch_before=" << _yErr << std::endl;
    std::cout << "yaw_before=" << _xErr << std::endl;

    //projectile_tansformoss_tool->solve(x_pos, y_pos, z_pos, p_y_err[1], p_y_err[0]);
    // 根据重力补偿的demo.cpp单位进行调整
//    p_y_err[0] = p_y_err[0] * 180 / CV_PI + 0.8; //yaw
//    p_y_err[1] = -p_y_err[1]*180/CV_PI + 2.5;  //pitch

 //   gravity_comp();

    shoot=1;
    float aim_angle = (p_y_err[1])*CV_PI/180;
    Point2f aim_pos = {(float)(cos(aim_angle)*distance_3d),(float)(sin(aim_angle)*distance_3d)};
    cout<<"aim_pos"<<aim_pos<<endl;
    //补偿后的角度
    //double comp_angle=aim_angle;
    double comp_angle=gravity_compensation(aim_pos,20);//小弹丸射速15m/s   英雄为20   //*****************************调初速度


    p_y_err[0] = p_y_err[0]- 0.5 ;//-7.8 ;//yaw增加方向为左方向，伏视角逆时针方向   原0.6
    p_y_err[1]+=(-comp_angle+aim_angle)*180/CV_PI + 2.5;//pitch

    cout<<"                                    yaw"<<p_y_err[0]<<endl;
    cout<<"                                  pitch"<<p_y_err[1]<<endl;

#ifdef DEBUG
    std::cout << "3.pitch_jiaodu=" << p_y_err[1] << std::endl;
    std::cout << "2.yaw=" << p_y_err[0] << std::endl;
    std::cout << "1.shoot=" << shoot << std::endl;
    //cout<<"                                     comp_angle:"<<comp_angle<<endl;
    //cout<<"                             aim_angle:"<<aim_angle<<endl;
    cout<<"                             comp_angle-aim_angle:"<<comp_angle-aim_angle<<endl;
    //std::cout<<"distance= " << distance_3d << std::endl;
    //std::cout << "pitch=" << p_y_err[1] << std::endl;
    //std::cout << "yaw=" << p_y_err[0] << std::endl;
#endif // DEBUG
    }
    else
    {
        p_y_err[0] = 0;
        p_y_err[1] = 0;//top_min=-19
        shoot=0;
        p_forecast_.reset();
    }


    return p_y_err;
}
int AngleSolver::shoot_get()
{
    return shoot;
}

void AngleSolver::P4P_solver()
{
    double x_pos = tVec.at<double>(0, 0);
    double y_pos = tVec.at<double>(1, 0);
    double z_pos = tVec.at<double>(2, 0);

    double tan_pitch = y_pos / sqrt(x_pos*x_pos + z_pos * z_pos);
    double tan_yaw = x_pos / z_pos;
    x_pitch = -atan(tan_pitch) * 180 / CV_PI;
    y_yaw = atan(tan_yaw) * 180 / CV_PI;

    //！cout<<"x_pitch"<<x_pitch<<endl;
    //！cout<<"y_yaw"<<y_yaw<<endl;
}

void AngleSolver::gravity_comp()
{


//下面是旧版的重力补偿
    gra_t=distance_3d/BULLETFIRE_V;
    if(distance_3d>GRACOMP_DIS)
    {
        p_y_err[0] = _xErr;
//       p_y_err[1] = _yErr;
        p_y_err[1] = _yErr-(CAM_SUPPORT_DIS+4.9*gra_t*gra_t)*0.001;
        //p_y_err[1] = _yErr-(BULLETFIRE_V*(distance_3d/BULLETFIRE_V)*(CAM_SUPPORT_DIS/distance_3d)+0.5*9.8*gra_t*gra_t);

    }
    else
    {
        p_y_err[0] = _xErr;
        p_y_err[1] = _yErr;
    }
}

//弹道模拟 传入目标点，初速度，角度 模拟出打击点  需要调的参数为T G K 初速度 ，T为模拟的间隔时间 G为重力加速度 K为阻力系数
//假如偏上 可以增大初速度，减小阻力   偏下 要减小初速度，增大阻力
//v16.5  K0.001  -0.74  偏上
//v16  K0.0015  -0.75  偏上
Point2f AngleSolver::trajectory_simulation(Point2f aim,double v0,double angle) {
#define T 0.01
#define G 9.8
#define K 0.0000001
#define dp_size 0.05
    Point2f v = {(float)(v0*cos(angle)),(float)(v0*sin(angle))};
    Point2f a;
    Point2f p = {0,0};
    double temp_v2;
    while (1) {
        temp_v2 =sqrt( v.x * v.x + v.y * v.y);
        a.x = (-K * v.x * temp_v2);
        a.y = (-K * v.y * temp_v2 + G);//下面是y轴正方向，所以要+G
        v += a * T;
        p += v * T;
        if (p.x > aim.x) return  { 0,p.y - aim.y }; //超过目标点则返回误差
        if (v.x < 1) return  { 0,p.y - aim.y }; //return { p.x - aim.x,p.y - aim.y };
    }
}
//重力补偿 传入目标点，初速度，返回实际打击角度
double AngleSolver::gravity_compensation(Point2f aim, double v0) {
    Point2f dp; //误差
    Point2f temp_aim=aim;
    double temp_angle;
    for (int i = 0; i < 10; i++) {
        temp_angle = atan(temp_aim.y / temp_aim.x);
        dp = trajectory_simulation(aim, v0, temp_angle);
        temp_aim -= dp;
        if (dp.y<dp_size&&dp.y>-dp_size){
            shoot=1;//可以发射
            return temp_angle;
        }
    }
   shoot=0; //十次迭代之后仍然没有找到目标说明打不到 不能发射
    return temp_angle;
}
