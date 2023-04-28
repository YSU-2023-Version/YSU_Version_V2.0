/*
@auther:糊盒员
通过历史观测结果拟合计算函数，预测pretime后的结果。
*************************************************************************
2022.06.05测试取用历史上7帧数据进行拟合，预测量0.5秒，结果正确，流畅顺滑，无突变。
理论上7帧可以直接对任何6阶导数是常数的运动进行预测，但是打符还需要重新调参尝试。
当前拟合目标函数形如f(t)=A*exp(-λ*t)+b，其中A，λ，b为待定系数。
打符的话可根据结果理想程度选用sin函数或指数函数
*/

#define DEBUG
#include "Main/headfiles.h"
#include "Forecast/forecast.h"
#include "Timer/Timer.h"
// #define DEBUG 1
#define max_time 60000
Mat show;
double getDistance(const Point2f &a,const Point2f &b)
{
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}
Point2f getMiddle(const Point2f &a,const Point2f &b)
{
    return Point2f(0.5*(a.x + b.x), 0.5*(a.y + b.y));
}
double max(const double &a,const double &b)
{
    return (a > b) ? a : b;
}
double min(const double &a,const double &b)
{
    return (b > a) ? a : b;
}

//拌线检测
//判断两条线段相交
 bool OnSegment(const Point2d &a,const Point2d &b,const Point2d &c)  //a,b,c共线时有效
{
    return c.x >= min(a.x, b.x) && c.x <= max(a.x, b.x) && c.y >= min(a.y, b.y) && c.y <= max(a.y, b.y);
}
 double xmult(const Point2d &a,const Point2d &b,const Point2d &c) //return 为正时=顺时针
{
    return (b.x - a.x)*(c.y - a.y) - (b.y - a.y)*(c.x - a.x);
}//向量叉乘
bool intersection_(const Point2d &a, const Point2d &b,const Point2d &c,const Point2d &d)//判断线段ab、cd是否相交
{
    double d1, d2, d3, d4;
    d1 = xmult(c, d, a);
    d2 = xmult(c, d, b);
    d3 = xmult(a, b, c);
    d4 = xmult(a, b, d);

    if (d1*d2 < 0 && d3*d4 < 0)  return 1;
    else    if (d1 == 0 && OnSegment(c, d, a)) return 1;
    else    if (d2 == 0 && OnSegment(c, d, b)) return 1;
    else    if (d3 == 0 && OnSegment(a, b, c)) return 1;
    else    if (d4 == 0 && OnSegment(a, b, d)) return 1;
    return 0;
}
bool intersection_(const vector<Point2d> & line1,const vector<Point2d> &line2)//vector<Point2f> line(2),代表一线段
{
    if (line1.size() != line2.size())return false;
    if (line1.size() != 2) return false;

    //首先判断line1的两个端点,在line2的两侧
    Point point1_11, point1_12, point1_21, point1_22;
    point1_11 = line2[0] - line1[0];
    point1_12 = line2[1] - line1[0];
    point1_21 = line2[0] - line1[1];
    point1_22 = line2[1] - line1[1];

    //point1_11.cross(point1_12)*point1_21.cross(point1_22)<0;//----------表明在两侧
    //再次判断line2的两个端点，在line1的两侧
    Point point2_11, point2_12, point2_21, point2_22;
    point2_11 = line1[0] - line2[0];
    point2_12 = line1[1] - line2[0];
    point2_21 = line1[0] - line2[1];
    point2_22 = line1[1] - line2[1];

    //point2_11.cross(point2_12)*point2_21.cross(point2_22)<0;
    return (point1_11.cross(point1_12)*point1_21.cross(point1_22) < 0 && point2_11.cross(point2_12)*point2_21.cross(point2_22) < 0);
}
//检测线段是否与矩形框相交(线段在矩形内部判定为不想交）
bool rect_line_intersection(const vector<Point2d> & line, const vector<Point2d>& rect)//rect的四个顶点(roi.x,roi.y),(roi.x,roi.y+roi.height-1),(roi.x+roi.width-1,roi.y),(roi.x+roi.width-1,roi.y+roi.height-1)
{
    if(line.size() != 2) return false;
    for (int i = 0; i < 4; i++)
    {
        vector<Point2d> line1;
        line1.push_back(rect[i]);
        line1.push_back(rect[(i + 1) % 4]);
        if (intersection_(line, line1)) return true;
    }
    return false;
}
/****************************
 * 以上，为辅助函数，需要进行预测结果范围约束时使用。
 * 以下，为预测类，调用回归拟合函数
 * ********************************/

Forecast::Forecast()
{};
//cv::KalmanFilter KF(stateNum, measureNum, 0) ;
Kalman kalman_p;
Kalman kalman_v;
void Forecast::Init()
{
    kalman_p.Kalman_init();
    kalman_v.Kalman_init();
//    Kalman_init(&KF);

    cout<<"forecast init begin"<<endl;
    string file_path="../xml_path/forecast.xml";
    FileStorage fr;
    fr.open(file_path,FileStorage::READ);
    while(!fr.isOpened()){
          cout<<"xml floading failed..."<<endl;
         fr.open(file_path,FileStorage::READ);
    }

    fr["record_history_size"]>>record_history_size;
    fr["record_history_interval_max"]>>record_history_interval_max;
    fr["pre_time"]>>pre_time;
    fr["lost_aim_max"]>>lost_aim_max;
    // fr["real_weight"]>>real_weight;
    // fr["fore_weight"]>>fore_weight;
    // fr["kal_Q"]>>Kalman_Q;
    // fr["kal_R"]>>Kalman_R;



    // if(real_weight>1||real_weight<0.5)real_weight=0.75;
    // if(fore_weight>0.5||fore_weight<0)fore_weight=0.15;
    // if(real_weight+fore_weight>1||real_weight+fore_weight<0)(real_weight=0.75,real_weight=0.15);
    // last_result_weight=1-real_weight-fore_weight;


    record_history.clear();
    now=chuo();
    recording_interval=0;
    lost_aim_num=0;



    result_center=Point2d(0,0);
    result.emplace_back(Point2d(0,0));
    result.emplace_back(Point2d(0,0));
    result.emplace_back(Point2d(0,0));
    result.emplace_back(Point2d(0,0));
    // last_result.emplace_back(Point2d(0,0));
    // last_result.emplace_back(Point2d(0,0));
    // last_result.emplace_back(Point2d(0,0));
    // last_result.emplace_back(Point2d(0,0));

     t=(double*)malloc(sizeof(double)*record_history_size);
    for(int i=0;i<2;i++){
    //  p_kal.emplace_back(std::make_unique<Kalman_t>());
    //  p_kal[i]->KalmanInit(Kalman_Q,Kalman_R);
     p_gsl[i]=new ysugsl();
     gslInit(p_gsl[i]);//c风格，传入指针
     ft[i]=(double*)malloc(sizeof(double)*record_history_size);
     weight[i]=(double*)malloc(sizeof (double)*record_history_size);
     for(int j=0;j<record_history_size;j++)
     {
        weight[i][j]=sqrt(j);//历史帧权重赋值，可根据调参结果进行权重修改。
                            //不建议x，y的权重不同，容易飘。
                            //2022,06,05测试取历史上7帧，理论上考虑了对时间的6阶倒数，打符应该也可以直接使用x、y坐标进行预测。
     }
    }


    cout<<"forecast init succeed"<<endl;


}


 vector<Point2d>& Forecast::
 forcast(vector<Point2d> &original,double time)
 {
#ifdef DEBUG
    show=Mat(1024,1080,CV_8UC3,Scalar(0,0,0));//观测结果，预测结果可视化
#endif
     
     //当前帧未发现目标
     if(original[0]==Point2d(0,0)&&original[1]==Point2d(0,0)&&original[2]==Point2d(0,0)&&original[3]==Point2d(0,0))
     {   //当前帧未发现目标
         lost_aim_num++;
         if(lost_aim_max==lost_aim_num)//超过max帧未发现目标则认为是丢失目标，清楚历史记录
         {
          record_history.clear();
          lost_aim_num=0;
          last_result={Point2d(0,0),Point2d(0,0),Point2d(0,0),Point2d(0,0)};
          return original;
         }else{
             return  original;
         }

     }
     //4.11更改，设置时间最大值
      int time_temp=time;
      time_temp=time_temp%max_time;

      time=time_temp;

      //***
      now=chuo(original,time);//
      original_vec_length=sqrt(pow(now.rect[0].x-now.center.x,2)+pow(now.rect[0].y-now.center.y,2));
      for(int i=0;i<4;i++)//记录四个顶点相对于中心点的方向向量。
      {vec_distribution[2*i]=now.rect[i].x-now.center.x;
       vec_distribution[2*i+1]=now.rect[i].y-now.center.y;
      }

      //for(int i=0;i<4;i++){result_limited[i]=Point2d(now.center.x+limited_ratio*vec_distribution[2*i],now.center.y+limited_ratio*vec_distribution[2*i+1]);}

        //每间隔record_history_interval_max帧记录一次观测结果，如果记录结果数量超过观测需要，则删除最早记录
      recording_interval++;
      if(recording_interval>=record_history_interval_max){
          recording_interval=0;
          record_history.emplace_back(now);
          while(record_history.size()>record_history_size){record_history.erase(record_history.begin());}
      }



      cout<<"                          record_history "<<record_history.size()<<endl;
      if(record_history.size()>=3)//记录数量达到观测需要再进行预测
      {
        result.clear();

        //预测Api，仅针对中心点进行预测。
        //get_forecast();

        Point2d speed={0,0};
        vector<Point2d>speeds;
        for(int i=1;i<record_history.size();i++)
        {

            if((record_history[i].time-record_history[i-1].time)!=0)
            {
                speeds.emplace_back(
                            Point(abs(record_history[i].center.x-record_history[i-1].center.x)/ abs(record_history[i].time-record_history[i-1].time),
                                 (abs(record_history[i].center.y-record_history[i-1].center.y)/ abs(record_history[i].time-record_history[i-1].time)
                        )));
            }
        }
        for(int i=0;i<speeds.size();i++)
        {
            speed+=speeds[i];
        }
        if(speeds.size()!=0)
        {
            speed.x/=speeds.size();
            speed.y/=speeds.size();
        }
        else
        {
            speed={0,0};
        }





//        result_center=Kalman_filter(&KF,result_center );
//zzrtest



              //result_center=kalman.Kalman_filter(result_center );
            speed=kalman_v.Kalman_filter(speed);
            result_center=Point(record_history[record_history.size()-1].center.x,record_history[record_history.size()-1].center.y);
            result_center=kalman_p.Kalman_filter(result_center+speed*pre_time);

        //对输出结果进行卡尔曼滤波视调参情况决定是否使用。
//        result_center.x=p_kal[0]->KalmanFilter(result_center.x);
//        result_center.y=p_kal[1]->KalmanFilter(result_center.y);

        //通过中心点和方向向量，复原目标装甲板。
        for(int i=0;i<4;i++)
        {
            result.emplace_back(Point2d(result_center.x+vec_distribution[2*i], result_center.y+vec_distribution[2*i+1]));
        }

        //调换四个顶点顺序，使其符合solvepnp要求。如果装甲板输出结果已经排序，此处不需要。
        // Point2d p[4]{*result.begin(),*(result.begin()+1),*(result.begin()+2),*(result.begin()+3)};
        // std::sort(p, p + 4, [](const Point2f & p1, const Point2f & p2) {return p1.x < p2.x; });
        // if (p[0].y < p[1].y){
        //     lu = p[0];
        //     ld = p[1];
        // }else{
        //     lu = p[1];
        //     ld = p[0];
        // }

        // if (p[2].y < p[3].y){
        //     ru = p[2];
        //     rd = p[3];
        // }
        // else {
        //     ru = p[3];
        //     rd = p[2];
        // }

        // result.clear();
        // result.emplace_back(lu);
        // result.emplace_back(ru);
        // result.emplace_back(rd);
        // result.emplace_back(ld);

#ifdef DEBUG


       line(show,original[0],original[1],Scalar(0,255,0),1);
       line(show,original[1],original[2],Scalar(0,255,0),1);
       line(show,original[2],original[3],Scalar(0,255,0),1);
       line(show,original[3],original[0],Scalar(0,255,0),1);



//       line(show,result_limited[0],result_limited[1],Scalar(255,255,0),1);
//       line(show,result_limited[1],result_limited[2],Scalar(255,255,0),1);
//       line(show,result_limited[2],result_limited[3],Scalar(255,255,0),1);
//       line(show,result_limited[3],result_limited[0],Scalar(255,255,0),1);



       line(show,result[0],result[1],Scalar(0,0,255),2);
       line(show,result[1],result[2],Scalar(0,0,255),2);
       line(show,result[2],result[3],Scalar(0,0,255),2);
       line(show,result[3],result[0],Scalar(0,0,255),2);

       cout<<"original.center"<<Point2d((original[0].x+original[1].x+original[2].x+original[3].x)/4,(original[0].y+original[1].y+original[2].y+original[3].y)/4)<<endl;
       cout<<"forecast.center"<<Point2d((lu.x+rd.x+ld.x+ru.x)/4,(lu.y+rd.y+ld.y+ru.y)/4)<<endl;
       cout<<"result.center"<<Point2d((result[0].x+result[1].x+result[2].x+result[3].x)/4,(result[0].y+result[1].y+result[2].y+result[3].y)/4)<<endl;
       imshow("forecast",show);
       waitKey(1);
#endif
        //记录历史预测结果，用于混合赋权输出最终结果。通过回归函数预测已经足够顺滑，故不需要了。上机运行一下，不报错就删掉
        // last_result.clear();
        // for(int i=0;i<4;i++){
        //     last_result.emplace_back(result[i]);
        // }

        return result;}
      else{
        return original;
      }
}


 void Forecast::get_forecast()
  {/*虽然是二维图像，但是本质上x的变化和y的变化并无关联，应该视为两个独立变量，分别是关于时间t的函数*/
    
     for(int i=0;i<record_history_size;i++)
     {//分别为t,x,y历史值赋值
        t[i]=record_history[i].time;
        ft[0][i]=record_history[i].center.x;
        ft[1][i]=record_history[i].center.y;
     }
     //分别打包装入gsl算子
     d[0]={(size_t)record_history_size,t,ft[0]};
     d[1]={(size_t)record_history_size,t,ft[1]};

    //生成预测时间，此步应尽可能靠近计算时间，以减少不必要的误差
    double  aim_time;
     getSystime(aim_time);
     //4.11更改，设置时间最大值
    int temp_time=aim_time;
    temp_time=temp_time%max_time;
    aim_time=temp_time;
//    //****
     //cout<<"\n\n                                     aim_time"<<aim_time<<endl;
     aim_time+=pre_time;

     result_center=Point2d(gsl_compute(p_gsl[0],d[0],aim_time,weight[0]),gsl_compute(p_gsl[1],d[1],aim_time,weight[1]));
}


//void Forecast::get_forecast( )
// {
//        vector<double> aim_time, resx,resy;
//        aim_time.clear();
//        resx.clear();
//        resy.clear();

//        vector<double> time;
//        time.clear();
//        for(vector<chuo>::iterator itt=record_history.begin();itt!=record_history.end();itt++)
//        //for(vector<chuo>::iterator itt=record_history.end()-1;;itt--)
//        {
//            time.emplace_back(itt->time);
//            //if(itt==record_history.begin())break;
//        }

//        vector<double> x,y;
//        x.clear();
//        y.clear();
//        //erwei shuzu hangbianli.

//        double aim;
//        getSystime(aim);
//        aim_time.emplace_back(aim+pre_time);

//          int i=0,j=0;
//          for(i=0;j<record_history[0].rect.size();i=0,j++){
//              for(vector<Point2d>::iterator it=((record_history.begin())->rect.begin()+j);
//                it!=((record_history.end()-1)->rect.begin()+j);  )
//                {
//                  it=((record_history.begin()+i)->rect.begin()+j);

//                     x.emplace_back(it->x);
//                     y.emplace_back(it->y);
//                     i++;
//                }

//            //cout<<"flag111"<<endl;
//           lagrangeint(time,x,aim_time,resx);
//           lagrangeint(time,y,aim_time,resy);
//           //cout<<"flag222"<<endl;

//           result.emplace_back(Point2d(*resx.begin(),*resy.begin()));
//           x.clear();
//           y.clear();
//           resx.clear();
//           resy.clear();
//          }
// }

// //拉格朗日差值函数，输入X，Y。求出xp对应的Y值
// bool Forecast::lagrangeint(vector<double>&X, vector<double>&Y, vector<double>&xp, vector<double> &get)
// {
//     double temp1 = 0;
//     int N0 = X.size();
//     int N1 = xp.size();
//     if (X.size() != Y.size())return false;
//     if (N1 == 0)return false;
//     get.clear();

//     for (int h = 0; h < N1; h++)
//     {
//         for (int i = 0; i < N0; i++)
//         {
//             double temp = Y[i];
//             for (int j = 0; j < N0; j++)
//             {
//                 if (i != j)
//                 {
//                     temp = temp * (xp[h] - X[j]);
//                     temp = temp / (X[i] - X[j]);
//                 }
//             }
//             temp1 += temp;
//         }
//         get.emplace_back(temp1);
//         temp1 = 0;
//     }
// }
