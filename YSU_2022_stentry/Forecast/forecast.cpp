/*
@auther:糊盒员
通过历史观测结果拟合计算函数，预测pretime后的结果。
*************************************************************************
2022.06.05测试取用历史上7帧数据进行拟合，预测量0.5秒，结果正确，流畅顺滑，无突变。
理论上7帧可以直接对任何6阶导数是常数的运动进行预测，但是打符还需要重新调参尝试。
当前拟合目标函数形如f(t)=A*exp(-λ*t)+b，其中A，λ，b为待定系数。
打符的话可根据结果理想程度选用sin函数或指数函数
*/


#include "Main/headfiles.h"
#include "Forecast/forecast.h"
#include "Timer/Timer.h"
#include <gsl/gsl_blas.h>

#include <gsl/gsl_matrix.h> // 包含GSL库中的矩阵相关数据结构和函数
#include <gsl/gsl_vector.h> // 包含GSL库中的向量相关数据结构和函数
#include <gsl/gsl_multifit.h> // 包含GSL库中的多项式拟合和线性回归相关函数

#include<gsl_permutation.h>
#include <gsl/gsl_linalg.h>

#define DEBUG 1
#define max_time 60000
//60000
Mat show;
Mat k_show;

String x_c1="0",x_c2="0",x_c3="0",x_t="0";

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
 bool OnSegment(const Point2f &a,const Point2f &b,const Point2f &c)  //a,b,c共线时有效
{
    return c.x >= min(a.x, b.x) && c.x <= max(a.x, b.x) && c.y >= min(a.y, b.y) && c.y <= max(a.y, b.y);
}
 double xmult(const Point2f &a,const Point2f &b,const Point2f &c) //return 为正时=顺时针
{
    return (b.x - a.x)*(c.y - a.y) - (b.y - a.y)*(c.x - a.x);
}//向量叉乘
bool intersection_(const Point2f &a, const Point2f &b,const Point2f &c,const Point2f &d)//判断线段ab、cd是否相交
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
bool intersection_(const vector<Point2f> & line1,const vector<Point2f> &line2)//vector<Point2f> line(2),代表一线段
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
bool rect_line_intersection(const vector<Point2f> & line, const vector<Point2f>& rect)//rect的四个顶点(roi.x,roi.y),(roi.x,roi.y+roi.height-1),(roi.x+roi.width-1,roi.y),(roi.x+roi.width-1,roi.y+roi.height-1)
{
    if(line.size() != 2) return false;
    for (int i = 0; i < 4; i++)
    {
        vector<Point2f> line1;
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
Kalman kalman_t;
Kalman kalman_x1;
Kalman kalman_x2;
void Forecast::Init()
{
    kalman_p.Kalman_init();
    kalman_t.Kalman_init();
    kalman_x1.Kalman_init();
    kalman_x2.Kalman_init();
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
    pitch_history.clear();
    now=chuo();
    recording_interval=0;
    lost_aim_num=0;



    result_center=Point2f(0,0);
    result.emplace_back(Point2f(0,0));
    result.emplace_back(Point2f(0,0));
    result.emplace_back(Point2f(0,0));
    result.emplace_back(Point2f(0,0));
     last_result.emplace_back(Point2f(0,0));
     last_result.emplace_back(Point2f(0,0));
     last_result.emplace_back(Point2f(0,0));
     last_result.emplace_back(Point2f(0,0));

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


 vector<Point2f>& Forecast::
 forcast(vector<Point2f> &original,double time,double y_p_recv[4])
 {
#ifdef DEBUG
    show=Mat(1024,1080,CV_8UC3,Scalar(0,0,0));//观测结果，预测结果可视化
    k_show=Mat(1024,1080,CV_8UC3,Scalar(0,0,0));
#endif

     //当前帧未发现目标
     if(original[0]==Point2f(0,0)&&original[1]==Point2f(0,0)&&original[2]==Point2f(0,0)&&original[3]==Point2f(0,0))
     {   //当前帧未发现目标
         lost_aim_num++;
         if(lost_aim_max==lost_aim_num)//超过max帧未发现目标则认为是丢失目标，清楚历史记录
         {
             //cout<<"                lost_aim: "<<lost_aim_num<<endl;
             //cout<<"                lost_aim: "<<lost_aim_num<<endl;
          record_history.clear();
          pitch_history.clear();
          lost_aim_num=0;
          last_result={Point2f(0,0),Point2f(0,0),Point2f(0,0),Point2f(0,0)};
          kalman_p.Kalman_init();
          return original;
         }else{
             original[0].x++;
             return  original;
         }

     }
     //***4.11更改，设置时间最大值
      int time_temp=time;
      if(record_history.size()>1)
      {
          if(abs(record_history[record_history.size()-2].time-record_history[record_history.size()-1].time)>max_time-200)
              {
                  //record_history.clear();
              }
      }
      time_temp=time_temp%max_time;
      time=time_temp;
//cout<<"                       time:"<<time<<endl;
      //***
      now=chuo(original,time);//
      original_vec_length=sqrt(pow(now.rect[0].x-now.center.x,2)+pow(now.rect[0].y-now.center.y,2));
      for(int i=0;i<4;i++)//记录四个顶点相对于中心点的方向向量。
      {vec_distribution[2*i]=now.rect[i].x-now.center.x;
       vec_distribution[2*i+1]=now.rect[i].y-now.center.y;
      }

      //for(int i=0;i<4;i++){result_limited[i]=Point2f(now.center.x+limited_ratio*vec_distribution[2*i],now.center.y+limited_ratio*vec_distribution[2*i+1]);}

        //每间隔record_history_interval_max帧记录一次观测结果，如果记录结果数量超过观测需要，则删除最早记录
      recording_interval++;
      if(recording_interval>=record_history_interval_max){
          recording_interval=0;
          record_history.emplace_back(now);
          pitch_history.emplace_back(y_p_recv[2]);

          while(record_history.size()>record_history_size){record_history.erase(record_history.begin());pitch_history.erase(pitch_history.begin());}
      }



      //cout<<"                          record_history "<<record_history.size()<<" "<<record_history_interval_max<<endl;
      //cout<<"                          record_history "<<record_history.size()<<" "<<record_history_interval_max<<endl;
      if(record_history.size()>=record_history_size)//记录数量达到观测需要再进行预测
      {
          return original;
        result.clear();
        lost_aim_num=0;
        //预测Api，仅针对中心点进行预测。
        //kalman_p.Kalman_filter(record_history[record_history.size()-1].center);
        //get_forecast();
        //result_center=kalman_p.Kalman_filter(result_center);

        //Point2f k_center=kalman_p.Kalman_filter(result_center);
        //result_center=k_center;

//        Point2f speed={0,0};
//        vector<Point2f>speeds;
//        for(int i=1;i<record_history.size();i++)
//        {

//            if((record_history[i].time-record_history[i-1].time)!=0)
//            {
//                speeds.emplace_back(
//                            Point2f((float)((record_history[i].center.x-record_history[i-1].center.x)/ (record_history[i].time-record_history[i-1].time)),
//                                 (float)((record_history[i].center.y-record_history[i-1].center.y)/ (record_history[i].time-record_history[i-1].time)))
//                        );
//            }
//        }
//        for(int i=0;i<speeds.size();i++)
//        {
//            speed+=speeds[i];
//        }
//        if(speeds.size()!=0)
//        {
//            speed.x/=speeds.size();
//            speed.y/=speeds.size();
//        }
//        else
//        {
//            speed={0,0};
//        }
//        //speed=kalman_v.Kalman_filter(speed);
//        x_c1=to_string(speed.x),x_c2=to_string(speed.y);
//        result_center=Point2f((float)(record_history[record_history.size()-1].center.x),(float)(record_history[record_history.size()-1].center.y));
//        result_center=(result_center+speed*pre_time);
//        //result_center=kalman_p.Kalman_filter(result_center);

        result_center=kalman_p.Kalman_filter(record_history[record_history.size()-1].center);
        Point last_result_center=Point2f((last_result[0].x+last_result[1].x+last_result[2].x+last_result[3].x)/4,(last_result[0].y+last_result[1].y+last_result[2].y+last_result[3].y)/4);



        int x_min_error=10;
        int x_max_error=100;
        int y_error=1;
        Point2f original_center=record_history[record_history.size()-1].center;
        if(abs(result_center.x-original_center.x)<x_min_error)
        {
            result_center.x=original_center.x;
        }

        if(result_center.x-original_center.x>x_max_error)
        {
            result_center.x=original_center.x+x_max_error;
        }

        if(result_center.x-original_center.x<-x_max_error)
        {
            result_center.x=original_center.x-x_max_error;
        }
        if(result_center.y-original_center.y>y_error)
        {
            result_center.y=original_center.y+y_error;
        }
        if(result_center.y-original_center.y<-y_error)
        {
            result_center.y=original_center.y-y_error;
        }

        //result_center.y=original_center.y;
        


//        result_center=Kalman_filter(&KF,result_center );
//zzrtest



              //result_center=kalman.Kalman_filter(result_center );

        //对输出结果进行卡尔曼滤波视调参情况决定是否使用。
//        result_center.x=p_kal[0]->KalmanFilter(result_center.x);
//        result_center.y=p_kal[1]->KalmanFilter(result_center.y);

        //通过中心点和方向向量，复原目标装甲板。
        int error=70;
        if(result_center.x-record_history[record_history.size()-1].center.x>error)
            result_center.x=record_history[record_history.size()-1].center.x+error;
        else if(result_center.x-record_history[record_history.size()-1].center.x<-error)
            result_center.x=record_history[record_history.size()-1].center.x-error;
        if(result_center.y-record_history[record_history.size()-1].center.y>error)
            result_center.y=record_history[record_history.size()-1].center.y+error;
        else if(result_center.y-record_history[record_history.size()-1].center.y<-error)
            result_center.y=record_history[record_history.size()-1].center.y-error;


            vector<Point2f> k_result;
        for(int i=0;i<4;i++)
        {
            result.emplace_back(Point2f(result_center.x+vec_distribution[2*i], result_center.y+vec_distribution[2*i+1]));
            //k_result.emplace_back(Point2f(k_center.x+vec_distribution[2*i], k_center.y+vec_distribution[2*i+1]));
        }
        last_result=result;
        //调换四个顶点顺序，使其符合solvepnp要求。如果装甲板输出结果已经排序，此处不需要。
        // Point2f p[4]{*result.begin(),*(result.begin()+1),*(result.begin()+2),*(result.begin()+3)};
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


       //cout<<"  \n\n\n\n                   result:"<<result.size()<<"\n\n\n";
       line(show,result[0],result[1],Scalar(0,0,254),2);
       line(show,result[1],result[2],Scalar(0,0,254),2);
       line(show,result[2],result[3],Scalar(0,0,254),2);
       line(show,result[3],result[0],Scalar(0,0,254),2);
       //x_c1=to_string(x_min_error);
       //x_c2=to_string(x_max_error);
       String s="min_x: "+x_c1+" max_x: "+x_c2;

       putText(show,s,Point(100,100),FONT_HERSHEY_SIMPLEX,1,Scalar(255,255,255),2);

       cout<<"original.center"<<Point2f((original[0].x+original[1].x+original[2].x+original[3].x)/4,(original[0].y+original[1].y+original[2].y+original[3].y)/4)<<endl;
       cout<<"forecast.center"<<Point2f((lu.x+rd.x+ld.x+ru.x)/4,(lu.y+rd.y+ld.y+ru.y)/4)<<endl;
       cout<<"result.center"<<Point2f((result[0].x+result[1].x+result[2].x+result[3].x)/4,(result[0].y+result[1].y+result[2].y+result[3].y)/4)<<endl;
       imshow("forecast",show);
       //imshow("k_F",show);
       imwrite("/home/robomaster/qt_workspace/picture/"+to_string(time)+".jpg",show);
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
double Forecast::my_gsl(data d, double aim_time)
{
    int time_resize=100;
    aim_time/=time_resize;

    const size_t n = d.n; // 样本数量
    const size_t p = 3; // 参数数量
    //double x[n] = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
    //double y[n] = {1.2, 2.1, 3.1, 4.0, 5.2, 6.1, 7.0, 8.2, 9.1, 10.2};

    // 构造设计矩阵和响应向量
    gsl_matrix *T = gsl_matrix_alloc(n, p);
    gsl_vector *Y = gsl_vector_alloc(n);
    for (int i = 0; i < n; i++) {
        double ti = d.t[i]/time_resize;
        gsl_matrix_set(T, i, 0, ti * ti);
        gsl_matrix_set(T, i, 1, ti);
        gsl_matrix_set(T, i, 2, 1.0);
        gsl_vector_set(Y, i, d.y[i]);
    }

    // 构造正则化矩阵
    double lambda = 0.1; // 正则化参数
    gsl_matrix *R = gsl_matrix_alloc(p, p);
    gsl_matrix_set_zero(R);
    gsl_matrix_set(R, 0 , 0 ,lambda);
    gsl_matrix_set(R, 1 , 1, lambda);
    gsl_matrix_set(R, 2 , 2, lambda);

    // 计算T^T T + lambda R
    gsl_matrix *TtT = gsl_matrix_alloc(p, p);
    gsl_blas_dgemm(CblasTrans, CblasNoTrans, 1.0, T, T, 0.0, TtT);
    gsl_matrix_add(TtT, R);

    // 计算X^T Y
    gsl_vector *TtY = gsl_vector_alloc(p);
    gsl_blas_dgemv(CblasTrans, 1.0, T, Y, 0.0, TtY);

    // 解线性方程组 (X^T X + lambda R) beta = X^T Y
    gsl_vector *beta = gsl_vector_alloc(p);
    gsl_permutation *perm = gsl_permutation_alloc(p);
    int signum;
    gsl_linalg_LU_decomp(TtT, perm, &signum);
    gsl_linalg_LU_solve(TtT, perm, TtY, beta);

    // 输出结果

    //std::cout << "a: " << gsl_vector_get(beta, 0) << std::endl;
    //std::cout << "b: " << gsl_vector_get(beta, 1) << std::endl;
    //std::cout << "c: " << gsl_vector_get(beta, 2) << std::endl;
    //std::cout << "a: " << gsl_vector_get(beta, 0) << std::endl;
    //std::cout << "b: " << gsl_vector_get(beta, 1) << std::endl;
    //std::cout << "c: " << gsl_vector_get(beta, 2) << std::endl;
    auto a=gsl_vector_get(beta, 0);
    auto b=gsl_vector_get(beta, 1);
    auto c=gsl_vector_get(beta, 2);
    x_c1=to_string(a),x_c2=to_string(b),x_c3=to_string(c),x_t=to_string(aim_time);
    // 释放内存
    gsl_matrix_free(T);
    gsl_vector_free(Y);
    gsl_matrix_free(R);
    gsl_matrix_free(TtT);
    gsl_vector_free(TtY);
    gsl_vector_free(beta);
    gsl_permutation_free(perm);
    return a*aim_time*aim_time+b*aim_time+c;


}

 void Forecast::get_forecast()
  {/*虽然是二维图像，但是本质上x的变化和y的变化并无关联，应该视为两个独立变量，分别是关于时间t的函数*/

     for(int i=0;i<record_history_size;i++)
     {//分别为t,x,y历史值赋值
        t[i]=record_history[i].time;
//        Point kPreCenter=kalman_t.Kalman_filter(record_history[i].center);
//        ft[0][i]=kPreCenter.x;
//        ft[1][i]=kPreCenter.y;
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
    aim_time+=pre_time;
//    //****
     //cout<<"\n\n                                     aim_time"<<aim_time<<endl;

     //cout<<"\n\n                                     pre_time"<<pre_time<<endl<<"                                     aim_time:"<<aim_time<<endl;
     //cout<<"\n\n                                     pre_time"<<pre_time<<endl<<"                                     aim_time:"<<aim_time<<endl;
     //result_center=Point2f(gsl_compute(p_gsl[0],d[0],aim_time,weight[0]),gsl_compute(p_gsl[1],d[1],aim_time,weight[1]));
     result_center=Point2f(my_gsl(d[0],aim_time),my_gsl(d[1],aim_time));

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
//              for(vector<Point2f>::iterator it=((record_history.begin())->rect.begin()+j);
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

//           result.emplace_back(Point2f(*resx.begin(),*resy.begin()));
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
