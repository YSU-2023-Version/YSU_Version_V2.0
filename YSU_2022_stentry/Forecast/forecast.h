#ifndef FORECAST_H
#define FORECAST_H

#include "Main/headfiles.h"//
#include "Kalman/kalman.h"
#include "Forecast/ysu_gsl.h"



struct chuo//li shi chuo
{
    vector<Point2d> rect;
    Point2d center;
    double time;
    chuo(){};
    chuo(vector<Point2d> &ora,double &time_):rect(ora),time(time_),center(Point2d((ora[0].x+ora[1].x+ora[2].x+ora[3].x)/4,(ora[0].y+ora[1].y+ora[2].y+ora[3].y)/4)){};
};


class Forecast
{public:
    Forecast();
    void Init();
    vector<Point2d>& forcast(vector<Point2d> &original,double time);
    Point2f lu, ld, ru, rd;


private:

    void get_forecast();
  //  bool lagrangeint(vector<double>&X, vector<double>&Y, vector<double>&xp, vector<double> &get);
    chuo now;
    vector<chuo> record_history;
    vector<Point2d> result;

    vector<Point2d>  last_result;

    int record_history_size;

    int record_history_interval_max;
    int recording_interval;

    double pre_time;


    int lost_aim_max;
    int lost_aim_num;

    // 各结果混合赋权滤波输出，如果单纯预测结果足够逼近实际且顺滑，则此部不需要。
    // double real_weight;
    // double fore_weight;
    // double last_result_weight;



    // double Kalman_Q,Kalman_R;
    //  vector<unique_ptr<Kalman_t>> p_kal;


    Point2d result_center;
    float vec_result_to_limited[2];
    float result_correct_ratio;
    double original_vec_length;
    double vec_distribution[8];

   ysugsl* p_gsl[2];

    double *ft[2];
    data d[2];
    double *weight[2];//预测历史帧权重。
    double *t;

<<<<<<< HEAD
    vector<Point2d> centers;//用于卡尔曼计算速度
=======

>>>>>>> origin/master

};

#endif // FORECAST_H
