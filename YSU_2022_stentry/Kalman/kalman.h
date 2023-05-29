/**
  * @author  Liu heng
  * ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½0ï؟½2ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7RoboMasterï؟½1ï؟½7ï؟½1ï؟½7ï؟½0ï؟½7
  */

#ifndef _KALMAN_H
#define _KALMAN_H
#include "Main/headfiles.h"

const int stateNum = 4;                                      //状态值4×1向量(x,y,△x,△y)
const int measureNum = 2;                                    //测量值2×1向量(x,y)

class Kalman{
public:
    Kalman();
    void Kalman_init(double Q=0.00001,double R=0.1);
    Point Kalman_filter( Point measure_point);
    Mat measurement;
private:
    unique_ptr<KalmanFilter> KF;//unique需要用std::move进行传参
    double anti_factor;
    Point last_point;
    bool is_jump;
};


//void Kalman_init(KalmanFilter* KF,double Q=0.00001,double R=0.1);
//Point Kalman_filter(KalmanFilter* KF, Point measure_point);

/**
  * @author  Liu heng
  * ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½0ï؟½2ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7RoboMasterï؟½1ï؟½7ï؟½1ï؟½7ï؟½0ï؟½7
  */




//class Kalman_t {
//public:
//    Kalman_t(){};
//    void KalmanInit(double T_Q,double T_R);
//    double KalmanFilter(double dat);
//private:
//    double X_last; //ï؟½1ï؟½7ï؟½1ï؟½7ï؟½0ï؟½5ï؟½0ï؟½2ï؟½1ï؟½7ï؟½0ï؟½9ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½0ï؟½5ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7  X(k-|k-1)
//    double X_mid;  //ï؟½1ï؟½7ï؟½1ï؟½7ï؟½0ï؟½2ï؟½0ï؟½2ï؟½1ï؟½7ï؟½0ï؟½9ï؟½1ï؟½7ï؟½0ï؟½0ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7  X(k|k-1)
//    double X_now;  //ï؟½1ï؟½7ï؟½1ï؟½7ï؟½0ï؟½2ï؟½0ï؟½2ï؟½1ï؟½7ï؟½0ï؟½9ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½0ï؟½5ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7  X(k|k)
//    double P_mid;  //ï؟½1ï؟½7ï؟½1ï؟½7ï؟½0ï؟½2ï؟½0ï؟½2ï؟½1ï؟½7ï؟½1ï؟½7ï؟½0ï؟½0ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½ï؟½ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7  P(k|k-1)
//    double P_now;  //ï؟½1ï؟½7ï؟½1ï؟½7ï؟½0ï؟½2ï؟½0ï؟½2ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½0ï؟½5ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½ï؟½ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7  P(k|k)
//    double P_last; //ï؟½1ï؟½7ï؟½1ï؟½7ï؟½0ï؟½5ï؟½0ï؟½2ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½0ï؟½5ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½ï؟½ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7  P(k-1|k-1)
//    double kg;     //kalmanï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7
//    double A;      //ï؟½0ï؟½3ï؟½0ï؟½1ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7
//    double B;
//    double Q;
//    double R;
//    double H;


//};




#endif
