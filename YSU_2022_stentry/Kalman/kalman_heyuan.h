#ifndef FORECAST_T
#define FORECAST_T

#include "Main/headfiles.h"
#include <iostream>
#include <thread>
#include <vector>
#include <memory>


//Tool class
class Kalman_t  {
public:
   Kalman_t(){};
   friend class Forecast;
private:
   void KalmanInit(float &T_Q,float &T_R);
   float KalmanForecast();
   float KalmanFilter(float& dat);
   float X_last; //ï؟½1ï؟½7ï؟½1ï؟½7ï؟½0ï؟½5ï؟½0ï؟½2ï؟½1ï؟½7ï؟½0ï؟½9ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½0ï؟½5ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7  X(k-|k-1)
   float X_mid;  //ï؟½1ï؟½7ï؟½1ï؟½7ï؟½0ï؟½2ï؟½0ï؟½2ï؟½1ï؟½7ï؟½0ï؟½9ï؟½1ï؟½7ï؟½0ï؟½0ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7  X(k|k-1)
   float X_now;  //ï؟½1ï؟½7ï؟½1ï؟½7ï؟½0ï؟½2ï؟½0ï؟½2ï؟½1ï؟½7ï؟½0ï؟½9ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½0ï؟½5ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7  X(k|k)
   float P_mid;  //ï؟½1ï؟½7ï؟½1ï؟½7ï؟½0ï؟½2ï؟½0ï؟½2ï؟½1ï؟½7ï؟½1ï؟½7ï؟½0ï؟½0ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½ï؟½ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7  P(k|k-1)
   float P_now;  //ï؟½1ï؟½7ï؟½1ï؟½7ï؟½0ï؟½2ï؟½0ï؟½2ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½0ï؟½5ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½ï؟½ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7  P(k|k)
   float P_last; //ï؟½1ï؟½7ï؟½1ï؟½7ï؟½0ï؟½5ï؟½0ï؟½2ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½0ï؟½5ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½ï؟½ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7  P(k-1|k-1)
   float kg;     //kalmanï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7
   float A;      //ï؟½0ï؟½3ï؟½0ï؟½1ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7
   float B;
   float Q;
   float R;
   float H;
};


//loop: getReal()->getNextForecast()
class Forecast{
public:
    static const int Num = 2;
    Forecast(float TQ = 0.00001 , float TR = 0.1);
    void Reset();
    Point2f& getNextForecast();
    Point2f& getReal(Point2f& observation);
private:
    Point2f ForecastValue;
    Point2f RealValue;
    std::vector<std::unique_ptr<Kalman_t>> Kalmaners;
    float T_Q,T_R;
};

#endif


