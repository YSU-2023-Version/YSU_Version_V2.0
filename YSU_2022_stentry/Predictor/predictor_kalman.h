/**
 * @date 2023/5/5 创建空类，打算实现角度解算、重力补偿和卡尔曼预测
*/
#ifndef PREDICTOR_KALMAN_H
#define PREDICTOR_KALMAN_H

#include "Main/headfiles.h"

class PredictorKalman
{
private:
    /* data */
public:
    PredictorKalman(/* args */);
    ~PredictorKalman();

    cv::Point2f predict(std::vector<cv::Point2f> src_points);
};



#endif // !PREDICTOR_KALMAN_H 