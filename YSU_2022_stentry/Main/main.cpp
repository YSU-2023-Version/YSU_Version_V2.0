//#define DEBUG

#ifndef DEBUG
#include "Main/headfiles.h"
#include "ThreadManager/thread_heyuan.h"

int main()
{
    // threadManager* thread_manager_ptr = new ThreadManager();

    threadManager thread_manager;

    std::thread producer( &threadManager::Produce,&thread_manager );

    std::thread consumer( &threadManager::Consume,&thread_manager );
    std::thread communication( &threadManager::Communicate,&thread_manager );

    producer.join();
    consumer.join();
    communication.join();


    return 0;
}
#else
#include <iostream>
#include <opencv2/opencv.hpp>
#include <Kalman/kalman.h>
using namespace cv;

// 鼠标回调函数，用于获取鼠标点击位置作为观测值
void onMouse(int event, int x, int y, int flags, void* userdata)
{
    if (event == EVENT_MOUSEMOVE)
    {
        Point* measurement = (Point*)userdata;
        measurement->x = x;
        measurement->y = y;
    }
}
Kalman k;

int main()
{
    k.Kalman_init();
    // 定义状态向量
    Point state(0, 0);
    // 定义状态转移矩阵
    Mat transitionMatrix = (Mat_<float>(2, 2) << 1, 0, 0, 1);
    // 定义观测矩阵
    Mat measurementMatrix = (Mat_<float>(2, 2) << 1, 0, 0, 1);
    // 定义过程噪声协方差矩阵
    Mat processNoiseCov = (Mat_<float>(2, 2) << 1e-5, 0, 0, 1e-5);
    // 定义测量噪声协方差矩阵
    Mat measurementNoiseCov = (Mat_<float>(2, 2) << 1e-1, 0, 0, 1e-1);
    // 定义先验误差协方差矩阵
    Mat errorCovPre = (Mat_<float>(2, 2) << 1, 0, 0, 1);
    // 定义后验误差协方差矩阵
    Mat errorCovPost = (Mat_<float>(2, 2) << 1, 0, 0, 1);
    // 定义初始状态估计误差协方差矩阵
    Mat initialStateCov = (Mat_<float>(2, 2) << 1, 0, 0, 1);
    // 定义初始状态估计
    Mat initialState = (Mat_<float>(2, 1) << 0, 0);
    // 定义初始观测值
    Point measurement(0, 0);
    // 创建窗口
    namedWindow("Kalman Filter");
    // 设置鼠标回调函数
    setMouseCallback("Kalman Filter", onMouse, &measurement);

    // 创建Kalman Filter对象
    KalmanFilter kf(2, 2, 0);

    // 初始化Kalman Filter对象
    kf.transitionMatrix = transitionMatrix;
    kf.measurementMatrix = measurementMatrix;
    kf.processNoiseCov = processNoiseCov;
    kf.measurementNoiseCov = measurementNoiseCov;
    kf.errorCovPre = errorCovPre;
    kf.errorCovPost = errorCovPost;
    kf.statePre = initialState;
    kf.statePost = initialState;
    kf.statePost.at<float>(0) = measurement.x;
    kf.statePost.at<float>(1) = measurement.y;

    // 开始追踪鼠标

    while (true)
    {
        // 预测下一步的状态
        Mat prediction = kf.predict();

        // 获取新的观测值
        kf.correct((Mat_<float>(2, 1) << measurement.x, measurement.y));
        state=measurement;
        state=k.Kalman_filter(state);
        // 更新状态向量
        //state.x = kf.statePost.at<float>(0);
        //state.y = kf.statePost.at<float>(1);

        //Point a=KalmanFilter()
        // 绘制状态向量
        Mat canvas(1080, 1080, CV_8UC3, Scalar(255, 255, 255));
        circle(canvas, state, 5, Scalar(0, 0, 255), -1);
        imshow("Kalman Filter", canvas);
        int c=waitKey(50);
        if(c==13)//回车
            break;
    }
    return 0;
}
#endif
