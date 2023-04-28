#ifndef THREADMANAGER_H
#define THREADMANAGER_H

#include "Main/headfiles.h"
#include "ArmorDetector/armor_detector.h"
#include "CameraManager/camera_manager.h"
#include "Pose/angle_solver.h"
#include "Communication/communication.h"
#include "Timer/Timer.h"
#include "Forecast/forecast.h"
#include "RuneDetector/rune_detector.h"


class ThreadManager
{
public:
    ThreadManager();
    void Init();
    void Produce();
    void Consume();
    void Communicate();



private:
    std::unique_ptr<ArmorDetector> p_armor_detector_;  // 装甲识别
    std::unique_ptr<Forecast> p_forecast_;
   //    std::unique_ptr<RuneDetector> p_rune_dectector_;  // 大符识别
    std::unique_ptr<AngleSolver> p_angle_solver_;  // 角度解算
    std::unique_ptr<CameraManager> p_camera_manager_; //相机读图管理
    //    std::unique_ptr<Classify> p_classify_; // 机器学习
    std::unique_ptr<Communication> p_communication_; //电控通信

    std::unique_ptr<RuneDetector> p_run_detector_;//


    Mat buffer[30];
    double sys_time[30];
    int i,j;
    std::mutex mutex;
    std::condition_variable condition; //条件变量对象
    vector<Point2d> object_armor_2Dpoints_;
    double sys_now;


};


#endif // THREADMANAGER_H
