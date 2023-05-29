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
#include "GafSolver/iterative_projectile_tool.h"
#include "GafSolver/gimbal_transform_tool.h"
#include "GafSolver/gaf_projectile_solver.h"

#define BUFFER_LENGTH 5

class ThreadManager
{
public:
    ThreadManager();
    void Init();
    void Produce();
    void Consume();
    void Communicate();



private:
    void subConsume();

    void InitThreadManager();


private:
    int FPS_count_;                                          // 帧数
    int base_time_;                                          // 稳定的时间
    std::unique_ptr<ArmorDetector> p_armor_detector_;        // 装甲识别
    std::unique_ptr<Forecast> p_forecast_;
   //    std::unique_ptr<RuneDetector> p_rune_dectector_;    // 大符识别
    std::unique_ptr<AngleSolver> p_angle_solver_;            // 角度解算
    std::unique_ptr<CameraManager> p_camera_manager_;        // 相机读图管理
    //    std::unique_ptr<Classify> p_classify_;             // 机器学习 弃用
    std::unique_ptr<Communication> p_communication_;         //电控通信

    std::unique_ptr<RuneDetector> p_run_detector_;           //

    double y_p_recv[30][4];//pit_angle;pit_speed;yaw_angle;yaw_speed;(time)
    double sys_time[30];
    std::mutex mutex;
    std::condition_variable condition; //条件变量对象
    vector<Point2d> object_armor_2Dpoints_;
    double sys_now;


    int Communit_FPS_;
    int Camera_FPS_;
    int Vision_FPS_;
    cv::Mat buffer_mat[BUFFER_LENGTH];
    std::mutex locker[BUFFER_LENGTH];

    std::vector<cv::Point2f> buffer_points[BUFFER_LENGTH];
    std::mutex locker_point[BUFFER_LENGTH];

    int i,j,j2;


};


#endif // THREADMANAGER_H
