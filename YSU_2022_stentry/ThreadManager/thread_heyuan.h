#include "../Main/headfiles.h"
#include "ArmorDetector/armor_detector.h"
#include "CameraManager/camera_manager.h"
#include "Pose/angle_solver.h"
#include "Communication/communication.h"
#include "Timer/Timer.h"
#include "Kalman/kalman_heyuan.h"
#include "RuneDetector/rune_detector.h"
#include "GafSolver/iterative_projectile_tool.h"
#include "GafSolver/gimbal_transform_tool.h"
#include "GafSolver/gaf_projectile_solver.h"


#include <mutex>
#include <cstdlib>

#define BUFFER_LENGTH 5


class threadManager{
public:
    threadManager();
    void Produce();
    void Consume();
    void Communicate();

private:
    void subConsume();

    std::unique_ptr<ArmorDetector> p_armor_detector_;        // 装甲识别
    std::unique_ptr<Forecast> p_forecast_;
   //    std::unique_ptr<RuneDetector> p_rune_dectector_;    // 大符识别
    std::unique_ptr<AngleSolver> p_angle_solver_;            // 角度解算
    std::unique_ptr<CameraManager> p_camera_manager_;        // 相机读图管理
    std::unique_ptr<Communication> p_communication_;         //电控通信
    std::unique_ptr<RuneDetector> p_run_detector_;           

    int Communit_FPS_;
    int Camera_FPS_;
    int Vision_FPS_;
    cv::Mat buffer_mat[BUFFER_LENGTH];
    std::mutex locker[BUFFER_LENGTH];
    double sys_time[BUFFER_LENGTH];

    std::vector<cv::Point2f> buffer_points[BUFFER_LENGTH];
    std::mutex locker_point[BUFFER_LENGTH];


    double y_p_recv[BUFFER_LENGTH][4];//pit_angle;pit_speed;yaw_angle;yaw_speed;(time)
    int i,j,j2;
    

};
