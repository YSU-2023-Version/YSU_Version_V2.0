#ifndef ANGLESOLVER_H
#define ANGLESOLVER_H

#include "Main/headfiles.h"
#include "GafSolver/iterative_projectile_tool.h"
#include "GafSolver/gimbal_transform_tool.h"
#include "GafSolver/gaf_projectile_solver.h"

#define GRACOMP_DIS  7500 //gravity compensation velocity threshold
#define BULLETFIRE_V 1000 //bullet firing velocity
#define CAM_SUPPORT_DIS 2450 //238mm
class AngleSolver
{
public:
    AngleSolver();
    void InitAngle();
    double * SolveAngle(vector<Point2f>& object_armor_points_,double p_y_recv[4]);
    int shoot_get();

    vector<cv::Point3f> obj;
    cv::Mat rVec;
    cv::Mat tVec;
    cv:: Mat cam;
    cv:: Mat disCoeffD;
    int shoot=0;
    float gra_t;
    double _xErr;
    double _yErr;
    double distance_3d;
    double p_y_err[2];//0:yaw,1:pitch
    void P4P_solver();
    void gravity_comp();

private:
    float y_yaw;
    float x_pitch;

    std::shared_ptr<rmoss_projectile_motion::GafProjectileSolver> gaf_solver; // 重力补偿
    std::shared_ptr<rmoss_projectile_motion::GimbalTransformTool> projectile_tansformoss_tool; // 重力补偿
    
};

#endif // ANGLESOLVER_H
