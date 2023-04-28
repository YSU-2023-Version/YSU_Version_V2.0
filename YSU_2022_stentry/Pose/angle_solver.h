#ifndef ANGLESOLVER_H
#define ANGLESOLVER_H

#include "Main/headfiles.h"
#define GRACOMP_DIS  7500 //gravity compensation velocity threshold
#define BULLETFIRE_V 1000 //bullet firing velocity
#define CAM_SUPPORT_DIS 2450 //238mm
class AngleSolver
{
public:
    AngleSolver();
    void InitAngle();
    double * SolveAngle(vector<cv::Point2d>& object_armor_points_);
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
    double p_y_err[2];
    void P4P_solver();
    void gravity_comp();
private:
    float y_yaw;
    float x_pitch;

};

#endif // ANGLESOLVER_H
