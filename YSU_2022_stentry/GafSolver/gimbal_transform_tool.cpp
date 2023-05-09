#include "GafSolver/gimbal_transform_tool.h"
#include<iostream>
#include<cmath>
#include "GafSolver/projectile_solver_interface.h"
using namespace std;
namespace rmoss_projectile_motion
{

    bool GimbalTransformTool::solve(double x, double y, double z, double& pitch, double& yaw)
    {
        if (!solver_) {
            // if model is nullptr, use line model.
            pitch = -atan2(y,z);
        }
        else {
            double angle;
            if (solver_->solve(z, y, angle)) {
                pitch = -angle;

            }
            else {
                error_message_ = solver_->error_message();
                return false;
            }
        }
        yaw = atan2(x, z);
        return true;
    }

}  // namespace rmoss_projectile_motion
