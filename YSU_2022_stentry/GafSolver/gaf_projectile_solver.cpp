#include "GafSolver/gaf_projectile_solver.h"

#include <cmath>
#include <string>
#include <memory>

const double GRAVITY = 9.7913;

namespace rmoss_projectile_motion
{

    GafProjectileSolver::GafProjectileSolver(double initial_vel, double friction_coeff)
        : initial_vel_(initial_vel), friction_coeff_(friction_coeff)
    {
        // forward motion model: gravity and air friction model
        // air friction of x-axis is considered（仅下降阶段考虑）
        auto forward_motion = [&](double given_angle, double given_x, double& h, double& t) {
            double& v = initial_vel_;
            if (given_angle > 0.01) {      // 存在上升阶段
                double t0, x0, y0;      // 上升阶段（最高点）
                t0 = v * sin(given_angle) / GRAVITY;
                x0 = v * cos(given_angle) * t0;
                y0 = GRAVITY * t0 * t0 / 2;
                if (given_x < x0) {      // 只有上升阶段,退化成抛物线模型
                    t = given_x / (v * cos(given_angle));
                    h = v * sin(given_angle) * t - GRAVITY * t * t / 2;
                }
                else {               // 先上升,后下降
                    double t1, x1;        // 下降阶段
                    x1 = given_x - x0;
                    t1 = (exp(friction_coeff_ * x1) - 1) /
                        (friction_coeff_ * v * cos(given_angle));
                    t = t0 + t1;
                    h = y0 - GRAVITY * t1 * t1 / 2;
                }
            }
            else {
                // 只有下降
                t = (exp(friction_coeff_ * given_x) - 1) /
                    (friction_coeff_ * v * cos(given_angle));        // 下降阶段时间
                h = v * sin(given_angle) * t - GRAVITY * t * t / 2;
            }
        };
        // configure iterative tool
        iterative_tool_ = std::make_shared<IterativeProjectileTool>();
        iterative_tool_->set_forward_motion(forward_motion);
        iterative_tool_->set_max_iter(100);
    }

    bool GafProjectileSolver::solve(double target_x, double target_h, double& angle)
    {
        return iterative_tool_->solve(target_x, target_h, angle);
    }

    std::string GafProjectileSolver::error_message()
    {
        return iterative_tool_->error_message();
    }

}  // namespace rmoss_projectile_motion