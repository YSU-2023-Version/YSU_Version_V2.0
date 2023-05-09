#pragma once
#ifndef RMOSS_PROJECTILE_MOTION__GIMBAL_TRANSFORM_TOOL_HPP_
#define RMOSS_PROJECTILE_MOTION__GIMBAL_TRANSFORM_TOOL_HPP_

#include <memory>
#include <string>
#include "GafSolver/projectile_solver_interface.h"

namespace rmoss_projectile_motion
{

    class GimbalTransformTool
    {
    public:
        /**
         * @brief Construct a new Gimbal Transform Tool
         *
         * @param solver Projectile solver for transform tool
         */
        explicit GimbalTransformTool(std::shared_ptr<rmoss_projectile_motion::ProjectileSolverInterface> solver = nullptr)
            : solver_(solver) {}

        /**
         * @brief Set the projectile solver
         *
         * @param solver Projectile solver for transform tool
         */
        void set_projectile_solver(std::shared_ptr<rmoss_projectile_motion::ProjectileSolverInterface> solver) { solver_ = solver; }
        /**
         * @brief Get error message
         *
         * @return std::string
         */
        std::string error_message() { return error_message_; }
        /**
         * @brief Solve gimbal angle
         * input, position of target object in *gimbal frame*
         * output, angle of gimbal(pitch, yaw)
         * @param x (m)
         * @param y (m)
         * @param z (m)
         * @param pitch (rad)
         * @param yaw (rad)
         * @return true
         * @return false
         */
        bool solve(double x, double y, double z, double& pitch, double& yaw);
        /*bool solve(::Vector3d position, double& pitch, double& yaw)
        {
            return solve(position(0), position(1), position(2), pitch, yaw);
        }

        bool solve(geometry_msgs::msg::Point position, double& pitch, double& yaw)
        {
            return solve(position.x, position.y, position.z, pitch, yaw);
        }*/

    private:
        std::shared_ptr<ProjectileSolverInterface> solver_;
        std::string error_message_;
    };

}  // namespace rmoss_projectile_motion

#endif  // RMOSS_PROJECTILE_MOTION__GIMBAL_TRANSFORM_TOOL_HPP_