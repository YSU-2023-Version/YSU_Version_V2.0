#pragma once
#ifndef RMOSS_PROJECTILE_MOTION__ITERATIVE_PROJECTILE_TOOL_HPP_
#define RMOSS_PROJECTILE_MOTION__ITERATIVE_PROJECTILE_TOOL_HPP_

#include <string>
#include <functional>

namespace rmoss_projectile_motion
{

    // ������ֵ��ͨ�����������˶�ѧ������⹤��
    class IterativeProjectileTool
    {
        // ��ˮƽ����ϵ��
        // given_x: input,�������/m
        // given_angle: input,����Ƕ�/rad
        // h: output,��������߶�/m
        // t: output,�������ʱ��/s
        typedef std::function<void(double given_angle, double given_x, double& h,
            double& t)> ForwardMotionFunc;

    public:
        IterativeProjectileTool() {}
        /**
         * @brief Set the max iteration number
         *
         * @param max_iter max iteration number
         */
        void set_max_iter(int max_iter) { max_iter_ = max_iter; }
        /**
         * @brief Set the forward motion fuction for the tool
         * ���õ���ǰ���˶����㺯��
         * ����������Ϊ: void f(ouble given_angle, double given_x, double & h, double & t)
         * ��ˮƽ����ϵ��
         * given_x: input,�������/m
         * given_angle: input,����Ƕ�/rad
         * h: output,��������߶�/m
         * t: output,�������ʱ��/s
         * @param forward_motion forward_motion function for the tool
         */
        void set_forward_motion(ForwardMotionFunc forward_motion) { forward_motion_func_ = forward_motion; }
        /**
         * @brief Solve projectile motion
         *
         * @param target_x horizontal distance of target (m)
         * @param target_h vertical distance of target (m)
         * @param angle ouput angle (rad)
         * @return true
         * @return false
         */
        bool solve(double target_x, double target_h, double& angle);
        /**
         * @brief Get error message for the tool
         *
         * @return std::string
         */
        std::string error_message() { return error_message_; }

    private:
        int max_iter_{ 20 };
        ForwardMotionFunc forward_motion_func_;
        std::string error_message_;
    };


}  // namespace rmoss_projectile_motion

#endif  // RMOSS_PROJECTILE_MOTION__ITERATIVE_PROJECTILE_TOOL_H