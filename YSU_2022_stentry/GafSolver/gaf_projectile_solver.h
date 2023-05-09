#pragma once
#ifndef RMOSS_PROJECTILE_MOTION__GAF_PROJECTILE_SOLVER_HPP_
#define RMOSS_PROJECTILE_MOTION__GAF_PROJECTILE_SOLVER_HPP_

#include <string>
#include <memory>

#include "GafSolver/projectile_solver_interface.h"
#include "GafSolver/iterative_projectile_tool.h"

namespace rmoss_projectile_motion
{

	// ���������Ϳ���������gaf:Gravity and Air Frication���ĵ���������������.
	class GafProjectileSolver : public ProjectileSolverInterface
	{
	public:
		GafProjectileSolver(double initial_vel, double friction_coeff);

		void set_initial_vel(double vel) { initial_vel_ = vel; }
		void set_friction_coeff(const double& friction_coeff) { friction_coeff_ = friction_coeff; }
		std::shared_ptr<IterativeProjectileTool> get_iterative_tool() { return iterative_tool_; }
		bool solve(double target_x, double target_h, double& angle) override;
		std::string error_message() override;

	private:
		std::shared_ptr<IterativeProjectileTool> iterative_tool_;
		// �ӵ�����
		double initial_vel_;
		// ��������ϵ��
		double friction_coeff_;
	};

}  // namespace rmoss_projectile_motion

#endif  // RMOSS_PROJECTILE_MOTION__GAF_PROJECTILE_SOLVER_HPP_