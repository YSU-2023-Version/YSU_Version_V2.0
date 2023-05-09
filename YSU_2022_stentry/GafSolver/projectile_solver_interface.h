#pragma once
#ifndef RMOSS_PROJECTILE_MOTION__PROJECTILE_SOLVER_INTERFACE_HPP_
#define RMOSS_PROJECTILE_MOTION__PROJECTILE_SOLVER_INTERFACE_HPP_

#include <string>

namespace rmoss_projectile_motion
{

	class ProjectileSolverInterface
	{
	public:
		// return true if transform successfully.
		virtual bool solve(double target_x, double target_h, double& angle) = 0;
		// get error message when solve() return false.
		virtual std::string error_message() = 0;
	};

}  // namespace rmoss_projectile_motion

#endif  // RMOSS_PROJECTILE_MOTION__PROJECTILE_SOLVER_INTERFACE_HPP_