#include "GafSolver/iterative_projectile_tool.h"
#include "GafSolver/gimbal_transform_tool.h"
#include "GafSolver/gaf_projectile_solver.h"
#include <iostream>
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;
int main() {
	auto gaf_solver = std::make_shared<rmoss_projectile_motion::GafProjectileSolver>(15, 0.01);
	auto projectile_tansformoss_tool = std::make_shared<rmoss_projectile_motion::GimbalTransformTool>(gaf_solver);
	double x =0.7, y =4.7, z = 9.1;//��λΪ��
	double pitch = -atan2(y, z) * 180 / CV_PI;
	double yaw=atan2(x,z)*180/CV_PI;
	cout << "pitch_before:" << pitch << endl;
	cout << "yaw_before:" << yaw << endl;
	double pitch2 = 0, yaw2 = 0;
	projectile_tansformoss_tool->solve(x, y, z, pitch2, yaw2);
	cout << "pitch:" << pitch2*180/CV_PI << endl;
	cout << "yaw:" << yaw2*180/CV_PI << endl;
}