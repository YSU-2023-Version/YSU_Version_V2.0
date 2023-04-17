#include "Main/headfiles.h"
#include "Pose/angle_solver.h"



AngleSolver::AngleSolver()
{

}

void AngleSolver::InitAngle()
{
         cam=Mat(3,3,CV_32FC1,Scalar::all(0));
         disCoeffD=Mat(1,5,CV_32FC1,Scalar::all(0));

         string file_path="../xml_path/camera.xml";
         FileStorage fr;
         fr.open(file_path,FileStorage::READ);
         while(!fr.isOpened()){
               cout<<"camera xml floading failed..."<<endl;
              fr=FileStorage(file_path,FileStorage::READ);
         }
         fr["camera-matrix"]>>cam;
         fr["distortion"]>>disCoeffD;
         fr.release();

       obj = vector<Point3f>{
            cv::Point3f(-61.5f,-30.0f,0),
            cv::Point3f(61.5f,-30.0f,0),
            cv::Point3f(61.5f,30.0f,0),
            cv::Point3f(-61.5f,30.0f,0)
       };
       rVec = cv::Mat::zeros(3,1,CV_64FC1);
       tVec = cv::Mat::zeros(3,1,CV_64FC1);
}

double * AngleSolver::SolveAngle(vector<Point2d>& object_armor_points_)
{
    if(!(object_armor_points_[0]==Point2d(0,0)&&object_armor_points_[1]==Point2d(0,0)&&object_armor_points_[2]==Point2d(0,0)&&object_armor_points_[3]==Point2d(0,0)))
    {
    cv::solvePnP(obj,object_armor_points_,cam,disCoeffD,rVec,tVec,false,SOLVEPNP_ITERATIVE);
    _xErr = atan(tVec.at<double>(0, 0) / tVec.at<double>(2, 0)) / 2 / CV_PI * 360;
    _yErr = atan(tVec.at<double>(1, 0) / tVec.at<double>(2, 0)) / 2 / CV_PI * 360;

    //cout<<"_xErr:"<<_xErr<<endl;
    //cout<<"_yErr:"<<_yErr<<endl;
    gravity_comp();
    double x_pos=tVec.at<double>(0,0);
    double y_pos=tVec.at<double>(1,0);
    double z_pos=tVec.at<double>(2,0);
    distance_3d=sqrt(x_pos*x_pos+y_pos*y_pos+z_pos*z_pos);
    cout<<"distance= "<<distance_3d<<endl;
    shoot=1;
    }
    else
    {
        p_y_err[0] = 0;
        p_y_err[1] = 0;//top_min=-19
        shoot=0;
    }

    return p_y_err;
}
int AngleSolver::shoot_get()
{
   // cout<<"shoot:"<<shoot<<endl;
    return shoot;
}

void AngleSolver::P4P_solver()
{
  double x_pos = tVec.at<double>(0, 0);
  double y_pos = tVec.at<double>(1, 0);
  double z_pos = tVec.at<double>(2, 0);

  double tan_pitch = y_pos / sqrt(x_pos*x_pos + z_pos * z_pos);
  double tan_yaw = x_pos / z_pos;
  x_pitch = -atan(tan_pitch) * 180 / CV_PI;
  y_yaw = atan(tan_yaw) * 180 / CV_PI;

  cout<<"x_pitch"<<x_pitch<<endl;
  cout<<"y_yaw"<<y_yaw<<endl;
}

void AngleSolver::gravity_comp()
{
    gra_t=distance_3d/BULLETFIRE_V;
    if(distance_3d>GRACOMP_DIS)
    {
        p_y_err[0] = _xErr;
//       p_y_err[1] = _yErr;
       p_y_err[1] = _yErr-(CAM_SUPPORT_DIS+4.9*gra_t*gra_t)*0.001;
        //p_y_err[1] = _yErr-(BULLETFIRE_V*(distance_3d/BULLETFIRE_V)*(CAM_SUPPORT_DIS/distance_3d)+0.5*9.8*gra_t*gra_t);
    }
    else
    {
        p_y_err[0] = _xErr;
        p_y_err[1] = _yErr;
    }
}
