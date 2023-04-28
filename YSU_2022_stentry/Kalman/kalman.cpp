#include "kalman.h"

Kalman::Kalman():
    KF(make_unique<KalmanFilter>(stateNum, measureNum, 0))
{}


void Kalman::Kalman_init(double Q,double R)
/*void Kalman_init(KalmanFilter* KF,double Q,double R)*/{
        KF->transitionMatrix = (Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);  //转移矩阵A
        //setIdentity是创建单位矩阵的，后面不带Scalar参数表示默认创建单位矩阵
        setIdentity(KF->measurementMatrix);                                             //测量矩阵H
        setIdentity(KF->processNoiseCov, Scalar::all(Q)); //0.00001                   //系统噪声方差矩阵Q
        setIdentity(KF->measurementNoiseCov, Scalar::all(R)); //0.1                   //测量噪声方差矩阵R
        setIdentity(KF->errorCovPost, Scalar::all(1));                                  //后验错误估计协方差矩阵P


}

Point Kalman::Kalman_filter(Point measure_point)
/*Point Kalman_filter(KalmanFilter* KF, Point measure_point)*/{
        //2.kalman prediction
        Mat measurement = Mat::zeros(measureNum, 1, CV_32F);                           //初始测量值x'(0)，因为后面要更新这个值，所以必须先定义
        Mat prediction =KF->predict();
        Point predict_pt = Point(prediction.at<float>(0), prediction.at<float>(1));   //预测值(x',y')
        //3.update measurement
        measurement.at<float>(0) = (float)measure_point.x;
        measurement.at<float>(1) = (float)measure_point.y;
        //4.update
        KF->correct(measurement);
        return  predict_pt;
}


//
//  * @name   kalmanCreate
//  * @brief  ����һ���������˲���
//  * @param  p:  �˲���
//  *         T_Q:ϵͳ����Э����
//  *         T_R:��������Э����
//  *
//  * @retval none
//  * @attention 	R�̶���QԽ�󣬴���Խ���β���ֵ��Q�������ֻ�ò���ֵ
//  *		       		��֮��QԽС����Խ����ģ��Ԥ��ֵ��QΪ������ֻ��ģ��Ԥ��
//  */
//void Kalman_t::KalmanInit(double T_Q,double T_R)
//{
//    X_last = (double)0;
//    P_last = 0;
//    Q = T_Q;
//    R = T_R;
//    A = 1;
//    B = 0;
//    H = 1;
//    X_mid = X_last;
//}


//  * @name   KalmanFilter
//  * @brief  �������˲���
//  * @param  p:  �˲���
//  *         dat:���˲�����
//  * @retval �˲��������
//  * @attention Z(k)��ϵͳ����,������ֵ   X(k|k)�ǿ������˲����ֵ,���������
//  *            A=1
//        B=0 H=1 I=1  W(K)  V(k)�Ǹ�˹������,�����ڲ���ֵ����,���Բ��ù�
//  *            �����ǿ�������5�����Ĺ�ʽ
//  *            һ��H'��Ϊ������,����Ϊת�þ���
//  */

//double Kalman_t::KalmanFilter(double dat)
//{
//    X_mid =A*X_last;                     //�ٶȶ�Ӧ��ʽ(1)    x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)
//    P_mid = A*P_last+Q;               //�ٶȶ�Ӧ��ʽ(2)    p(k|k-1) = A*p(k-1|k-1)*A'+Q
//    kg = P_mid/(P_mid+R);             //�ٶȶ�Ӧ��ʽ(4)    kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
//    X_now = X_mid+kg*(dat-X_mid);     //�ٶȶ�Ӧ��ʽ(3)    x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
//    P_now = (1-kg)*P_mid;                //�ٶȶ�Ӧ��ʽ(5)    p(k|k) = (I-kg(k)*H)*P(k|k-1)
//    P_last = P_now;                         //״̬����
//    X_last = X_now;
//    return X_now;							  //���Ԥ����x(k|k)
//}
