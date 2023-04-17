

#include "kalman.h"

Cv_Kalman_t::Cv_Kalman_t():
    KF(std::make_unique<cv::KalmanFilter>(Q,R))
{
    //KF->transitionMatrix = *(Mat_<float>(2, 2) << 1, 1, 0, 1);  //转移矩阵A[1,1;0,1]


    //将下面几个矩阵设置为对角阵
    setIdentity(KF->measurementMatrix);                             //测量矩阵H
    setIdentity(KF->processNoiseCov, Scalar::all(1e-5));            //系统噪声方差矩阵Q
    setIdentity(KF->measurementNoiseCov, Scalar::all(1e-1));        //测量噪声方差矩阵R
    setIdentity(KF->errorCovPost, Scalar::all(1));                  //后验错误估计协方差矩阵P

    randn(KF->statePost, Scalar::all(0), Scalar::all(0.1));          //x(0)初始化
};



/**
  * @name   kalmanCreate
  * @brief  ����һ���������˲���
  * @param  p:  �˲���
  *         T_Q:ϵͳ����Э����
  *         T_R:��������Э����
  *
  * @retval none
  * @attention 	R�̶���QԽ�󣬴���Խ���β���ֵ��Q�������ֻ�ò���ֵ
  *		       		��֮��QԽС����Խ����ģ��Ԥ��ֵ��QΪ������ֻ��ģ��Ԥ��
  */
void Kalman_t::KalmanInit(double T_Q,double T_R)
{
    X_last = (double)0;
    P_last = 0;
    Q = T_Q;
    R = T_R;
    A = 1;
    B = 0;
    H = 1;
    X_mid = X_last;
}

/**
  * @name   KalmanFilter
  * @brief  �������˲���
  * @param  p:  �˲���
  *         dat:���˲�����
  * @retval �˲��������
  * @attention Z(k)��ϵͳ����,������ֵ   X(k|k)�ǿ������˲����ֵ,���������
  *            A=1
		B=0 H=1 I=1  W(K)  V(k)�Ǹ�˹������,�����ڲ���ֵ����,���Բ��ù�
  *            �����ǿ�������5�����Ĺ�ʽ
  *            һ��H'��Ϊ������,����Ϊת�þ���
  */

double Kalman_t::KalmanFilter(double dat)
{
    X_mid =A*X_last;                     //�ٶȶ�Ӧ��ʽ(1)    x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)
    P_mid = A*P_last+Q;               //�ٶȶ�Ӧ��ʽ(2)    p(k|k-1) = A*p(k-1|k-1)*A'+Q
    kg = P_mid/(P_mid+R);             //�ٶȶ�Ӧ��ʽ(4)    kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
    X_now = X_mid+kg*(dat-X_mid);     //�ٶȶ�Ӧ��ʽ(3)    x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
    P_now = (1-kg)*P_mid;                //�ٶȶ�Ӧ��ʽ(5)    p(k|k) = (I-kg(k)*H)*P(k|k-1)
    P_last = P_now;                         //״̬����
    X_last = X_now;
    return X_now;							  //���Ԥ����x(k|k)
}
