#include<Kalman/kalman_heyuan.h>

Forecast::Forecast(float TQ , float TR){
   T_Q = TQ;
   T_R = TR;
   Kalmaners.clear();
   for (int i = 0; i < Num; i++){
      Kalmaners.emplace_back(std::make_unique<Kalman_t>());
      Kalmaners[i]->KalmanInit(T_Q, T_R);
   }
}

void Forecast::Reset(){
   for (auto &kalman:Kalmaners){
      kalman->KalmanInit(T_Q,T_R);
      kalman->KalmanForecast();
   }
}

Point2f& Forecast::getNextForecast(){
   ForecastValue = Point2f( Kalmaners[0]->KalmanForecast(), Kalmaners[1]->KalmanForecast());
   return ForecastValue;
}

Point2f& Forecast::getReal(Point2f& observation){
   RealValue = Point2f(Kalmaners[0]->KalmanFilter(observation.x),Kalmaners[1]->KalmanFilter(observation.y));
   return RealValue;
}


void Kalman_t::KalmanInit(float& T_Q,float& T_R)
{
   X_last = 0.0f;
   P_last = 0.0f;
   Q = T_Q;
   R = T_R;
   A = 1;
   B = 0;
   H = 1;
   X_mid = X_last;
}

float Kalman_t::KalmanForecast(){
    X_mid = A*X_last;                     //�ٶȶ�Ӧ��ʽ(1)    x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)
    P_mid = A*P_last+Q;                     //�ٶȶ�Ӧ��ʽ(2)    p(k|k-1) = A*p(k-1|k-1)*A'+Q
    return X_mid;
}

float Kalman_t::KalmanFilter(float& dat)
{
   kg = P_mid/(P_mid+R);             //�ٶȶ�Ӧ��ʽ(4)    kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
   X_now = X_mid+kg*(dat-X_mid);     //�ٶȶ�Ӧ��ʽ(3)    x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
   P_now = (1-kg)*P_mid;                //�ٶȶ�Ӧ��ʽ(5)    p(k|k) = (I-kg(k)*H)*P(k|k-1)
   
   P_last = P_now;                         //״̬����
   X_last = X_now;
   return X_now;							  //���Ԥ����x(k|k)
}



