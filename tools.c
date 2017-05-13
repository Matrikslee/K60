#include "tools.h"

float fconstrain(float s, float l, float h) {
  if(s<l) return l;
  if(s>h) return h;
  return s;
}

void kalman_Filter(struct angle_t* ptr){
//卡尔曼滤波参数与函数
  const float dt=0.005;
  //角度数据置信度,角速度数据置信度
  const float Q_angle=0.0005, Q_gyro=0.0055; 
  const float R_angle=0.75 ,C_0 = 1; 
  static float P[2][2] = {{ 1, 0 },{ 0, 1 }};
  static float q_bias;
  static float angle, angle_dot;
  
  float Pdot[4];
  float angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
//angleAx 和 gyroGy
  angle += (ptr->angle_dot-q_bias) * dt;
  angle_err = ptr->angle - angle;
  
  Pdot[0] = Q_angle - P[0][1] - P[1][0];
  Pdot[1] = - P[1][1];
  Pdot[2] = - P[1][1];
  Pdot[3] = Q_gyro;
  
  P[0][0] += Pdot[0] * dt;
  P[0][1] += Pdot[1] * dt;
  P[1][0] += Pdot[2] * dt;
  P[1][1] += Pdot[3] * dt;
  
  PCt_0 = C_0 * P[0][0];
  PCt_1 = C_0 * P[1][0];
  E = R_angle + C_0 * PCt_0;
  K_0 = PCt_0 / E;
  K_1 = PCt_1 / E;
  t_0 = PCt_0;
  t_1 = C_0 * P[0][1];
  
  P[0][0] -= K_0 * t_0;
  P[0][1] -= K_0 * t_1;
  P[1][0] -= K_1 * t_0;
  P[1][1] -= K_1 * t_1;
  
  angle += K_0 * angle_err; //best angle
  q_bias += K_1 * angle_err;
  angle_dot = ptr->angle_dot - q_bias;//best angular velocity

  //output
  ptr->angle = angle;
  ptr->angle_dot = angle_dot;
}