/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2016,Bluewhale
 *     All rights reserved.
 *     http://www.lyeec.me
 *     http://www.hsmouc.com
 *
 *     ��ע�������⣬�����������ݰ�Ȩ����Bluewhale���У�δ����������������ҵ��;��
 *     �޸�����ʱ���뱣��Bluewhale�İ�Ȩ������
 *
 * @file       user.c
 * @brief      bluewhale�������źŲɼ������ݴ���
 * @author     
 * @version    v1.0
 * @date       2016-12-12
 */
#include "include.h"
#include "interface.h"
#include "common.h"
#include "tools.h"

float balance_fit(float set_angle) {
  const float Kp = 1200;
  const float Kd = 20;
  
  return Kp*(pAngle->angle - set_angle) + Kd*pAngle->angle_dot;
}

float speed_fit(float set_speed) {
  const float threshold = 10;
  const float Kp = 100;
  const float Ki = 2;
  static float integration;
  
  float error;
  float output;
  
  error = set_speed - pSpeed->now_speed;
  
  output = Kp*error;
  
  //���ַ����PID
  if(fabs(error) > threshold) {
    integration += error;
    output += Ki * integration;
  }
  
  // ? �����Ƿ��΢����,�����,Ӧ����ʲô����΢����
  return output;
}

float turn_fit() {
  const float Kp = 20;
  const float Kd = 100;
  
  return Kp*pDirct->offset + Kd*pDirct->slope; // ������μ�����ǰ��
}

void status_update(void) {
  float set_angle = 5.6;
  float set_speed = 10;
  float pwm1, pwm2, turn_need;
  
  pwm1 = balance_fit(set_angle);
  pwm2 = speed_fit(set_speed);
  turn_need = turn_fit();
  
  motor_control(pwm1-pwm2+turn_need, pwm1-pwm2-turn_need);
}
