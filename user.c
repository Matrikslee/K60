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
#include "user.h"
#include "include.h"
#include "interface.h"
#include "common.h"
#include "tools.h"

float balance_fit(float set_angle) {
  const float Kp = 1100;
  const float Kd = 18;
  
  return Kp*(pAngle->angle - set_angle) + Kd*pAngle->angle_dot;
}

float speed_fit(float set_speed) {
  const float Ilimit = 100;
  const float threshold = 10;
  const float Kp = 10;
  const float Ki = 2;
  static float integration;
  
  float error;
  float output;
  
  error = set_speed - pSpeed->now_speed;
  
  output = Kp*error;
  
  //���ַ����PID
  if(fabs(error) > threshold) {
    integration += error;
    integration = fconstrain(integration, -Ilimit, Ilimit);
    output += Ki * integration;
  }
  
  return output;
}

float turn_fit() {
  const float Kp = 15;
  const float Kd = 120;
  
  static float last_err;
  
  float error;
  
  error = pDirct->offset;
  
  return Kp*error + Kd*(last_err-error);
}

void status_update(void) {
  static uint8 cnt;
  float set_angle = 1.5;
  float set_speed = 10;
  float balance, speed, turn;
    
  balance = balance_fit(set_angle);
  speed = speed_fit(set_speed);
  turn = turn_fit();
  
  if(cnt < 200) {
    ++cnt;
    speed = turn = 0;
  }
  
  motor_control(balance-speed+turn, balance-speed-turn);
}
