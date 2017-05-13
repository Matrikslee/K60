#ifndef __INTERFACE_H__
#define __INTERFACE_H__

#include "common.h"

#define ACCZ_X (0)
#define ACCZ_Y (1)
#define ACCZ_Z (2)

#define GYRO_X (3)
#define GYRO_Y (4)

/* 角度类型封装
** angle:       滤波后的角度
** angleDot:    滤波后的角速度
*/
struct angle_t {
  float angle;
  float angle_dot;
};


/* 速度类型封装
** avg_speed:   小车左右轮转速滤波后的平均值
*/
struct speed_t{
  float now_speed;
};

/* 转向类型封装
** offset:      小车离跑道中线的偏移量
** turn_rate:   由图像得出的小车应转向的量
** front_value: 由图像和当前速度算出的提前量
*/
struct dirct_t{
  float slope;   //图像中线斜率
  float offset;  //小车离中线的偏移
};

// 初始化接口函数
void IMU_userInit(void);        //惯性单元初始化函数
void LED_userInit(void);        //LED灯初始化函数
void FTM_userInit(void);        //FTM通道初始化函数
void CAMERA_userInit(void);

void updateAngle(void);         //角度数据更新函数
void updateSpeed(void);         //速度数据更新函数
void updateDirct(void);         //方向数据更新函数

extern const struct angle_t *const pAngle;
extern const struct speed_t *const pSpeed;
extern const struct dirct_t *const pDirct;

/* 
*  @brief 电机输出函数
*  @since v1.0
*  @input 范围-5000~5000
*/
void motor_control(float left,float right);

extern uint8 img_prepared;

#endif
