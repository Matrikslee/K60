#ifndef __INTERFACE_H__
#define __INTERFACE_H__

#include "common.h"

#define ACCZ_X (0)
#define ACCZ_Y (1)
#define ACCZ_Z (2)

#define GYRO_X (3)
#define GYRO_Y (4)

/* �Ƕ����ͷ�װ
** angle:       �˲���ĽǶ�
** angleDot:    �˲���Ľ��ٶ�
*/
struct angle_t {
  float angle;
  float angle_dot;
};


/* �ٶ����ͷ�װ
** avg_speed:   С��������ת���˲����ƽ��ֵ
*/
struct speed_t{
  float now_speed;
};

/* ת�����ͷ�װ
** offset:      С�����ܵ����ߵ�ƫ����
** turn_rate:   ��ͼ��ó���С��Ӧת�����
** front_value: ��ͼ��͵�ǰ�ٶ��������ǰ��
*/
struct dirct_t{
  float slope;   //ͼ������б��
  float offset;  //С�������ߵ�ƫ��
};

// ��ʼ���ӿں���
void IMU_userInit(void);        //���Ե�Ԫ��ʼ������
void LED_userInit(void);        //LED�Ƴ�ʼ������
void FTM_userInit(void);        //FTMͨ����ʼ������
void CAMERA_userInit(void);

void updateAngle(void);         //�Ƕ����ݸ��º���
void updateSpeed(void);         //�ٶ����ݸ��º���
void updateDirct(void);         //�������ݸ��º���

extern const struct angle_t *const pAngle;
extern const struct speed_t *const pSpeed;
extern const struct dirct_t *const pDirct;

/* 
*  @brief ����������
*  @since v1.0
*  @input ��Χ-5000~5000
*/
void motor_control(float left,float right);

extern uint8 img_prepared;

#endif
