#include "MK60_ADC.h"
#include "MK60_FTM.h"
#include "VCAN_LED.h"
#include "VCAN_CAMERA.h"
#include "common.h"
#include "tools.h"
#include "isr.h"
#include "interface.h"

#define MAX_PWM_DUTY (10000)
#define HALF_MAX_PWM_DUTY (MAX_PWM_DUTY/2)
#define PWM_RIGHT FTM_CH5
#define PWM_LEFT  FTM_CH6

#define GYRO_X_OFFSET (0x085B)
#define GYRO_Y_OFFSET (0)
#define GYRO_Z_OFFSET (0)
#define ACCZ_X_OFFSET (0x07C0)
#define ACCZ_Y_OFFSET (0)
#define ACCZ_Z_OFFSET (0)
#define _PI (3.1415926f)
#define IMU_NUMBER (5)

// Memory to save img
static uint8 imgbuff[CAMERA_SIZE];
static uint8 img[CAMERA_H][CAMERA_W];

static const ADCn_Ch_e ADC_Check_Maps[] = {
  ADC1_SE4a,    //ACCZ_X
  ADC1_SE5a,    //ACCZ_Y
  ADC0_SE17,    //ACCZ_Z
  ADC1_SE6a,    //GYRO1
  ADC1_SE7a,    //GYRO2
};

// IMU configurations
static float imu_ratio[IMU_NUMBER] = {0.086,0.086,0,0.23578,0.120248};
static uint32 imu_offset[IMU_NUMBER] = {ACCZ_X_OFFSET,ACCZ_Y_OFFSET,ACCZ_Z_OFFSET,GYRO_X_OFFSET, GYRO_Y_OFFSET};

static struct angle_t ga;
static struct speed_t gs;
static struct dirct_t gd;

const struct angle_t *const pAngle = &ga;
const struct speed_t *const pSpeed = &gs;
const struct dirct_t *const pDirct = &gd;


void IMU_userInit(void) {
  int i;
  for(i = 0; i < IMU_NUMBER; i++){
    adc_init(ADC_Check_Maps[i]);
  }
}

float getIMUValue(uint8 index) {
  int16 tmp_value;
  tmp_value = adc_once(ADC_Check_Maps[index], ADC_16bit)>>4;
  return ((float)tmp_value-imu_offset[index])*imu_ratio[index];
}

void updateAngle(void) {
  //const float dt = 0.005;
  //const float beta = 0.99611;
  //static float angle;
  float accz;
  float gyro;

  accz = getIMUValue(ACCZ_X);
  accz = asin(fconstrain(accz,-100,100)/100.)*180/_PI;

  gyro = getIMUValue(GYRO_X);

  //angle = beta*(angle+gyro*dt)+(1-beta)*accz;

  ga.angle = accz;
  ga.angle_dot = gyro;

  kalman_Filter(&ga);
}

void updateDirct(void) {
  int i,j;
  float errs[CAMERA_H];
  float avg_x, avg_y;
  float num, den;
  float a,b;

  img_extract(img, imgbuff, CAMERA_SIZE);
  avg_x = avg_y = 0;
  for ( i = 0; i < CAMERA_H; ++ i ){
    avg_x += i;
    float tmp = 0;
    for ( j = 0; j < CAMERA_W; ++ j ) {
      if(img[i][j]) {
        tmp += j-(CAMERA_W-1)/2.;
      }
    }
    //此处应该将图像处理结果翻转过来
    errs[CAMERA_H-1-i] = tmp/CAMERA_W;
    avg_y += errs[CAMERA_H-1-i];
  }
  avg_x /= CAMERA_H;
  avg_y /= CAMERA_H;

  num = den = 0;
  for ( i = 0; i < CAMERA_H; ++ i ){
    num += (i-avg_x)*(errs[i]-avg_y);
    den += (i-avg_x)*(i-avg_x);
  }
  a = (num/den)/CAMERA_H;
  b = avg_y - a*avg_x;

  gd.slope = a;
  gd.offset = b;
  gd.advance = 0;
}

void updateSpeed(void) {
  const float beta = 0.3;
  static float filt;
  float lcnt, rcnt, avg;

  rcnt = -ftm_quad_get(FTM1);
  ftm_quad_clean(FTM1);

  lcnt = ftm_quad_get(FTM2);
  ftm_quad_clean(FTM2);

  avg = (lcnt+rcnt)/2.0f;

  filt = filt*beta + avg*(1-beta);

  gs.now_speed = filt;
}

void LED_userInit(void){
  led_init (LED2);
  led_init (LED1);
  led_init (LED0);
  led (LED0,LED_OFF);
  led (LED1,LED_OFF);
  led (LED2,LED_ON);
}

void FTM_userInit(void){
  ftm_quad_init(FTM1);     //PTA12计数 PTA13方向
  ftm_quad_init(FTM2);     //PTA10计数 PTA11方向
  ftm_pwm_init(FTM0, FTM_CH5, MAX_PWM_DUTY, HALF_MAX_PWM_DUTY);//PTD5
  ftm_pwm_init(FTM0, FTM_CH6, MAX_PWM_DUTY, HALF_MAX_PWM_DUTY);//PTD6
}

void CAMERA_userInit(void) {
  camera_init(imgbuff);
}

void motor_control(float left, float right){
  const float dead_zone = 100;
  left = HALF_MAX_PWM_DUTY+left;
  right = HALF_MAX_PWM_DUTY+right;

  // 考虑死区的输出限幅
  left = fconstrain(left, dead_zone, MAX_PWM_DUTY-dead_zone);
  right= fconstrain(right,dead_zone, MAX_PWM_DUTY-dead_zone);

  ftm_pwm_duty(FTM0, PWM_RIGHT, (uint32)right);
  ftm_pwm_duty(FTM0, PWM_LEFT, (uint32)left);
}
