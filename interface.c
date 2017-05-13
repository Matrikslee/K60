#include "interface.h"
#include "MK60_ADC.h"
#include "MK60_FTM.h"
#include "VCAN_LED.h"
#include "VCAN_CAMERA.h"
#include "tools.h"
#include "isr.h"

#define MAX_PWM_DUTY (10000)
#define HALF_MAX_PWM_DUTY (MAX_PWM_DUTY/2)
#define PWM_RIGHT FTM_CH5
#define PWM_LEFT  FTM_CH6

#define GYRO_X_OFFSET (0x0878)
#define GYRO_Y_OFFSET (0)
#define GYRO_Z_OFFSET (0)
#define ACCZ_X_OFFSET (0x07b0)
#define ACCZ_Y_OFFSET (0)
#define ACCZ_Z_OFFSET (0)
#define _PI (3.1415926f)
#define IMU_NUMBER (5)

// Memory to save img
static uint8 imgbuff[CAMERA_SIZE];
uint8 img_prepared;

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
  float accz;
  float gyro;

  accz = getIMUValue(ACCZ_X);
  accz = asin(fconstrain(accz,-100,100)/100.)*180/_PI;

  gyro = getIMUValue(GYRO_X);

  ga.angle = accz;
  ga.angle_dot = gyro;
  
  kalman_Filter(&ga);
}

void updateDirct(void) {
  const uint8 threshold_img = 3;

  float mid[CAMERA_H], sum;
  
  if(!img_prepared) return;
  uint8 i,j, len;
  uint8 img[CAMERA_H][CAMERA_W];

  img_extract(img, imgbuff, CAMERA_SIZE);
  
  sum = 0;

  for ( i = 20, len = 0; i < CAMERA_H; ++i ) {
    uint8 cnt = 0;
    float left, right;
    for ( j = 20; j < CAMERA_W; ++j ) {
      if(!img[i][j]) {
        if( ++cnt > threshold_img) {
          right = j+1-cnt-CAMERA_W/2.;
          break;
        }
      } else {
        cnt = 0;
      }
    }
    for ( j = 60, cnt = 0; j > 0; --j ) {
      if(!img[i][j]) {
        if( ++cnt > threshold_img) {
          left = j+1+cnt-cnt-CAMERA_W/2.;
          break;
        }        
      } else {
        cnt = 0;
      }
    }
    mid[CAMERA_H-i-1] = (left+right)/2;
    sum += mid[CAMERA_H-i-1];
  }
 
  gd.offset = sum / (CAMERA_H-20);
  
  img_prepared = 0;
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
  ftm_pwm_init(FTM0, PWM_RIGHT, MAX_PWM_DUTY, HALF_MAX_PWM_DUTY);//PTD5
  ftm_pwm_init(FTM0, PWM_LEFT, MAX_PWM_DUTY, HALF_MAX_PWM_DUTY);//PTD6
}

void CAMERA_userInit(void) {
  camera_init(imgbuff);
}

static uint8 STOP;

void motor_control(float left, float right){
  const uint32 protect_v = 100;
  const uint32 threshold = 100;
  static uint32 cnt = 0;
  
  if(fabs(left) > HALF_MAX_PWM_DUTY*0.8 || fabs(right) > HALF_MAX_PWM_DUTY*0.8) {
    if(cnt < threshold) {
      ++cnt;
    } else {
      STOP = 1;
    }
  } else {
    cnt = 0;
  }
  
  if(STOP) {
    ftm_pwm_duty(FTM0, PWM_RIGHT, HALF_MAX_PWM_DUTY);
    ftm_pwm_duty(FTM0, PWM_LEFT, HALF_MAX_PWM_DUTY);
    return;
  }
  
  left = HALF_MAX_PWM_DUTY+left;
  right = HALF_MAX_PWM_DUTY+right;

  left = fconstrain(left, protect_v, MAX_PWM_DUTY-protect_v);
  right= fconstrain(right,protect_v, MAX_PWM_DUTY-protect_v);

  ftm_pwm_duty(FTM0, PWM_RIGHT, (uint32)right);
  ftm_pwm_duty(FTM0, PWM_LEFT, (uint32)left);
}
