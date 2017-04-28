/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,ɽ��Ƽ�
 *     All rights reserved.
 *     �������ۣ�ɽ����̳ http://www.vcan123.com
 *
 *     ��ע�������⣬�����������ݰ�Ȩ����ɽ��Ƽ����У�δ����������������ҵ��;��
 *     �޸�����ʱ���뱣��ɽ��Ƽ��İ�Ȩ������
 *
 * @file       main.c
 * @brief      ɽ��K60 ƽ̨������
 * @author     ɽ��Ƽ�
 * @version    v5.0
 * @date       2013-08-28
 */

#include "common.h"
#include "include.h"
#include "interface.h"
#include "user.h"
#include "isr.h"

void  main(void) {
  
  LED_userInit();
  IMU_userInit();
  IRQ_userInit();
  FTM_userInit();
  
  LCD_str((Site_t){0,0},"Cam init OK!",FCOLOUR,BCOLOUR);
  LCD_FStr_CH((Site_t){0,110},vcan_str,sizeof(vcan_str)/LCD_CH_SIZE,FCOLOUR,BCOLOUR);
  
  while(1){
    if(is_update) {
      status_update();
      is_update = 0;
    }
    
    //LCD_Img_Binary_Z((Site_t){0,0}, (Size_t){LCD_W,LCD_H}, imgbuff, imgsize);       //LCD��ʾ

    //vcan_sendimg(imgbuff, sizeof(imgbuff));
  }
}