/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,山外科技
 *     All rights reserved.
 *     技术讨论：山外论坛 http://www.vcan123.com
 *
 *     除注明出处外，以下所有内容版权均属山外科技所有，未经允许，不得用于商业用途，
 *     修改内容时必须保留山外科技的版权声明。
 *
 * @file       main.c
 * @brief      山外K60 平台主程序
 * @author     山外科技
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
    
    //LCD_Img_Binary_Z((Site_t){0,0}, (Size_t){LCD_W,LCD_H}, imgbuff, imgsize);       //LCD显示

    //vcan_sendimg(imgbuff, sizeof(imgbuff));
  }
}