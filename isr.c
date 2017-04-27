#include "vcan_camera.h"
#include "interface.h"
#include "MK60_PIT.h"

uint8 is_update;

void PIT0_IRQHandler() {
  static uint8 cnt;

  cnt = (cnt+1)%10;

  updateAngle();
  
  if(!cnt) {
    updateSpeed();
  }
  
  if(cnt%5==0) {
    updateDirct();
  }
  
  is_update = 1;
}

void DMA0_IRQHandler() {
  camera_dma();
  camera_get_img();
}

void PORTA_IRQHandler(){
    uint8  n;    //引脚号
    uint32 flag;

    while(!PORTA_ISFR);
    flag = PORTA_ISFR;
    PORTA_ISFR  = ~0;                                   //清中断标志位

    n = 29;                                             //场中断
    if(flag & (1 << n)){                                 //PTA29触发中断
      camera_vsync();
    }
#if ( CAMERA_USE_HREF == 1 )                            //使用行中断
    n = 28;
    if(flag & (1 << n)){                                 //PTA28触发中断
      camera_href();
    }
#endif
}

void IRQ_userInit(void) {
  pit_init_ms(PIT0, 5);
  set_vector_handler(PIT0_VECTORn , PIT0_IRQHandler);
  enable_irq (PIT0_IRQn);
  
  set_vector_handler(PORTA_VECTORn , PORTA_IRQHandler);   //设置 PORTA 的中断服务函数为 PORTA_IRQHandler
  set_vector_handler(DMA0_VECTORn , DMA0_IRQHandler);     //设置 DMA0 的中断服务函数为 PORTA_IRQHandler 
}