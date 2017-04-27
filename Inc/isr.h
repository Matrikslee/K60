#ifndef __ISR_H__
#define __ISR_H__

#include "common.h"

void IRQ_userInit(void);        //中断向量初始化函数

extern uint8 is_update;

#endif