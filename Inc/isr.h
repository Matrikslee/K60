#ifndef __ISR_H__
#define __ISR_H__

#include "vcan_camera.h"
#include "interface.h"
#include "MK60_PIT.h"
#include "common.h"

void IRQ_userInit(void);        //�ж�������ʼ������

extern uint8 is_pit_updated;

#endif