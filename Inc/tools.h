#ifndef __TOOLS_H__
#define __TOOLS_H__

#include "interface.h"

float fconstrain(float, float, float);
void kalman_Filter(struct angle_t* ptr);

#endif