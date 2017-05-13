#ifndef __TOOLS_H__
#define __TOOLS_H__

#include "interface.h"

#define MAXLEN 60

struct least_squares_t {
  uint32 len;
  float x[MAXLEN];
  float y[MAXLEN];
  float offset;
  float slope;
};

float fconstrain(float, float, float);
void kalman_Filter(struct angle_t* ptr);
void least_squares_fit(struct least_squares_t* p);

#endif