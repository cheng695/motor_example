#ifndef TOOL_H
#define TOOL_H

#include <stdint.h>

extern uint8_t send_str2[sizeof(float) * 10];

void vofa_send(float x1, float x2, float x3, float x4, float x5, float x6, float x7, float x8, float x9);
float clamp(float value, float min, float max);

#endif
