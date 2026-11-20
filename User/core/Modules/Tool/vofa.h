#ifndef VOFA_H
#define VOFA_H

#include <stdint.h>
#include <string.h>

uint8_t send_str2[sizeof(float) * 10];
void vofa_send(float x1, float x2, float x3, float x4, float x5, float x6, float x7, float x8, float x9)
{
    const float channels[9] = {x1, x2, x3, x4, x5, x6, x7, x8, x9};
    const uint8_t sendSize = sizeof(float); 

    memcpy(send_str2, channels, sendSize * 9);

    *((uint32_t*)&send_str2[sizeof(float) * 9]) = 0x7F800000; 
}

#endif
