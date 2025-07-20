#ifndef __CONTROL_H
#define __CONTROL_H

#include "stm32f10x.h"

// PID结构体定义
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    
    float Error;
    float Last_Error;
    float Integral;
    
    float Output;
} PID;

void PID_Init(float Kp, float Ki, float Kd);
float Position_PID_FindLine(float target_error);

#endif
