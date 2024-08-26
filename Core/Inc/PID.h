//
// Created by 30709 on 2024/5/31.
//

#ifndef TEST1_PID_H
#define TEST1_PID_H
#include "main.h"
 float PositionControl(int16_t Speed,int16_t Target);
float SpeedControl(int16_t  Speed,int16_t Target);
float Follow_PositionControl(int16_t Speed,int16_t Target);
float Const_SpeedControl(int16_t  Speed,int16_t Target);
float PWM_Limit(float Reailty,float target);
#endif
