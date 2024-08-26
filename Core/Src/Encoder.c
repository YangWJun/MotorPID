//
// Created by 30709 on 2024/5/31.
//
#include "Encoder.h"

int16_t Encoder_Get(void)
{
    int16_t Temp;
    Temp = __HAL_TIM_GET_COUNTER(&htim1);
    __HAL_TIM_SET_COUNTER(&htim1,0);
    return Temp;
}  //获取编码器的值


