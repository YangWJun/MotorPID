//
// Created by 30709 on 2024/5/31.
//
#include "Motor.h"
void Motor_SetSpeed(int16_t Speed)
{
    if (Speed > 0)
    {
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
if(Speed==0)
            __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, 0 );
else
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, Speed+25);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
        if(Speed==0)
                    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, 0);
        else
                __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, -Speed+25);
    } //正反转改变
}

