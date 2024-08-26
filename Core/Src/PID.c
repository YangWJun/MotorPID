
#include "PID.h"


float Bias=0,Pwm_S=0,Last_bias=0,Prev_bias=0;
float  Last_Error=0, Pwm_integral,Pwm_P,Pwm_D;
/*单速度环*/
 float C_Speed_Kp=13,C_Speed_Ki=1.2,C_Speed_Kd=13;
 /*串级PID*/
 float Speed_Kp=13,Speed_Ki=1.2,Speed_Kd=8;   //速度环参数
float Position_Kp = 0.36,  Position_Kd =2.2,Position_Ki=0;   //位置环参数
/*单位置环*/
float F_Position_Kp = 0.6,  F_Position_Kd =0.5,F_Position_Ki=0.02;   //位置环参数
float PositionControl(int16_t Speed,int16_t Target) {
     float Error = Target - Speed;
     //微分项
     Pwm_D = Error - Last_Error;
     //积分项
     Pwm_integral += Error;
     // 积分限幅
     if (Pwm_integral > 600)
         Pwm_integral = 600;
     if (Pwm_integral < -600)
         Pwm_integral = -600;
//输出PWM计算

         Pwm_P = Position_Kp * Error +
                 Position_Ki * Pwm_integral +
                 Position_Kd * Pwm_D;
         Last_Error = Error; //误差传递

         return Pwm_P;
       //位置环（位置式PID）
 }

float Follow_PositionControl(int16_t Speed,int16_t Target) {
    float Error = Target - Speed;
    //微分项
    Pwm_D = Error - Last_Error;
    //积分项
    Pwm_integral += Error;
    // 积分限幅
    if (Pwm_integral > 1000)
        Pwm_integral = 1000;
    if (Pwm_integral < -1000)
        Pwm_integral = -1000;
//输出PWM计算

    Pwm_P = F_Position_Kp * Error +
            F_Position_Ki * Pwm_integral +
            F_Position_Kd * Pwm_D;
    Last_Error = Error; //误差传递

    return Pwm_P;
    //位置环（位置式PID）
}
float SpeedControl(int16_t  Speed,int16_t Target)
{

    Bias=Target-Speed;
    //误差计算
    Pwm_S += (Speed_Kp*(Bias-Last_bias))
           +(Speed_Ki*Bias)
           +(Speed_Kd*(Bias-2*Last_bias+Prev_bias));

    Prev_bias=Last_bias;                                 //误差传递
    Last_bias=Bias;	                                   //误差传递
     //输出限幅
    return Pwm_S;
}  //速度环（增量式PID）
float Const_SpeedControl(int16_t  Speed,int16_t Target)
{

    Bias=Target-Speed;
    //误差计算
    Pwm_S += (Speed_Kp*(Bias-Last_bias))
             +(Speed_Ki*Bias)
             +(Speed_Kd*(Bias-2*Last_bias+Prev_bias));

    Prev_bias=Last_bias;                                 //误差传递
    Last_bias=Bias;	                                   //误差传递
    //输出限幅
    return Pwm_S;
}  //速度环（增量式PID）

float PWM_Limit(float Reailty,float target)
{
    if(Reailty<-target)
        Reailty=-target;
    if(Reailty> target)
        Reailty= target;
    return Reailty;
 }  //PID输出限幅；