#ifndef OPEN_V_CONTROL
#define OPEN_V_CONTROL

#include "Ndef_Motor.h"
#include <cmath>

#define PI 3.1415926

float mach_angle=0, elec_angle=0;
float u_d=0, u_q=0, u_alpha=0, u_beta=0, u_a=0, u_b=0, u_c=0, pwm_a=0, pwm_b=0, pwm_c=0;
float timestamp=0, zero_angle=0;

// 相关参数定义

#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

// cv使用宏定义的限制函数增快运行速度 使用问号表达式

MOTOR motor = {
    12.5f, //额定电压
    7.0f,  //电机 极对数
    5.0f,  //目标速度   rad/s
    0.0f   //目标力矩
};
float a = motor.motor_pairs;
// 初始化电机结构体

float Normalize(float angle){
    float ang_i =  fmod(angle, 2*PI);
    return ang_i > 0 ? ang_i : ang_i=2*PI;
}

// 归一化取模，使大于2pi的弧度均转为之内



// 电角度求解

void setPWM(float a, float b, float c){
    
}

// PWM赋值


// 开环速度解算，Key：单次运行时间（mircos()函数）本次时间戳减去上次时间戳为单次循环时间



// Park逆变换、Clarke逆变换，得到所需的Ua、Ub、Uc，Ud暂取0




#endif