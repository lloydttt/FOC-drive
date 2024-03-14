#ifndef OPEN_V_CONTROL
#define OPEN_V_CONTROL

#include "Ndef_Motor.h"


// 相关参数定义



// cv使用宏定义的限制函数增快运行速度 使用问号表达式

MOTOR motor = {
    12.5f,
    7.0f,
    5.0f,
    0.0f
};
float a = motor.motor_pairs;
// 初始化电机结构体



// 归一化取模，使大于2pi的弧度均转为之内



// 电角度求解


// PWM赋值


// 开环速度解算，Key：单次运行时间（mircos()函数）本次时间戳减去上次时间戳为单次循环时间



// Park逆变换、Clarke逆变换，得到所需的Ua、Ub、Uc，Ud暂取0




#endif