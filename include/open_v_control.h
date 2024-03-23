#ifndef OPEN_V_CONTROL
#define OPEN_V_CONTROL

#pragma once

#include <Arduino.h>
#include "Ndef_Motor.h"
#include <cmath>


// 用CV模式，电流最高给到1A+。慎用，电机发热严重。


float mach_angle=0, elec_angle=0;
float u_d=0, u_q=0, u_alpha=0, u_beta=0, u_a=0, u_b=0, u_c=0, pwm_a=0, pwm_b=0, pwm_c=0;
float timestamp=0, zero_angle=0;

// 相关参数定义

#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

// cv使用宏定义的限制函数增快运行速度 使用问号表达式


void ov_init(){
    pinMode(PWM_A, OUTPUT);
    pinMode(PWM_B, OUTPUT);
    pinMode(PWM_C, OUTPUT);
    ledcSetup(channel_A, 30000, 8);
    ledcSetup(channel_B, 30000, 8);
    ledcSetup(channel_C, 30000, 8);
    ledcAttachPin(PWM_A, channel_A);
    ledcAttachPin(PWM_B, channel_B);
    ledcAttachPin(PWM_C, channel_C);
}
// 初始化函数

MOTOR motor = {
    12.5f, //额定电压
    7.0f,  //电机 极对数
    10.0f,  //目标速度   rad/s
    0.0f   //目标力矩
};
float a = motor.motor_pairs;
// 初始化电机结构体

float Normalize(float angle_i){
    float ang_i =  fmod(angle_i, 2*PI);
    return ang_i > 0 ? ang_i : ang_i=2*PI;
}

// 归一化取模，使大于2pi的弧度均转为之内

void cal_e_angle(){
    elec_angle = motor.motor_pairs * mach_angle;
}

// 电角度求解

void setPWM(float a, float b, float c){
    pwm_a = _constrain(a / motor.elec_volt_constrain, 0.0f , 1.0f );
    pwm_b = _constrain(b / motor.elec_volt_constrain, 0.0f , 1.0f );
    pwm_c = _constrain(c / motor.elec_volt_constrain, 0.0f , 1.0f );

    ledcWrite(channel_A, pwm_a*255);
    ledcWrite(channel_B, pwm_b*255);
    ledcWrite(channel_C, pwm_c*255);
}

// PWM赋值
void cal_v(){
    unsigned long now_i = micros();
    float loop_time = (now_i - timestamp)*1e-6f;
    mach_angle = Normalize(mach_angle + motor.aim_vocility*loop_time);
    u_q = motor.elec_volt_constrain/2.5;
    timestamp = now_i;


}

// 开环速度解算，Key：单次运行时间（mircos()函数）本次时间戳减去上次时间戳为单次循环时间
// 代替了位置闭环中，机械角度闭环与u_q计算，速度开环，力矩给定

void cal_kernal(){
    elec_angle = Normalize(elec_angle + zero_angle);
    u_alpha = -u_q * sin(elec_angle);
    u_beta = u_q * cos(elec_angle);

// park逆变换
    u_a = u_alpha + motor.elec_volt_constrain/2;
    u_b = (sqrt(3)*u_beta - u_alpha)/2 + motor.elec_volt_constrain/2;
    u_c = (-sqrt(3)*u_beta - u_alpha)/2 + motor.elec_volt_constrain/2;

// Clarke逆变换

}
// Park逆变换、Clarke逆变换，得到所需的Ua、Ub、Uc，Ud暂取0

void open_v_run(){
    cal_v();
    cal_e_angle();
    cal_kernal();
    setPWM(u_a, u_b, u_c);
}


#endif