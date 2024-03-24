#ifndef CLOSET_P_CONTROL_H
#define CLOSET_P_CONTROL_H

#pragma once

#include <Arduino.h>
#include "pid_defi.h"
#include "Ndef_Motor.h"
#include <cmath>
#include <AS5600.h>

float elec_angle=0;
float u_d=0, u_q=0, u_alpha=0, u_beta=0, u_a=0, u_b=0, u_c=0, pwm_a=0, pwm_b=0, pwm_c=0;
float timestamp=0, zero_e_angle=0;
float aim_angle = -PI/2;  //rad  注意要为负

int DIR = 1;

#define _3PI_2 4.71238898038f
// 相关参数定义

#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

// cv使用宏定义的限制函数增快运行速度 使用问号表达式

PID gen_uq;


MOTOR motor = {
    12.6f, //额定电压
    7.0f,  //电机 极对数
    5.0f,  //目标速度   rad/s
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
    elec_angle = (float)((motor.motor_pairs * DIR) * getAngle_Without_track() - zero_e_angle);

    // Serial.print("e_angle = ");Serial.println(elec_angle);
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



float generate_i(float a){

    PID_calc(&gen_uq, aim_angle, (DIR * a));
    float res = gen_uq.output*180/PI;
    // Serial.print(" res=");Serial.println(res);
    return res;
}

void put_u(float a){
    u_q = generate_i(a);
    u_q = _constrain(u_q, -6, 6);
    // Serial.print(" u_q=");Serial.println(u_q);
}

// 位置闭环计算电角度，计算u_q（力矩同时闭环）   调用cal_e_angle求电角度

void cal_R_kernal(float Uq,float angle_el){


    angle_el = Normalize(angle_el);
    u_alpha = -Uq * sin(angle_el);
    u_beta = Uq * cos(angle_el);

// park逆变换
    u_a = u_alpha + motor.elec_volt_constrain/2;
    u_b = (sqrt(3)*u_beta - u_alpha)/2 + motor.elec_volt_constrain/2;
    u_c = (-sqrt(3)*u_beta - u_alpha)/2 + motor.elec_volt_constrain/2;

// Clarke逆变换
    setPWM(u_a, u_b, u_c);
}
// Park逆变换、Clarke逆变换，得到所需的Ua、Ub、Uc，Ud暂取0


void cp_init(){
    pinMode(PWM_A, OUTPUT);
    pinMode(PWM_B, OUTPUT);
    pinMode(PWM_C, OUTPUT);
    ledcSetup(channel_A, 30000, 8);
    ledcSetup(channel_B, 30000, 8);
    ledcSetup(channel_C, 30000, 8);
    ledcAttachPin(PWM_A, channel_A);
    ledcAttachPin(PWM_B, channel_B);
    ledcAttachPin(PWM_C, channel_C);
    delay(500);
    BeginSensor();

    // u_q = 3;
    // elec_angle = _3PI_2;
    cal_R_kernal(u_q, _3PI_2);
    delay(3000);
    cal_e_angle();
    zero_e_angle = elec_angle;

    cal_R_kernal(0, _3PI_2);
    cal_e_angle();
    PID_Init(&gen_uq, 0.133, 0, 0, 50, 0.9);
    // cal_e_angle();
    // put_u();
    // cal_R_kernal();
    // delay(2);
    // cal_e_angle();
    // zero_e_angle = elec_angle;

    // gen_uq.kp = 0.133;
// 校准

    // delay(3000);
    // zero_e_angle=_electricalAngle();
    // setPhaseVoltage(0, 0,PI*1.5);
    Serial.print("0电角度:");Serial.println(zero_e_angle);
    delay(100);
}
// 初始化函数


void test(){
    // float temp = getAngle_Without_track();
    // Serial.println(temp);
    // delay(100);
    float sensor_angle = getAngle();
    cal_e_angle();
    put_u(sensor_angle);
    // Serial.print("角度:");Serial.println(sensor_angle);
    cal_e_angle();
    cal_R_kernal(u_q, elec_angle);
    // delay(1);
}



// 执行/测试


#endif