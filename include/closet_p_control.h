#ifndef CLOSET_P_CONTROL_H
#define CLOSET_P_CONTROL_H

// #pragma once

#include <Arduino.h>
#include "pid_defi.h"
#include "Ndef_Motor.h"
#include <cmath>
#include <Sensor_AS5600.h>

float elec_angle=0;
float u_d=0, u_q=0, u_alpha=0, u_beta=0, u_a=0, u_b=0, u_c=0, pwm_a=0, pwm_b=0, pwm_c=0;
float timestamp=0, zero_e_angle=0;
float aim_angle = -0;  //rad  注意要为负

int DIR = 1;

#define _3PI_2 4.71238898038f
// 相关参数定义

#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

// cv使用宏定义的限制函数增快运行速度 使用问号表达式

PID gen_uq;
Sensor_AS5600 S0=Sensor_AS5600(0);
TwoWire S0_I2C = TwoWire(0);

MOTOR motor = {
    12.6f, //额定电压
    7.0f,  //电机 极对数
    5.0f,  //目标速度   rad/s
    0.0f   //目标力矩
};
// float a = motor.motor_pairs;
// 初始化电机结构体

float Normalize(float angle_i){
    float ang_i =  fmod(angle_i, 2*PI);
    return ang_i > 0 ? ang_i : ang_i=2*PI;
}

// 归一化取模，使大于2pi的弧度均转为之内

float cal_e_angle(){
    return Normalize((float)((motor.motor_pairs * DIR) * S0.getMechanicalAngle() - zero_e_angle));

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


void put_u(float a){

    PID_calc(&gen_uq, aim_angle, (DIR * a));
    u_q = gen_uq.output*180/PI;
    // Serial.print(" res=");Serial.println(res);
    // return res;
    u_q = _constrain(u_q, -motor.elec_volt_constrain/2, motor.elec_volt_constrain/2);
    // Serial.print(" u_q=");Serial.println(u_q);
}

// 位置闭环计算电角度，计算u_q（力矩同时闭环）   调用cal_e_angle求电角度

void setTorque(float Uq,float angle_el){
    S0.Sensor_update(); //！new

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
    Serial.println("PWM OK.");
    // BeginSensor();
    //AS5600
    S0_I2C.begin(19,18, 400000UL);
    S0.Sensor_init(&S0_I2C);   //初始化编码器0
    Serial.println("编码器加载完毕");

    setTorque(3, _3PI_2);
    delay(1000);
    S0.Sensor_update(); 
    zero_e_angle = cal_e_angle();
    setTorque(0, _3PI_2);
    // cal_e_angle();
    Serial.print("0电角度:");Serial.println(zero_e_angle);
// 零点角度检测

    PID_Init(&gen_uq, 0.133, 0, 0, 50, 0.9);

    delay(100);
}
// 初始化函数


void test(){

    float sensor_angle = S0.getAngle();
    // cal_e_angle();
    put_u(sensor_angle);
    // Serial.print("角度:");Serial.println(sensor_angle);
    elec_angle = cal_e_angle();
    setTorque(u_q, elec_angle);
    // delay(1);
}



// 执行/测试


#endif