#include "open_v_control.h"

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

void cal_v(){
    

}


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
