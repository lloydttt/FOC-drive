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



}
