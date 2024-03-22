#ifndef NDEF_MOTOR_H
#define NDEF_MOTOR_H

#pragma once

#define PWM_A 32
#define PWM_B 33
#define PWM_C 25

#define channel_A 0
#define channel_B 1
#define channel_C 2


typedef struct newdefMotor{
    float elec_volt_constrain;
    float motor_pairs;
    float aim_vocility;
    float aim_torque;

}MOTOR, *PTRMOTOR;


#endif