#ifndef PID_DEFI_H
#define PID_DEFI_H

typedef struct{
    float kp, ki, kd;
    float error, last_error;
    float integral, MaxIntegral;
    float output, MaxOutput;
}PID;



void PID_Init(PID *pid, float P, float I, float D, float maxO,float maxI){
    pid->kp = P;
    pid->ki = I;
    pid->kd = D;
    pid->MaxIntegral = maxI;
    pid->MaxOutput = maxO;
}

void PID_calc(PID *pid, float target, float feedback){
    pid->last_error = pid->error;
    pid->error = (target - feedback);
    float pout = pid->error * pid->kp;
    float dout = (pid->error - pid->last_error) * pid->kd;
    pid->integral = pid->integral + pid->error;
    if(pid->integral > pid->MaxIntegral)
        pid->integral = pid->MaxIntegral;
    else if (pid->integral < -pid->MaxIntegral)
        pid->integral = pid->MaxIntegral;
    pid->output = pout+dout+pid->integral;
    if (pid->output > pid->MaxOutput)
        pid->output = pid->MaxOutput;
    else if(pid->output < -pid->MaxOutput)
        pid->output = pid->MaxOutput;
}


#endif