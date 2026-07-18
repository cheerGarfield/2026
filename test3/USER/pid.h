#ifndef __PID_H
#define __PID_H

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float setpoint;
    float integral;
    float prev_error;
    float output;
    float max_integral;
    float max_output;
} PID_t;

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, float max_out);
float PID_Update(PID_t *pid, float current_value);
void PID_Reset(PID_t *pid);
void PID_SetSetpoint(PID_t *pid, float sp);

#endif
