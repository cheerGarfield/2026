#include "pid.h"

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, float max_out)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->setpoint = 0.0f;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output = 0.0f;
    pid->max_integral = max_out * 0.5f;
    pid->max_output = max_out;
}

float PID_Update(PID_t *pid, float current_value)
{
    float error, derivative;

    /* current_value 视为误差量, setpoint 作为基准偏移(默认0)
       error = current_value - setpoint
       这样调用者传入误差 e 时, PID 输出与 e 同号 */
    error = current_value - pid->setpoint;

    // 积分累加（限幅防饱和）
    pid->integral += error;
    if (pid->integral > pid->max_integral)
        pid->integral = pid->max_integral;
    else if (pid->integral < -pid->max_integral)
        pid->integral = -pid->max_integral;

    // 微分
    derivative = error - pid->prev_error;
    pid->prev_error = error;

    // PID输出
    pid->output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;

    // 输出限幅
    if (pid->output > pid->max_output)
        pid->output = pid->max_output;
    else if (pid->output < -pid->max_output)
        pid->output = -pid->max_output;

    return pid->output;
}

void PID_Reset(PID_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output = 0.0f;
}

void PID_SetSetpoint(PID_t *pid, float sp)
{
    pid->setpoint = sp;
    PID_Reset(pid);
}
