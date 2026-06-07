#include "zf_common_headfile.hpp"

PID_TypeDef TracePID;
PID_TypeDef AnglePID;
PID_TypeDef Speed_lPID;
PID_TypeDef Speed_rPID;
PID_TypeDef Delta_SpPID;

// PID初始化
void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd, float output_limit, float integral_limit)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->output_limit = output_limit;
    pid->integral_limit = integral_limit;

    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}

//==================== 增量式 PID（内环：电机/角度控制）====================
float PID_Incremental_Calculate(PID_TypeDef *pid, float feedback, float setpoint)
{
    if(pid == NULL) return 0.0f;

    // 计算当前偏差
    pid->error = setpoint - feedback;

    // 增量式PID公式
    float delta_u = pid->Kp * (pid->error - pid->last_error)
                   + pid->Ki * pid->error
                   + pid->Kd * (pid->error - 2 * pid->last_error + pid->prev_error);

    // 输出累加 + 限幅 + 积分限幅
    pid->output += delta_u;
    if(pid->output >  pid->output_limit)  pid->output =  pid->output_limit;
    if(pid->output < -pid->output_limit)  pid->output = -pid->output_limit;

    // 更新偏差历史
    pid->prev_error = pid->last_error;
    pid->last_error = pid->error;

    return pid->output;
}

//==================== 位置式 PID（外环：寻迹/位置控制）====================
float PID_Positional_Calculate(PID_TypeDef *pid, float feedback, float setpoint)
{
    if(pid == NULL) return 0.0f;

    // 计算当前偏差
    pid->error = setpoint - feedback;

    float I_THRESHOLD = 3;
    if ((pid->error/setpoint) < I_THRESHOLD)
    {
        // 积分累加 + 积分限幅
        pid->integral += pid->Ki * pid->error;
        if(pid->integral >  pid->integral_limit)  pid->integral =  pid->integral_limit;
        if(pid->integral < -pid->integral_limit)  pid->integral = -pid->integral_limit;
    }
    else
        pid->integral = 0;
   
    // 位置式PID公式
    float output =  pid->Kp * pid->error
                  + pid->integral
                  + pid->Kd * (pid->error - pid->last_error);

    // 输出限幅
    pid->output = output;
    if(pid->output >  pid->output_limit)  pid->output =  pid->output_limit;
    if(pid->output < -pid->output_limit)  pid->output = -pid->output_limit;

    // 更新上一次偏差
    pid->last_error = pid->error;

    return pid->output;
}

// PID重置
void PID_Reset(PID_TypeDef *pid)
{
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}
