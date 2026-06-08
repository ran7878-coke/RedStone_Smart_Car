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

    // 输出累加 + 限幅
    pid->output += delta_u;
    if(pid->output >  pid->output_limit)  pid->output =  pid->output_limit;
    if(pid->output < -pid->output_limit)  pid->output = -pid->output_limit;

    // 更新偏差历史
    pid->prev_error = pid->last_error;
    pid->last_error = pid->error;

    return pid->output;
}

//==================== 位置式 PID ====================
float PID_Positional_Calculate(PID_TypeDef *pid, float feedback, float setpoint)
{
    if(pid == NULL) return 0.0f;

    // 计算当前偏差
    pid->error = setpoint - feedback;

    // 积分分离：误差变化率大 → 系统在暂态 → 冻结积分
    // 变化率小 → 已趋稳态 → 激活积分消静差。不受滞后影响
    if (pid->Ki > 0.001f)
    {
        float de = fabs(pid->error - pid->last_error);  // |Δerror|
        float DE_THRESH = 0.005f;  // 误差每周期变化 < 此值认为系统稳定
        if (de < DE_THRESH)
        {
            pid->integral += pid->Ki * pid->error;
            if(pid->integral >  pid->integral_limit)  pid->integral =  pid->integral_limit;
            if(pid->integral < -pid->integral_limit)  pid->integral = -pid->integral_limit;
        }
        // else: 误差太大，冻结积分，防止风up
    }
    else
    {
        // Ki=0，跳过积分
        pid->integral = 0.0f;
    }

    // 位置式 PID 公式
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
