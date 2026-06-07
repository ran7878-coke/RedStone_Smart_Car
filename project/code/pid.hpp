#ifndef __PID_HPP
#define __PID_HPP

#define PWM_MAX       100   // PWM最大值
#define LINE_THRESHOLD 120       // 折线判定阈值（传感器偏差超15=进入折线）
#define BIAS_DEADZONE 0         // 直道死区（偏差<5=直道）

// PID结构体定义
typedef struct {
    float Kp;             // 比例系数
    float Ki;             // 积分系数
    float Kd;             // 微分系数
    float output_limit;   // 输出限幅
    float integral_limit; // 积分限幅（抗积分饱和）

    float error;          // 当前偏差
    float last_error;     // 上一次偏差
    float prev_error;     // 上上次偏差（增量式专用）
    float integral;       // 积分累积值（位置式专用）
    float output;         // PID输出值
} PID_TypeDef;

void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd, float output_limit, float integral_limit);
float PID_Incremental_Calculate(PID_TypeDef *pid, float feedback, float setpoint);
float PID_Positional_Calculate(PID_TypeDef *pid, float feedback, float setpoint);
void PID_Reset(PID_TypeDef *pid);

extern PID_TypeDef TracePID;
extern PID_TypeDef AnglePID;
extern PID_TypeDef Speed_lPID;
extern PID_TypeDef Speed_rPID;
extern PID_TypeDef Delta_SpPID;

#endif