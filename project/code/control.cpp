#include "zf_common_headfile.hpp"
#include "zf_driver_gpio.hpp"
//#define LEFT_DEADZONE 7 //PWM左轮死区补偿，融入前馈
//#define RIGHT_DEADZONE 8 //PWM右轮死区补偿，融入前馈
#define ZERO_THRESHOLD 2 //死区判断阈值
//速度前馈需要上赛道重调
#define kf_l 22.049 //左PWM与速度的前馈系数1   只做了正向！反向会有一定误差  21.46
#define bf_l 493.24 //左PWM与速度的前馈系数2  PWM_f = target_speed * kf + bf    409.89
#define kf_r 22.049 //右PWM与速度的前馈系数1   只做了正向！反向会有一定误差 19.733
#define bf_r 493.24 //右PWM与速度的前馈系数2  PWM_f = target_speed * kf + bf    497.42,593.24
//角度前馈
#define kf_turn 7.25//轮距/2  kf_turn * omega = delt_target_speed

zf_device_imu imu_dev;
float target_lspeed = 0;
float target_rspeed = 0;
float current_lspeed = 0;
float current_rspeed = 0;
float average_speed = 0;
float current_delta_Sp = 0;
float target_delta_Sp = 0;
float delta_Sp = 0;
float target_lsp = 0;
float target_rsp = 0;
float left_PWM_PI = 0;
float right_PWM_PI = 0;
float left_PWM = 0;
float right_PWM = 0;
float target_omega = 0;
float current_omega = 0;
float steer = 0;
uint8 t_n = 0;//中断计数

const float encoder_filter = 0.3;//编码器滤波系数(越小越强) ps.实测建议在0.25~0.35之间
const float filter_off = 4;//速度需要大幅变动时，停止滤波的速度差值

// 限幅函数
int constrain(int val, int min_val, int max_val)
{
    if(val > max_val) return max_val;
    if(val < min_val) return min_val;
    return val;
}

float Feed_Forward_l(float target_speed)
{
    if (target_speed > ZERO_THRESHOLD)//死区判定
        return target_speed * kf_l + bf_l;//输出前馈
    else if (target_speed < -ZERO_THRESHOLD)
        return  -(-target_speed * kf_l + bf_l);
    else
        return 0;
}

float Feed_Forward_r(float target_speed)
{
    if (target_speed > ZERO_THRESHOLD)//死区判定
        return target_speed * kf_r + bf_r;//输出前馈
    else if (target_speed < -ZERO_THRESHOLD)
        return  -(-target_speed * kf_r + bf_r);
    else
        return 0;
}

void line_follow_pid_control(void)
{
    if (t_n == 10)
        t_n = 0;//重置计数
    t_n++;
    //===================== 编码器取速度，滤波 =====================
    //编码器读取，并滤波
    float raw_lspeed = get_left_speed_mps();//左
    if (abs(raw_lspeed - current_lspeed) > filter_off)//大于阈值不滤波
        current_lspeed = raw_lspeed;
    else//小于阈值滤波
       current_lspeed = (encoder_filter * raw_lspeed) + ((1.0 - encoder_filter) * current_lspeed);
    float raw_rspeed = get_right_speed_mps();//右
    if (abs(raw_rspeed - current_rspeed) > filter_off)
        current_rspeed = raw_rspeed;
    else
        current_rspeed = (encoder_filter * raw_rspeed) + ((1.0 - encoder_filter) * current_rspeed);
    average_speed = (current_lspeed + current_rspeed) / 2;

    // ===================== 1. 外环：图像偏差 → 目标角速度 =====================
    //center_error在图像处理中给出
    float center_error = get_center_error();
    //if (t_n == 10)//100ms跑一次
        target_omega = PID_Positional_Calculate(&TracePID, center_error, 0.0f);//根据偏差得出角速度
    //target_omega = (curvature) * 0.15;

    // ===================== 2. 内环：陀螺仪角速度 → 转向差速 =====================
    current_omega = imu_dev.get_gyro_z();  // 替换为你的陀螺仪航向角函数
    steer = PID_Positional_Calculate(&AnglePID, current_omega, target_omega);

    // ===================== 3. 差速环：编码器 → 差速 =====================
    //给出目标速度，基础 + 角度环输出 + 角度前馈
    target_lspeed = BASE_SPEED + steer + kf_turn * target_omega;
    target_rspeed = BASE_SPEED - steer - kf_turn * target_omega;
    target_delta_Sp = target_lspeed - target_rspeed;
    if ( target_delta_Sp > 0.1)
    {
        current_delta_Sp = current_lspeed - current_rspeed;
        delta_Sp = PID_Positional_Calculate(&Delta_SpPID, current_delta_Sp, target_delta_Sp);
        target_lsp += delta_Sp;
        target_rsp -= delta_Sp;
    }
    else
    {
        current_delta_Sp = 0;
        target_lsp = target_lspeed;
        target_rsp = target_rspeed;
    }

    // ===================== 4. 速度环：编码器 → 速度 =====================
    left_PWM_PI = PID_Incremental_Calculate(&Speed_lPID, current_lspeed, target_lsp);//左
    right_PWM_PI = PID_Incremental_Calculate(&Speed_rPID, current_rspeed, target_rsp);//右
    //输出PWM = 前馈 + PI输出
    left_PWM = Feed_Forward_l(target_lspeed) + left_PWM_PI;
    right_PWM = Feed_Forward_r(target_rspeed) + right_PWM_PI;

    // ===================== 6. 限幅（防止超范围） =====================
     left_PWM  = constrain(left_PWM,  -2000, 2000);
     right_PWM = constrain(right_PWM, -2000, 2000);

    // ===================== 7. 输出到电机 =====================
    set_left_speed((int)left_PWM);
    set_right_speed((int)right_PWM);
}
