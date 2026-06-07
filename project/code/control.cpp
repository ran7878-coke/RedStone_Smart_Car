#include "zf_common_headfile.hpp"
#include "zf_driver_gpio.hpp"
#include "battery_voltage.hpp"
#include <cstdio>
#include <cstring>

//#define LEFT_DEADZONE 7 //PWM左轮死区补偿，融入前馈
//#define RIGHT_DEADZONE 8 //PWM右轮死区补偿，融入前馈
#define ZERO_THRESHOLD 0.08 //死区判断阈值
//速度前馈需要上赛道重调
#define kf_l 598.707 //左PWM与速度的前馈系数1   只做了正向！反向会有一定误差  21.46    22.049
#define bf_l 493.24 //左PWM与速度的前馈系数2  PWM_f = target_speed * kf + bf    409.89
#define kf_r 598.707 //右PWM与速度的前馈系数1   只做了正向！反向会有一定误差 19.733
#define bf_r 493.24 //右PWM与速度的前馈系数2  PWM_f = target_speed * kf + bf    497.42,593.24
//角度前馈
#define kf_turn 0.0725//轮距/2  kf_turn * omega = delt_target_speed

zf_device_imu imu_dev;
float target_omega = 0;
float current_omega = 0;
float steer = 0;
float current_lspeed = 0;
float current_rspeed = 0;
float target_speed = 0;
float average_speed = 0;
float PWM_l = 0;
float PWM_r = 0;
float target_lspeed = 0;
float target_rspeed = 0;
float target_delta_Sp = 0;
float current_delta_Sp = 0;
float PWM_delta = 0;
float left_PWM = 0;
float right_PWM = 0;
float raw_lspeed = 0;
float raw_rspeed = 0;

uint8 t_n = 0;//中断计数


const float encoder_filter = 0.01;//编码器滤波系数(越小越强)

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
        return kf_l * target_speed + bf_l;//输出前馈
    else if (target_speed < -ZERO_THRESHOLD)
        return  -(kf_l * target_speed + bf_l);
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
    //编码器读取，并滤波（始终滤波，毛刺不穿透）
    raw_lspeed = get_left_speed_mps();//左
    current_lspeed = (encoder_filter * raw_lspeed) + ((1.0 - encoder_filter) * current_lspeed);
    raw_rspeed = get_right_speed_mps();//右
    current_rspeed = (encoder_filter * raw_rspeed) + ((1.0 - encoder_filter) * current_rspeed);

    // ===================== 1. 外环：图像偏差 → 目标角速度 =====================
    //center_error在图像处理中给出
    float center_error = get_center_error();
    //if (t_n == 10)//100ms跑一次
    target_omega = PID_Positional_Calculate(&TracePID, center_error, 0.0f);//根据偏差得出角速度
    //target_omega = (curvature) * 0.15;

    // ===== target_omega 变化率限制，只削尖峰不引入滞后 =====
    static float smooth_target_omega = 0;
    #define OMEGA_SLEW 1.0f  // 每周期最大变化 1.0 rad/s，小变化直通
    float omega_delta = target_omega - smooth_target_omega;
    if (omega_delta >  OMEGA_SLEW)      smooth_target_omega += OMEGA_SLEW;
    else if (omega_delta < -OMEGA_SLEW) smooth_target_omega -= OMEGA_SLEW;
    else                                smooth_target_omega  = target_omega;

    // ===================== 2. 内环：陀螺仪角速度 → 转向差速 =====================
    //current_omega = imu_dev.get_gyro_z();  // 替换为你的陀螺仪航向角函数
    //steer = PID_Positional_Calculate(&AnglePID, current_omega, target_omega);

     // ===================== 3. 线速度环：编码器 → 线速度 =====================
    //给出目标速度，基础
    //target_speed = 0;
    average_speed = (current_lspeed + current_rspeed) / 2;
    //target_omega = 0;//6.28/5;
    target_lspeed = 1.2 + kf_turn * smooth_target_omega;
    target_rspeed = 1.2 - kf_turn * smooth_target_omega;
    //输出PWM = 前馈 + 线速度环PI输出
    PWM_l = Feed_Forward_l(target_lspeed) + PID_Incremental_Calculate(&Speed_lPID, current_lspeed, target_lspeed);
    PWM_r = Feed_Forward_r(target_rspeed) + PID_Incremental_Calculate(&Speed_rPID, current_rspeed, target_rspeed);

    // ===================== 4. 差速环：编码器 → 差速 =====================
    target_delta_Sp = kf_turn * smooth_target_omega * 2;
    current_delta_Sp = current_lspeed - current_rspeed;
    PWM_delta = PID_Incremental_Calculate(&Delta_SpPID, current_delta_Sp, target_delta_Sp);

    // ===================== 6. 合成最终PWM，限幅（防止超范围） =====================
    left_PWM = PWM_l + PWM_delta;
    right_PWM = PWM_r - PWM_delta;

    // float bv_f = battery_voltage_get() / 11100;//电压前馈
    // left_PWM = left_PWM * bv_f;
    // right_PWM = right_PWM * bv_f;

    left_PWM  = constrain(left_PWM,  -2000, 2000);
    right_PWM = constrain(right_PWM, -2000, 2000);

    // ===================== 7. 输出到电机 =====================
    set_left_speed((int)left_PWM);
    set_right_speed((int)right_PWM);
}

// ===================== 热加载 PID 参数（文件方式）=====================
#define TUNE_FILE "/home/root/pid_tune.txt"

void pid_tuner_check(void)
{
    static int counter = 0;
    if (++counter < 30) return;   // 约1秒检查一次
    counter = 0;

    FILE *f = fopen(TUNE_FILE, "r");
    if (!f) return;

    char key[32];
    float val;
    bool changed = false;

    while (fscanf(f, "%31s %f", key, &val) == 2)
    {
        if (key[0] == '#') { while (fgetc(f) != '\n' && !feof(f)); continue; }

        if      (strcmp(key, "Speed_l_Kp") == 0)  { Speed_lPID.Kp = val; changed = true; }
        else if (strcmp(key, "Speed_l_Ki") == 0)  { Speed_lPID.Ki = val; changed = true; }
        else if (strcmp(key, "Speed_l_Kd") == 0)  { Speed_lPID.Kd = val; changed = true; }
        else if (strcmp(key, "Speed_r_Kp") == 0)  { Speed_rPID.Kp = val; changed = true; }
        else if (strcmp(key, "Speed_r_Ki") == 0)  { Speed_rPID.Ki = val; changed = true; }
        else if (strcmp(key, "Speed_r_Kd") == 0)  { Speed_rPID.Kd = val; changed = true; }
        else if (strcmp(key, "Delta_Sp_Kp") == 0) { Delta_SpPID.Kp = val; changed = true; }
        else if (strcmp(key, "Delta_Sp_Ki") == 0) { Delta_SpPID.Ki = val; changed = true; }
        else if (strcmp(key, "Delta_Sp_Kd") == 0) { Delta_SpPID.Kd = val; changed = true; }
        else if (strcmp(key, "Trace_Kp") == 0)    { TracePID.Kp = val; changed = true; }
        else if (strcmp(key, "Trace_Ki") == 0)    { TracePID.Ki = val; changed = true; }
        else if (strcmp(key, "Trace_Kd") == 0)    { TracePID.Kd = val; changed = true; }
        else if (strcmp(key, "Angle_Kp") == 0)    { AnglePID.Kp = val; changed = true; }
        else if (strcmp(key, "Angle_Ki") == 0)    { AnglePID.Ki = val; changed = true; }
        else if (strcmp(key, "Angle_Kd") == 0)    { AnglePID.Kd = val; changed = true; }
    }
    fclose(f);

    if (changed)
    {
        remove(TUNE_FILE);
        printf("[TUNE] l=%.0f/%.2f/%.3f r=%.0f/%.2f/%.3f d=%.0f/%.2f/%.3f t=%.1f/%.2f/%.3f a=%.4f/%.2f/%.3f\n",
               Speed_lPID.Kp, Speed_lPID.Ki, Speed_lPID.Kd,
               Speed_rPID.Kp, Speed_rPID.Ki, Speed_rPID.Kd,
               Delta_SpPID.Kp, Delta_SpPID.Ki, Delta_SpPID.Kd,
               TracePID.Kp, TracePID.Ki, TracePID.Kd,
               AnglePID.Kp, AnglePID.Ki, AnglePID.Kd);
    }
}