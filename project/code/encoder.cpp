#include "zf_common_headfile.hpp"

zf_driver_encoder encoder_quad_1(ENCODER_QUAD_1_PATH);
zf_driver_encoder encoder_quad_2(ENCODER_QUAD_2_PATH);

// 编码器原始值
int16_t encoder_left;
int16_t encoder_right;

// 速度
float speed_left_mps;    // 左 m/s
float speed_right_mps;   // 右 m/s
float speed_left_kmh;    // 左 km/h
float speed_right_kmh;   // 右 km/h

// 里程（总行驶距离）
float total_distance_left;   // 左轮总里程 米
float total_distance_right;  // 右轮总里程 米

// 加速度
float accel_left_mps2;    // 左加速度 m/s²
float accel_right_mps2;   // 右加速度 m/s²

// 上一时刻速度（用于计算加速度）
float last_speed_left = 0;
float last_speed_right = 0;

// 10ms 中断回调：计算 速度 + 里程 + 加速度
void encoder_update(void)
{
    // 1. 读取10ms内脉冲
    encoder_left  = encoder_quad_2.get_count();
    encoder_right = -encoder_quad_1.get_count();

    // ===================== 速度计算 =====================
    speed_left_mps  = (encoder_left / PULSE_PER_WHEEL_REV) * WHEEL_CIRCUMFERENCE * 200;
    speed_right_mps = (encoder_right / PULSE_PER_WHEEL_REV) * WHEEL_CIRCUMFERENCE * 200;
    speed_left_kmh  = speed_left_mps  * 3.6f;
    speed_right_kmh = speed_right_mps * 3.6f;

    // ===================== 里程累加 =====================
    float delta_left  = fabs(encoder_left)  * WHEEL_CIRCUMFERENCE / PULSE_PER_WHEEL_REV;
    float delta_right = fabs(encoder_right) * WHEEL_CIRCUMFERENCE / PULSE_PER_WHEEL_REV;
    total_distance_left  += delta_left;
    total_distance_right += delta_right;

    // ===================== 加速度计算 =====================
    accel_left_mps2  = (speed_left_mps  - last_speed_left)  / DT;
    accel_right_mps2 = (speed_right_mps - last_speed_right) / DT;
    last_speed_left  = speed_left_mps;
    last_speed_right = speed_right_mps;

    // 清零编码器
    encoder_quad_1.clear_count();
    encoder_quad_2.clear_count();
}


// ===================== 外部调用函数 =====================
// 获取速度 m/s
float get_left_speed_mps(void)  { return speed_left_mps; }
float get_right_speed_mps(void) { return speed_right_mps; }

// 获取速度 km/h
float get_left_speed_kmh(void)  { return speed_left_kmh; }
float get_right_speed_kmh(void) { return speed_right_kmh; }

// 获取里程 米
float get_left_distance(void)   { return total_distance_left; }
float get_right_distance(void)  { return total_distance_right; }

// 获取加速度 m/s²
float get_left_accel(void)      { return accel_left_mps2; }
float get_right_accel(void)     { return accel_right_mps2; }

// 清零里程
void clear_distance(void)
{
    total_distance_left = 0;
    total_distance_right = 0;
}