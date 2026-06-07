#ifndef _ENCODER_HPP
#define _ENCODER_HPP

// ===================== 硬件固定参数 =====================
#define PULSE_PER_WHEEL_REV    4642.133f   // 轮胎一圈脉冲      1024*2*68/30一圈脉冲乘以齿轮比
#define WHEEL_CIRCUMFERENCE    0.20106f  // 轮胎周长 米
#define DT                    0.01f     // 10ms = 0.01秒
// ======================================================

// 编码器对象
#define ENCODER_QUAD_1_PATH ZF_ENCODER_QUAD_1
#define ENCODER_QUAD_2_PATH ZF_ENCODER_QUAD_2

void encoder_init(void);
void encoder_update(void);  //更新编码器速度里程数据

// ===================== 外部调用函数 =====================
// 获取速度 m/s
float get_left_speed_mps(void);
float get_right_speed_mps(void);

// 获取速度 km/h
float get_left_speed_kmh(void);
float get_right_speed_kmh(void);

// 获取里程 米
float get_left_distance(void);
float get_right_distance(void);

// 获取加速度 m/s²
float get_left_accel(void);
float get_right_accel(void);

// 清零里程
void clear_distance(void);

#endif