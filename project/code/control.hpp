#ifndef CONTROL_HPP
#define CONTROL_HPP

void line_follow_pid_control(void);
void pid_tuner_check(void);
#define CENTER_X (IMAGE_W / 2)
#define BASE_SPEED 0.8
extern zf_device_imu imu_dev;
float get_target_angle(void);
float get_steer(void);
extern float target_omega;
extern float current_omega;
extern float steer;
extern float current_lspeed;
extern float current_rspeed;
extern float target_speed;
extern float average_speed;
extern float PWM_l;
extern float PWM_r;
extern float target_lspeed;
extern float target_rspeed;
extern float target_delta_Sp;
extern float current_delta_Sp;
extern float PWM_delta;
extern float left_PWM;
extern float right_PWM;
extern float raw_lspeed;
extern float raw_rspeed;

#endif 