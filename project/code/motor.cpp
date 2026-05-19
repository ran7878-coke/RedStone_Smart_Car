#include "zf_common_headfile.hpp"

struct pwm_info drv8701e_pwm_1_info;
struct pwm_info drv8701e_pwm_2_info;


zf_driver_gpio  drv8701e_dir_1(DIR_1_PATH, O_RDWR);
zf_driver_gpio  drv8701e_dir_2(DIR_2_PATH, O_RDWR);
zf_driver_pwm   drv8701e_pwm_1(PWM_1_PATH);
zf_driver_pwm   drv8701e_pwm_2(PWM_2_PATH);

int8 duty = 0;
bool dir = true;

extern zf_driver_pit system_pit;

void sigint_handler(int signum) 
{
    printf("收到Ctrl+C，程序即将退出\n");
    exit(0);
}

void cleanup()
{
    // 需要先停止定时器线程，后面才能稳定关闭电机，电调，舵机等
    system_pit.stop();
    printf("程序异常退出，执行清理操作\n");
    // 关闭电机
    drv8701e_pwm_1.set_duty(0);   
    drv8701e_pwm_2.set_duty(0);    
}


void motor_Init(void)
{
    drv8701e_pwm_1.get_dev_info(&drv8701e_pwm_1_info);
    drv8701e_pwm_2.get_dev_info(&drv8701e_pwm_2_info);

    // 注册清理函数
    atexit(cleanup);

    // 注册SIGINT信号的处理函数
    signal(SIGINT, sigint_handler);

}

void set_left_speed(int duty)
{
     if(duty >= 0)                               // 正转
        {
            drv8701e_dir_1.set_level(0);         // DIR输出高电平
            drv8701e_pwm_1.set_duty(duty);       // 计算占空比
        }
        else
        {
            drv8701e_dir_1.set_level(1);         // DIR输出低电平
            drv8701e_pwm_1.set_duty(-duty);      // 计算占空比
        }
}

void set_right_speed(int duty)
{
     if(duty >= 0)                               // 正转
        {
            drv8701e_dir_2.set_level(0);         // DIR输出高电平
            drv8701e_pwm_2.set_duty(duty);       // 计算占空比
        }
        else
        {
            drv8701e_dir_2.set_level(1);         // DIR输出低电平
            drv8701e_pwm_2.set_duty(-duty);      // 计算占空比
        }
}