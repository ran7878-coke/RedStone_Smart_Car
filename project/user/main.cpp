
/*********************************************************************************************************************
* LS2K0300 Opensourec Library 即（LS2K0300 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是LS2K0300 开源库的一部分
*
* LS2K0300 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          main
* 公司名称          成都逐飞科技有限公司
* 适用平台          LS2K0300
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者           备注
* 2025-12-27        大W            first version
********************************************************************************************************************/

#include "zf_common_headfile.hpp"

zf_driver_pit system_pit;

// 10ms中断
void system_pit_callback(void)
{
        encoder_update();  // 编码器计算
        //printf("get_target_angle%f\n",get_target_angle());
        //printf("get_steer:%f\n",get_steer());
        //printf("get_gyro_z:%f\n",imu_dev.get_gyro_z()*0.01f);
        //printf("\n");
}

int main(int, char**) 
{
//******************************初始化区*************************************
    ips200.init(FB_PATH);

    if(uvc_dev.init(UVC_PATH) < 0)
    {
        return -1;
    }

    imu_dev.init();
    motor_Init();
    
//******************************pit中断配置**********************************

    system_pit.init_ms(100, system_pit_callback);

//******************************pid参数配置**********************************
    PID_Init(&TracePID,  3.5f, 0.0f,  2.5f,  50.0f,  0.0f);    // 图像→角度  Kp小 Kd大
    PID_Init(&AnglePID,  0.3f, 0.0f,  3.5f,  50.0f,  0.0f);    // 角度→电机  Kp大 Kd中
    PID_Init(&SpeedPID,  0.5f, 0.3f,  0.0f,  25.0f,   10.0f);    // 速度环     Kp小 Kd小

//******************************主循环**********************************

            while(1)
            {

                if(uvc_dev.wait_image_refresh() == 0)
                {
                    //system_delay_ms(100);
                    image_process();
                    //line_follow_pid_control();
                } 
/*
                float v_left  = get_left_speed_mps();
                float v_right = get_right_speed_mps();
                float dist_left = get_left_distance();
                float dist_right = get_right_distance();
                float a_left  = get_left_accel();
                float a_right = get_right_accel();

                // 打印
                printf("=========================================\r\n");
                printf("速度L：%.3f m/s      R：%.3f m/s\r\n", v_left, v_right);
                printf("里程L：%.4f 米      R：%.4f 米\r\n", dist_left, dist_right);
                printf("加速度L：%.2f m/s²   R：%.2f m/s²\r\n", a_left, a_right);
                printf("=========================================\r\n\r\n");

                system_delay_ms(200);    */           
  //              printf("=====================================\r\n");
  //              printf("Roll   = %.2f °\r\n", eulerAngle.roll);    // 横滚
 //               printf("Pitch  = %.2f °\r\n", eulerAngle.pitch);   // 俯仰
  //              printf("Yaw    = %.2f °\r\n", eulerAngle.yaw);     // 偏航
   //             printf("=====================================\r\n\r\n");  
 //  printf("steer:%f",get_steer());
 //  printf("target_angle:%f",get_target_angle());
            }
    }
