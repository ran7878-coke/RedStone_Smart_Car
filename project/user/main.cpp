
// /*********************************************************************************************************************
// * LS2K0300 Opensourec Library 即（LS2K0300 开源库）是一个基于官方 SDK 接口的第三方开源库
// * Copyright (c) 2022 SEEKFREE 逐飞科技
// *
// * 本文件是LS2K0300 开源库的一部分
// *
// * LS2K0300 开源库 是免费软件
// * 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
// * 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
// *
// * 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
// * 甚至没有隐含的适销性或适合特定用途的保证
// * 更多细节请参见 GPL
// *
// * 您应该在收到本开源库的同时收到一份 GPL 的副本
// * 如果没有，请参阅<https://www.gnu.org/licenses/>
// *
// * 额外注明：
// * 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
// * 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
// * 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
// * 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
// *
// * 文件名称          main
// * 公司名称          成都逐飞科技有限公司
// * 适用平台          LS2K0300
// * 店铺链接          https://seekfree.taobao.com/
// *
// * 修改记录
// * 日期              作者           备注
// * 2025-12-27        大W            first version
// ********************************************************************************************************************/

// #include "zf_common_headfile.hpp"
// #include "control.hpp"
// zf_driver_pit system_pit;
// uint16 t = 0;//调试用

// // 10ms中断
// void system_pit_callback(void)
// {
//     encoder_update();  // 编码器计算
//     //line_follow_pid_control(); //pid控制
//     // printf("get_target_angle%f\n",get_target_angle());
//     // printf("get_steer:%f\n",get_steer());
//     // printf("get_gyro_z:%f\n",imu_dev.get_gyro_z()*0.01f);
//     // printf("\n");
// }

// int main(int, char**) 
// {
// //******************************初始化区*************************************
//     ips200.init(FB_PATH);

//     if(uvc_dev.init(UVC_PATH) < 0)
//     {
//         return -1;
//     }

//     imu_dev.init();
//     motor_Init();
    
// //******************************pit中断配置**********************************
//     system_pit.init_ms(100, system_pit_callback);
    
// //******************************pid参数配置**********************************
//     PID_Init(&TracePID,  0.08f, 0.0f,  0.005f,  100.0f,  0.0f);    // 图像→角度
//     PID_Init(&AnglePID,  0.0005f, 0.0f,  0.0f,  1.0f,  0.0f);    // 角速度环
//     PID_Init(&Speed_lPID,  15.0f, 0.1f,  0.0f,  700.0f,   10.0f);    // 速度环
//     PID_Init(&Speed_rPID,  15.0f, 0.1f,  0.0f,  700.0f,   10.0f);    // 速度环
//     PID_Init(&Delta_SpPID,  7.0f, 0.0f,  0.20f,  900.0f,   150.0f);    // 差速环

// //******************************主循环**********************************

//             while(1)
//             {
//                 if(uvc_dev.wait_image_refresh() == 0)
//                 {
//                     //system_delay_ms(100);
                    
//                     // if (t == 20)
//                     //     target_lspeed = target_rspeed = 12;
//                     // t++;  
//                     // if (t == 40)
//                     // {
//                     //     target_lspeed = 13;
//                     //     target_rspeed = 11;
//                     // }
//                     //     //target_lspeed = target_rspeed = 16;
//                     // if (t == 65)
//                     // {
//                     //     t = 0;
//                     //     target_lspeed = target_rspeed = 0;
//                     // }
//                     line_follow_pid_control(); //pid控制
//                     //printf("samples:%f, %f, %f\n", target_omega, left_PWM, right_PWM);//调试输出用
//                     image_process();
                    
//                 } 
// /*
//                 float v_left  = get_left_speed_mps();
//                 float v_right = get_right_speed_mps();
//                 float dist_left = get_left_distance();
//                 float dist_right = get_right_distance();
//                 float a_left  = get_left_accel();
//                 float a_right = get_right_accel();

//                 // 打印
//                 printf("=========================================\r\n");
//                 printf("速度L：%.3f m/s      R：%.3f m/s\r\n", v_left, v_right);
//                 printf("里程L：%.4f 米      R：%.4f 米\r\n", dist_left, dist_right);
//                 printf("加速度L：%.2f m/s²   R：%.2f m/s²\r\n", a_left, a_right);
//                 printf("=========================================\r\n\r\n");

//                 system_delay_ms(200);    */           
//   //              printf("=====================================\r\n");
//   //              printf("Roll   = %.2f °\r\n", eulerAngle.roll);    // 横滚
//  //               printf("Pitch  = %.2f °\r\n", eulerAngle.pitch);   // 俯仰
//   //              printf("Yaw    = %.2f °\r\n", eulerAngle.yaw);     // 偏航
//    //             printf("=====================================\r\n\r\n");  
//  //  printf("steer:%f",get_steer());
//  //  printf("target_angle:%f",get_target_angle());
//             }
//     }


// main.cpp




// 引入模型推理相关头文件（根据你的模型封装实际路径调整）
#include "zf_common_headfile.hpp"
#include "control.hpp"
#include "recognition.hpp"
zf_driver_pit system_pit;










void ips_display_rgb_image(const uint16_t* img, int img_w, int img_h)
{
    if(!img) return;

    // IPS200: 240x320 竖屏, RGB565 高字节在前(color_mode=1)
    // 摄像头 OpenCV COLOR_BGR2BGR565 输出也是高字节在前
    const int screen_w = 240;
    const int screen_h = 320;

    // 图像居中显示
    int start_x = (screen_w - img_w) / 2;
    int start_y = (screen_h - img_h) / 2;

    // 正确API: show_rgb565_image(x, y, img, src_w, src_h, dis_w, dis_h, color_mode)
    // color_mode: 0=低字节在前(默认), 1=高字节在前(凌瞳SCC8660摄像头)
    // OpenCV COLOR_BGR2BGR565 在 little-endian 龙芯上输出低字节在前 → 用 0
    // 如果颜色异常（偏紫/偏绿），改为 1 试试
    ips200.show_rgb565_image(start_x, start_y,
        img,
        img_w, img_h,      // 源图宽高
        img_w, img_h,      // 显示宽高（不缩放）
        0);                 // 低字节在前
}

// ================================================================
// 在屏幕上显示识别结果
// ================================================================
// void ips_display_result(const char* result, float confidence)
// {
//     if(!ips200.fb_ptr) return;

//     // 在屏幕底部显示结果（白色文字，黑色背景）
//     char buf[64];
//     snprintf(buf, sizeof(buf), "%s (%.1f%%)", result, confidence);
    
//     // 逐飞库的字符串显示函数（如果你的库有ips200.printf就用这个）
//     ips200.set_cursor(10, 280);
//     ips200.set_text_color(0xFFFF, 0x0000); // 白字黑底
//     ips200.printf("%s", buf);
// }
int main(int, char**) 
{
//******************************初始化区*************************************
    ips200.init(FB_PATH);

    if(uvc_dev.init(UVC_PATH) < 0)
    {
        return -1;
    }

    imu_dev.init();
    Mahony_Init();
    motor_Init();

    
    
//******************************pit中断配置**********************************
//     system_pit.init_ms(100, system_pit_callback);
// //******************************pid参数配置**********************************
    PID_Init(&TracePID,  0.08f, 0.0f,  0.005f,  100.0f,  0.0f);    // 图像→角度
    PID_Init(&AnglePID,  0.0005f, 0.0f,  0.0f,  1.0f,  0.0f);    // 角速度环
    PID_Init(&Speed_lPID,  15.0f, 0.1f,  0.0f,  700.0f,   10.0f);    // 速度环
    PID_Init(&Speed_rPID,  15.0f, 0.1f,  0.0f,  700.0f,   10.0f);    // 速度环
    PID_Init(&Delta_SpPID,  7.0f, 0.0f,  0.20f,  900.0f,   150.0f);    // 差速环

//******************************主循环**********************************
    while(1)
    {
        if(uvc_dev.wait_image_refresh() == 0)
        {
            // 安全检查：摄像头是否正常打开
            if (!uvc_dev.is_camera_opened()) {
                system_delay_ms(10);
                continue;
            }

            // 先获取灰度图验证帧有效（避免 OpenCV cvtColor BGR565 崩溃）
            uint8_t* gray_ptr = uvc_dev.get_gray_image_ptr();
            if (gray_ptr == nullptr) {
                printf("⚠️ 灰度帧为空，跳过\n");
                system_delay_ms(10);
                continue;
            }

            // 获取RGB图像（灰度验证通过后才获取）
            const uint16_t* image_data = uvc_dev.get_rgb_image_ptr();
            if (image_data == nullptr) {
                printf("⚠️ RGB帧为空，跳过\n");
                continue;
            }

            // 每10帧显示一次（降低OpenCV + 显示负载）
            static int display_frame = 0;
            display_frame++;
            if (display_frame % 10 == 0) {
                ips_display_rgb_image(image_data, IMAGE_W, IMAGE_H);
            }

            // 图像分类
            //image_process();
            run_recognition();
            line_follow_pid_control(); //pid控制
        }
    }
}

// ====================== 以下为模型接口的示例实现（需替换为你的实际模型逻辑） ======================
// 示例：模型初始化（你需替换为实际的模型加载逻辑，比如加载onnx/tflite模型）





