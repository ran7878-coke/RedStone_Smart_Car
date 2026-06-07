
#ifndef __MAHONYAHRS_HPP
#define __MAHONYAHRS_HPP

// 传感器数据结构体
typedef struct
{
    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} icm_param_t;

// 欧拉角结构体
typedef struct
{
    float roll;
    float pitch;
    float yaw;           // ±180°，会跳变
    float yaw_cont;      // 连续解缠值，可超过 ±180°
} euler_param_t;

extern icm_param_t icm_data;
extern euler_param_t eulerAngle;
extern zf_device_imu imu_dev;
// 接口函数
void Mahony_Init(void);
void Mahony_GetAngles(void);
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
void QuaternionsToEulerAngle(void);
void Mahony_update(void);
void Mahony_ResetZero(void);
void Mahony_Manual_ResetZero(void);
uint8_t Is_Mahony_Ready(void);

#endif
