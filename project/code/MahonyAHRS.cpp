#include "zf_common_headfile.hpp"
#include <chrono>

//-------------------------------------------------------------------------------------------
float twoKi;
float q0, q1, q2, q3;
float integralFBx, integralFBy, integralFBz;
float invSampleFreq;
float roll_mahony, pitch_mahony, yaw_mahony;

int16 imu_acc_x = 0,imu_acc_y = 0,imu_acc_z = 0;
int16 imu_gyro_x = 0,imu_gyro_y = 0,imu_gyro_z = 0;
int16 imu_mag_x = 0,imu_mag_y = 0,imu_mag_z = 0;

icm_param_t icm_data;
euler_param_t eulerAngle;


#define twoKpDef	(2.0f * 1.0f)
#define twoKiDef	(2.0f * 0.0f)

// ===========================================================================================
// 安装误差与零偏校准使用的全局变量
// ===========================================================================================
float q0_ref = 1.0f;
float q1_ref = 0.0f;
float q2_ref = 0.0f;
float q3_ref = 0.0f;

float   mahony_calibrate_elapsed = 0.0f;   // 已校准时间（秒）
uint8_t mahony_is_calibrated = 0;          // 是否已经完成校准标志位

// yaw 解缠状态
static float last_yaw_raw = 0;
static float yaw_accum    = 0;
static bool  first_yaw    = true;

//-------------------------------------------------------------------------------------------
// 快速开方函数（不需要修改）
float Mahony_invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    y = y * (1.5f - (halfx * y * y));
    return y;
}

void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc, qd;

    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        recipNorm = Mahony_invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        if(twoKi > 0.0f)
        {
            integralFBx += twoKi * halfex  * invSampleFreq;
            integralFBy += twoKi * halfey  * invSampleFreq;
            integralFBz += twoKi * halfez  * invSampleFreq;
            gx += integralFBx;
            gy += integralFBy;
            gz += integralFBz;
        }
        else
        {
            integralFBx = 0.0f;
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        gx += twoKpDef * halfex;
        gy += twoKpDef * halfey;
        gz += twoKpDef * halfez;
    }

    gx *= (0.5f  * invSampleFreq);
    gy *= (0.5f  * invSampleFreq);
    gz *= (0.5f  * invSampleFreq);

    qa = q0;
    qb = q1;
    qc = q2;
    qd = q3;

    q0 += (-qb * gx - qc * gy - qd * gz);
    q1 += (qa * gx + qc * gz - qd * gy);
    q2 += (qa * gy - qb * gz + qd * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    recipNorm = Mahony_invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

//-------------------------------------------------------------------------------------------
// 姿态归零函数（将当前四元数记为基准坐标系）
//-------------------------------------------------------------------------------------------
void Mahony_ResetZero(void)
{
    q0_ref = q0;
    q1_ref = q1;
    q2_ref = q2;
    q3_ref = q3;

    // 同步重置 yaw 解缠
    yaw_accum = 0;
    last_yaw_raw = 0;
    first_yaw = true;
}

//-------------------------------------------------------------------------------------------
// 四元数转欧拉角（已包含四元数乘法的安装误差/零偏补偿）
//-------------------------------------------------------------------------------------------
void QuaternionsToEulerAngle(void)
{
    // 1. 获取基准四元数的逆（共轭四元数）
    float inv_q0 =  q0_ref;
    float inv_q1 = -q1_ref;
    float inv_q2 = -q2_ref;
    float inv_q3 = -q3_ref;

    // 2. 四元数乘法：q_out = inv_q_ref ⊗ q_current
    // 这一步在三维空间中去除了初始的安装倾角与零偏
    float out_q0 = inv_q0*q0 - inv_q1*q1 - inv_q2*q2 - inv_q3*q3;
    float out_q1 = inv_q0*q1 + inv_q1*q0 + inv_q2*q3 - inv_q3*q2;
    float out_q2 = inv_q0*q2 - inv_q1*q3 + inv_q2*q0 + inv_q3*q1;
    float out_q3 = inv_q0*q3 + inv_q1*q2 - inv_q2*q1 + inv_q3*q0;

    // 3. 用去除了安装误差的相对四元数 (out_q) 计算输出的欧拉角
    // 横滚
    roll_mahony = asinf(-2.0f * (out_q1*out_q3 - out_q0*out_q2));
    eulerAngle.roll = roll_mahony * 57.29578f;

    // 俯仰
    pitch_mahony = atan2f(2.0f * (out_q0*out_q1 + out_q2*out_q3), 1.0f - 2.0f * (out_q1*out_q1 + out_q2*out_q2));
    eulerAngle.pitch = pitch_mahony * 57.29578f;

    // 偏航（±180°，会跳变）
    yaw_mahony = atan2f(2.0f * (out_q1*out_q2 + out_q0*out_q3), 1.0f - 2.0f * (out_q2*out_q2 + out_q3*out_q3));
    eulerAngle.yaw = yaw_mahony * 57.29578f;

    // 连续解缠 yaw（越过 ±180° 不跳变，累加真实转角）
    if (first_yaw)
    {
        yaw_accum = eulerAngle.yaw;
        first_yaw = false;
    }
    else
    {
        float delta = eulerAngle.yaw - last_yaw_raw;
        if (delta > 180.0f)       delta -= 360.0f;
        else if (delta < -180.0f) delta += 360.0f;
        yaw_accum += delta;
    }
    last_yaw_raw = eulerAngle.yaw;
    eulerAngle.yaw_cont = yaw_accum;
}

//-------------------------------------------------------------------------------------------
// 【对外接口】给逐飞调用，计算姿态
//-------------------------------------------------------------------------------------------
void Mahony_GetAngles(void)
{
    // 直接使用 icm_data 里的数据
    MahonyAHRSupdateIMU(
        icm_data.gyro_x,
        icm_data.gyro_y,
        icm_data.gyro_z,
        icm_data.acc_x,
        icm_data.acc_y,
        icm_data.acc_z
    );
    // 内部包含了基于基准四元数的补偿计算
    QuaternionsToEulerAngle();
}

//-------------------------------------------------------------------------------------------
// 初始化
//-------------------------------------------------------------------------------------------
void Get_InitAngle(float ax, float ay, float az)
{
    float recipNorm = Mahony_invSqrt(ax*ax + ay*ay + az*az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    float roll_init = atan2f(ay, az);
    float pitch_init = atan2f(-ax, sqrtf(ay*ay + az*az));

    float cr = cosf(roll_init * 0.5f);
    float sr = sinf(roll_init * 0.5f);
    float cp = cosf(pitch_init * 0.5f);
    float sp = sinf(pitch_init * 0.5f);

    q0 = cr * cp;
    q1 = sr * cp;
    q2 = cr * sp;
    q3 = -sr * sp;

    recipNorm = Mahony_invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

void Mahony_Init(void)
{
    twoKi = twoKiDef;
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;

    integralFBx = 0.0f;
    integralFBy = 0.0f;
    integralFBz = 0.0f;
    invSampleFreq = 0.02f;  // 默认 50Hz，首帧会被 chrono dt 覆盖

    // 时间基准校准（用实际 dt，sampleFrequency 仅作初始 dt 参考）
    mahony_calibrate_elapsed = 0.0f;
    mahony_is_calibrated = 0;

    // 初始化时没有安装误差补偿（基准为标准重力坐标系）
    q0_ref = 1.0f;
    q1_ref = 0.0f;
    q2_ref = 0.0f;
    q3_ref = 0.0f;
}

void Mahony_update(void)
{
    // ---- 计算实际 dt（替代固定 invSampleFreq）----
    static auto last_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    float dt = std::chrono::duration<float>(now - last_time).count();
    last_time = now;

    // 首帧 / 异常帧保护：默认 20ms（50Hz）
    if (dt <= 0.0f || dt > 0.1f) dt = 0.02f;

    // ---- 读取逐飞 IMU 原始数据 ----
    imu_acc_x = imu_dev.get_acc_x();
    imu_acc_y = imu_dev.get_acc_y();
    imu_acc_z = imu_dev.get_acc_z();

    imu_gyro_x = imu_dev.get_gyro_x();
    imu_gyro_y = imu_dev.get_gyro_y();
    imu_gyro_z = imu_dev.get_gyro_z();

    // ---- 把逐飞数据 → 填入 icm_data ----
    icm_data.acc_x = imu_acc_x / 2048.0f;
    icm_data.acc_y = imu_acc_y / 2048.0f;
    icm_data.acc_z = imu_acc_z / 2048.0f;

    icm_data.gyro_x = imu_gyro_x * 0.001065f;
    icm_data.gyro_y = imu_gyro_y * 0.001065f;
    icm_data.gyro_z = imu_gyro_z * 0.001065f;

    // ---- 用实际 dt 驱动积分 ----
    invSampleFreq = dt;

    // ---- 计算姿态 ----
    Mahony_GetAngles();

    // ---- 5秒时间校准（累积 dt，不受帧率影响）----
    if (mahony_is_calibrated == 0)
    {
        mahony_calibrate_elapsed += dt;

        eulerAngle.roll     = 0.0f;
        eulerAngle.pitch    = 0.0f;
        eulerAngle.yaw      = 0.0f;
        eulerAngle.yaw_cont = 0.0f;

        if (mahony_calibrate_elapsed >= 5.0f)
        {
            Mahony_ResetZero();
            mahony_is_calibrated = 1;
        }
    }
}

//-------------------------------------------------------------------------------------------
// 运行中途手动归零函数
// 适用场景：运行一段时间后出现严重漂移，通过按键或遥控指令触发，将当前姿态强行归零
//-------------------------------------------------------------------------------------------
void Mahony_Manual_ResetZero(void)
{
    // 1. 将当前的绝对四元数记录为新的基准（消除当前偏角）
    q0_ref = q0;
    q1_ref = q1;
    q2_ref = q2;
    q3_ref = q3;

    // 2. 【关键步骤】清空 Mahony 算法内部的积分误差
    // 防止之前积累的陀螺仪零飘积分继续拉扯现在的姿态，导致归零后继续快速漂移
    integralFBx = 0.0f;
    integralFBy = 0.0f;
    integralFBz = 0.0f;

    // 3. 顺便把当前输出的欧拉角强制覆写为 0
    eulerAngle.roll = 0.0f;
    eulerAngle.pitch = 0.0f;
    eulerAngle.yaw = 0.0f;
    eulerAngle.yaw_cont = 0.0f;

    // 同步重置 yaw 解缠
    yaw_accum = 0;
    last_yaw_raw = 0;
    first_yaw = true;
}

uint8_t Is_Mahony_Ready(void)
{
    return mahony_is_calibrated;
}
