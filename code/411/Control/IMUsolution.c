#include "IMUsolution.h"
#include "ICM42688.h"
// ========== 使用你工程里的 dt / halfT ==========
#define dt     0.002f
#define halfT  0.001f




// ========== 使用你工程里的全局分辨率（直接 extern） ==========
extern float accel_curr_resolution; // g / count
extern float gyro_curr_resolution;  // dps / count

// ========== 可按需调整的增益 ==========
#ifndef Kp
#define Kp  10.0f     // 重力修正的比例系数
#endif
#ifndef Ki
#define Ki  0.000f    // 重力修正的积分系数（先为0，跑稳再慢慢加）
#endif

// ========== 常量 ==========

#define DEG2RAD 0.01745329251994329577f
#define RAD2DEG 57.295779513082320876f

// ========== 模块内部状态 ==========
static IMUFusionAlgo s_algo = IMU_ALGO_COMPLEMENTARY_PI;

// 四元数（初始姿态 0/0/0）
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

// IMUupdate() 版本内部的积分项 & 欧拉角缓存
static float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;
static float euler_roll_deg_PI = 0.0f, euler_pitch_deg_PI = 0.0f, euler_yaw_deg_PI = 0.0f;

// algorithm() 版本的欧拉角缓存
static float euler_roll_deg_V2 = 0.0f, euler_pitch_deg_V2 = 0.0f, euler_yaw_deg_V2 = 0.0f;

// 陀螺零偏（rad/s）
static float bias_gx = 0.0f, bias_gy = 0.0f, bias_gz = 0.0f;

// ========== 内部工具 ==========
static inline float invSqrt_fast(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

//设置陀螺零偏（单位输入是 °/s，内部转为 rad/s 存储）。
static void imu_fusion_set_bias(int16_t gx, int16_t gy, int16_t gz)
{
    bias_gx = gx * DEG2RAD;
    bias_gy = gy * DEG2RAD;
    bias_gz = gz * DEG2RAD;
}

// ========== 互补滤波实现 1：IMUupdate（与你提供的一致，做了小幅整理） ==========
static void IMUupdate_PI(float gx, float gy, float gz, float ax, float ay, float az)
{
    float q0t = q0, q1t = q1, q2t = q2, q3t = q3;
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm < 1e-6f) return;  // 防止除零
    ax /= norm; ay /= norm; az /= norm;

    // 用当前姿态计算重力在机体系上的分量（方向余弦矩阵第三行）
    float q0q0 = q0*q0, q0q1 = q0*q1, q0q2 = q0*q2;
    float q1q1 = q1*q1, q1q3 = q1*q3, q2q2 = q2*q2, q2q3 = q2*q3, q3q3 = q3*q3;

    float vx = 2.0f * (q1q3 - q0q2);
    float vy = 2.0f * (q0q1 + q2q3);
    float vz = q0q0 - q1q1 - q2q2 + q3q3;

    // 叉积误差
    float ex = (ay * vz - az * vy);
    float ey = (az * vx - ax * vz);
    float ez = (ax * vy - ay * vx);

    // 积分项
    exInt += Ki * ex;
    eyInt += Ki * ey;
    ezInt += Ki * ez;

    // 误差补偿到角速度
    gx += Kp * ex + exInt;
    gy += Kp * ey + eyInt;
    gz += Kp * ez + ezInt;

    // 四元数微分方程（欧拉步积分）
    q0 = q0t + (-q1t*gx - q2t*gy - q3t*gz) * halfT;
    q1 = q1t + ( q0t*gx + q2t*gz - q3t*gy) * halfT;
    q2 = q2t + ( q0t*gy - q1t*gz + q3t*gx) * halfT;
    q3 = q3t + ( q0t*gz + q1t*gy - q2t*gx) * halfT;

    // 归一化
    norm = invSqrt_fast(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= norm; q1 *= norm; q2 *= norm; q3 *= norm;

    // 欧拉角（注意：yaw 这里仍主要靠陀螺积分，未用磁力计）
    euler_yaw_deg_PI   = euler_yaw_deg_PI + gz * halfT * 2.0f * RAD2DEG;
    euler_pitch_deg_PI = asinf(-2.0f*q1*q3 + 2.0f*q0*q2) * RAD2DEG;
    euler_roll_deg_PI  = atan2f(2.0f*q2*q3 + 2.0f*q0*q1, -2.0f*q1*q1 - 2.0f*q2*q2 + 1.0f) * RAD2DEG;
}

// ========== 互补滤波实现 2==========
static void IMUupdate_V2(float ax, float ay, float az, float gx, float gy, float gz)
{
    // 提取姿态矩阵中重力方向向量
    float Vx = 2.0f*(q1*q3 - q0*q2);
    float Vy = 2.0f*(q0*q1 + q2*q3);
    float Vz = 1.0f - 2.0f*q1*q1 - 2.0f*q2*q2;

    // 姿态误差
    float ex = ay*Vz - az*Vy;
    float ey = az*Vx - ax*Vz;
    float ez = ax*Vy - ay*Vx;

    // 积分项（使用 dt）
    static float accex = 0, accey = 0, accez = 0;
    accex += Ki * ex * dt;
    accey += Ki * ey * dt;
    accez += Ki * ez * dt;

    // 误差补偿
    gx += Kp * ex + accex;
    gy += Kp * ey + accey;
    gz += Kp * ez + accez;

    // 四元数积分
    float gxh = 0.5f * gx * dt;
    float gyh = 0.5f * gy * dt;
    float gzh = 0.5f * gz * dt;

    float q0t = q0, q1t = q1, q2t = q2, q3t = q3;
    q0 = q0t + (-q1t*gxh - q2t*gyh - q3t*gzh);
    q1 = q1t + ( q0t*gxh + q2t*gzh - q3t*gyh);
    q2 = q2t + ( q0t*gyh - q1t*gzh + q3t*gxh);
    q3 = q3t + ( q0t*gzh + q1t*gyh - q2t*gxh);

    // 归一化
    float recip = invSqrt_fast(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recip; q1 *= recip; q2 *= recip; q3 *= recip;

    // 欧拉角
    float g1 = 2.0f*(q1*q3 - q0*q2);
    float g2 = 2.0f*(q0*q1 + q2*q3);
    float g3 = q0*q0 - q1*q1 - q2*q2 + q3*q3;
    float g4 = 2.0f*(q1*q2 + q0*q3);
    float g5 = q0*q0 + q1*q1 - q2*q2 - q3*q3;

    euler_pitch_deg_V2 = -asinf(g1) * RAD2DEG;
    euler_roll_deg_V2  =  atanf(g2 / g3) * RAD2DEG;
    euler_yaw_deg_V2   =  atanf(g4 / g5) * RAD2DEG;
}

// ========== 对外 API ==========
//选择算法
void imu_fusion_init(IMUSOLUTION_INIT_DATA *imu_init_data)
{
    s_algo = imu_init_data->algo;
    imu_fusion_reset();
		imu_fusion_set_bias(imu_init_data->static_gx, imu_init_data->static_gy, imu_init_data->static_gz);
}


void imu_fusion_reset(void)
{
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    exInt = eyInt = ezInt = 0.0f;
    euler_roll_deg_PI = euler_pitch_deg_PI = euler_yaw_deg_PI = 0.0f;
    euler_roll_deg_V2 = euler_pitch_deg_V2 = euler_yaw_deg_V2 = 0.0f;
}

void imu_fusion_process(const ICM42688_Acc_Raw_Data_t *accraw, const ICM42688_Gyro_Raw_Data_t *gyroraw)
{
    if (!accraw) return;
		if (!gyroraw) return;
		//printf("%d,%d,%d,%d,%d\n", raw->accel_x,raw->accel_y,raw->accel_z,raw->gyro_x, raw->gyro_y, raw->gyro_z);
		
    // 原始值 → 物理量
    float ax_g = (float)accraw->accel_x * accel_curr_resolution;  // g
    float ay_g = (float)accraw->accel_y * accel_curr_resolution;  // g
    float az_g = (float)accraw->accel_z * accel_curr_resolution;  // g

    float gx_dps = (float)gyroraw->gyro_x  * gyro_curr_resolution; // °/s
    float gy_dps = (float)gyroraw->gyro_y  * gyro_curr_resolution; // °/s
    float gz_dps = (float)gyroraw->gyro_z  * gyro_curr_resolution; // °/s

    // dps → rad/s，并减去零偏
    float gx = gx_dps * DEG2RAD - bias_gx;
    float gy = gy_dps * DEG2RAD - bias_gy;
    float gz = gz_dps * DEG2RAD - bias_gz;

    // 低重力/撞击时可选择跳过修正（可选）
    float anorm = sqrtf(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
    if (anorm < 0.2f) {
        // 近似失重：仅做陀螺积分。这里简单地调用各自函数即可，因为它们内部会使用陀螺积分。
        // 若希望严格跳过加速度修正，可在函数内部加开关；此处保持简单实现。
    }

    if (s_algo == IMU_ALGO_COMPLEMENTARY_PI) {
        // IMUupdate: gx/gy/gz (rad/s), ax/ay/az (g)
        IMUupdate_PI(gx, gy, gz, ax_g, ay_g, az_g);
    } else {
        // algorithm: ax/ay/az (g), gx/gy/gz (rad/s)
        IMUupdate_V2(ax_g, ay_g, az_g, gx, gy, gz);
    }
}

void imu_fusion_get_euler(float *roll_deg, float *pitch_deg, float *yaw_deg)
{
    if (s_algo == IMU_ALGO_COMPLEMENTARY_PI) {
        if (roll_deg)  *roll_deg  = euler_roll_deg_PI;
        if (pitch_deg) *pitch_deg = euler_pitch_deg_PI;
        if (yaw_deg)   *yaw_deg   = euler_yaw_deg_PI;
    } else {
        if (roll_deg)  *roll_deg  = euler_roll_deg_V2;
        if (pitch_deg) *pitch_deg = euler_pitch_deg_V2;
        if (yaw_deg)   *yaw_deg   = euler_yaw_deg_V2;
    }
}

void imu_fusion_get_quaternion(float *oq0, float *oq1, float *oq2, float *oq3)
{
    if (oq0) *oq0 = q0;
    if (oq1) *oq1 = q1;
    if (oq2) *oq2 = q2;
    if (oq3) *oq3 = q3;
}






