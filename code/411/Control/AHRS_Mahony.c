/*
本文件是是一个基于 Mahony 算法的姿态航向参考系统（AHRS）实现，用于通过传感器数据（加速度计、陀螺仪、磁力计）估计设备的3D姿态。
__主要算法__: Mahony AHRS 算法，一种互补滤波器，用于融合多传感器数据进行姿态估计。
__支持的传感器__:
- ICM42688: 6轴IMU（3轴加速度计 + 3轴陀螺仪）
- QMC5883P: 3轴磁力计

*/
#include "AHRS_Mahony.h"
#include "ICM42688.h"
#include "QMC5883P.h"
#include <stdio.h>
#include <string.h>

// 数学常量
#define DEG2RAD 0.01745329251994329577f
#define RAD2DEG 57.295779513082320876f
#define PI 3.14159265358979323846f

// 传感器分辨率（从ICM42688.c中的全局变量）
extern float gyro_curr_resolution;   // dps / count
extern float accel_curr_resolution;  // g / count

// 磁力计分辨率
#define MAG_RESOLUTION 0.000244140625f  // 8Gauss range

// ========== 调试状态变量 ==========
static AHRS_Debug_Stage_t s_debug_stage = DEBUG_STAGE_0_SENSOR_CHECK;
static sensor_check_data_t s_sensor_data = {0};

// 坐标系变换系统
// IMU坐标系 → 机体坐标系映射（已确认）：
// IMU.X = -飞机Y → 机体X从IMU.Y轴获取, 符号为正
// IMU.Y = 飞机X  → 机体Y从IMU.X轴获取, 符号为负
// IMU.Z = 飞机Z  → 机体Z从IMU.Z轴获取, 符号为负
// - 处理IMU传感器坐标系到机体坐标系的映射
// - 包含轴重映射和符号校正

static coordinate_transform_t s_coord_transform = 
{
    .acc_x_sign = -1, .acc_y_sign = -1, .acc_z_sign = 1,      // 
    .gyro_x_sign = -1, .gyro_y_sign = -1, .gyro_z_sign = -1,    // 
    .mag_x_sign = -1, .mag_y_sign = 1, .mag_z_sign = -1,        // 
    .acc_axis_map = {1, 0, 2},   
    .gyro_axis_map = {1, 0, 2},  // 机体X←-IMU.Y, 机体Y←IMU.X, 机体Z←IMU.Z
    .mag_axis_map = {0, 1, 2}    
};

// Mahony算法参数
static float s_sample_period = 0.002f;  // 500Hz默认采样率
static float s_kp = 0.8f;               // 比例增益
static float s_ki = 0.001f;             // 积分增益
static float s_kp_mag = 0.2f;           // 磁力计增益 (优化: 0.05~0.2范围)

// 四元数状态
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

// 欧拉角缓存
static float s_roll_deg = 0.0f;
static float s_pitch_deg = 0.0f;
static float s_yaw_deg = 0.0f;

// 积分项（用于消除陀螺仪漂移）
static float integral_fbx = 0.0f, integral_fby = 0.0f, integral_fbz = 0.0f;

// ========== 辅助函数 ==========

/**
 * @brief 快速倒数平方根算法
 */
static inline float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    uint32_t i;
    memcpy(&i, &y, sizeof(i));
    i = 0x5f3759df - (i >> 1);
    memcpy(&y, &i, sizeof(y));
    y = y * (1.5f - (halfx * y * y));
    return y;
}

/**
 * @brief 坐标系变换：根据映射表和符号转换传感器数据
 */
static void apply_coordinate_transform(const int16_t raw[3], float transformed[3], 
                                       const uint8_t axis_map[3], 
                                       int8_t sign_x, int8_t sign_y, int8_t sign_z,
                                       float resolution)
{
    int8_t signs[3] = {sign_x, sign_y, sign_z};
    // 机体坐标X轴 = 传感器坐标axis_map[0]轴 * 符号
    transformed[0] = (float)raw[axis_map[0]] * signs[0] * resolution;
    transformed[1] = (float)raw[axis_map[1]] * signs[1] * resolution;
    transformed[2] = (float)raw[axis_map[2]] * signs[2] * resolution;
}

/**
 * @brief 四元数归一化
 */
static inline void quaternion_normalize(float *q0, float *q1, float *q2, float *q3)
{
    float norm = invSqrt((*q0)*(*q0) + (*q1)*(*q1) + (*q2)*(*q2) + (*q3)*(*q3));
    *q0 *= norm;
    *q1 *= norm;
    *q2 *= norm;
    *q3 *= norm;
}

/**
 * @brief 四元数转欧拉角（ZYX顺序 - NED坐标系）
 */
static inline void quaternion_to_euler(float q0, float q1, float q2, float q3, 
                                       float *roll, float *pitch, float *yaw)
{
    // Roll (x-axis rotation)
    float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    *roll = atan2f(sinr_cosp, cosr_cosp) * RAD2DEG;

    // Pitch (y-axis rotation)
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabsf(sinp) >= 1.0f)
        *pitch = copysignf(1.5707963267948966f, sinp) * RAD2DEG;
    else
        *pitch = asinf(sinp) * RAD2DEG;

    // Yaw (z-axis rotation)
    float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    *yaw = atan2f(siny_cosp, cosy_cosp) * RAD2DEG;
}

/**
 * @brief 第2步：仅使用加速度计计算Roll和Pitch
 */
static __attribute__((unused)) void calc_roll_pitch_from_accel(
    float ax, float ay, float az, float *roll, float *pitch
)
{
    // Roll = atan2(ay, az)
    *roll = atan2f(ay, az) * RAD2DEG;
    
    // Pitch = atan2(-ax, sqrt(ay*ay + az*az))
    *pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD2DEG;
}

/**
 * @brief 第3步：使用加速度计和磁力计计算Yaw
 * 前提：Roll和Pitch已知
 */
static __attribute__((unused)) void calc_yaw_from_mag(
    float roll_rad, float pitch_rad, float mx, float my, float mz, float *yaw
)
{
    // 【调试】打印输入数据
    static uint32_t debug_count = 0;
    if (debug_count++ % 50 == 0) {  // 每50次打印一次
        printf("[Yaw Debug] mx=%.4f, my=%.4f, mz=%.4f\r\n", mx, my, mz);
        printf("           roll=%.2f deg, pitch=%.2f deg\r\n", 
               roll_rad * RAD2DEG, pitch_rad * RAD2DEG);
    }
    
    // 将磁力计数据补偿到水平面
    float cos_roll = cosf(roll_rad);
    float sin_roll = sinf(roll_rad);
    float cos_pitch = cosf(pitch_rad);
    float sin_pitch = sinf(pitch_rad);
    
    // 旋转到水平面
    float mag_x_h = mx * cos_pitch + my * sin_roll * sin_pitch + mz * cos_roll * sin_pitch;
    float mag_y_h = my * cos_roll - mz * sin_roll;
    
    // 【调试】打印水平分量
    if (debug_count % 50 == 1) {
        printf("           mag_x_h=%.4f, mag_y_h=%.4f\r\n", mag_x_h, mag_y_h);
    }
    
    // Yaw = atan2(-mag_y_h, mag_x_h)
    *yaw = atan2f(-mag_y_h, mag_x_h) * RAD2DEG;
    
    // 【调试】打印计算结果
    if (debug_count % 50 == 1) {
        printf("           yaw=%.2f deg\r\n\r\n", *yaw);
    }
}

/**
 * @brief Mahony AHRS融合算法（仅IMU版本）
 * 使用陀螺仪和加速度计计算Roll/Pitch
 */
static void mahony_update_imu(float gx, float gy, float gz,
                               float ax, float ay, float az)
{
    // 归一化加速度计测量值
    float recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    if (recipNorm == 0.0f) return;
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;
    
    // 从当前四元数计算期望的重力方向
    float vx = 2.0f * (q1 * q3 - q0 * q2);
    float vy = 2.0f * (q0 * q1 + q2 * q3);
    float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
    
    // 计算误差（用叉积表示）
    float ex = ay * vz - az * vy;
    float ey = az * vx - ax * vz;
    float ez = ax * vy - ay * vx;
    
    // 积分项（用于消除陀螺仪漂移）
    integral_fbx += ex * s_sample_period;
    integral_fby += ey * s_sample_period;
    integral_fbz += ez * s_sample_period;
    
    // 限制积分项（防止积分饱和）
    if (integral_fbx > 1.0f) integral_fbx = 1.0f;
    if (integral_fbx < -1.0f) integral_fbx = -1.0f;
    if (integral_fby > 1.0f) integral_fby = 1.0f;
    if (integral_fby < -1.0f) integral_fby = -1.0f;
    if (integral_fbz > 1.0f) integral_fbz = 1.0f;
    if (integral_fbz < -1.0f) integral_fbz = -1.0f;
    
    // 用PI控制器修正陀螺仪
    gx += s_kp * ex + s_ki * integral_fbx;
    gy += s_kp * ey + s_ki * integral_fby;
    gz += s_kp * ez + s_ki * integral_fbz;
    
    // 四元数微分方程（陀螺仪积分）
    float q0t = q0, q1t = q1, q2t = q2, q3t = q3;
    
    q0 = q0t + (-q1t * gx - q2t * gy - q3t * gz) * 0.5f * s_sample_period;
    q1 = q1t + ( q0t * gx + q2t * gz - q3t * gy) * 0.5f * s_sample_period;
    q2 = q2t + ( q0t * gy - q1t * gz + q3t * gx) * 0.5f * s_sample_period;
    q3 = q3t + ( q0t * gz + q1t * gy - q2t * gx) * 0.5f * s_sample_period;
    
    // 归一化四元数
    quaternion_normalize(&q0, &q1, &q2, &q3);
}

/**
 * @brief Mahony AHRS融合算法（包含磁力计版本 - 完整版）
 * 使用陀螺仪、加速度计和磁力计计算完整姿态
 */
static void mahony_update_mag(float gx, float gy, float gz,
                               float ax, float ay, float az,
                               float mx, float my, float mz)
{
    // 归一化加速度计测量值
    float recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    if (recipNorm == 0.0f) return;
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;
    
    // 归一化磁力计数据
    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    if (recipNorm == 0.0f) return;
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;
    
    // 从当前四元数计算期望的重力方向（机体坐标系）
    float vx = 2.0f * (q1 * q3 - q0 * q2);
    float vy = 2.0f * (q0 * q1 + q2 * q3);
    float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
    
    // 加速度计误差（重力方向）
    float ex = ay * vz - az * vy;
    float ey = az * vx - ax * vz;
    float ez = ax * vy - ay * vx;
    
    // 将磁力计从机体坐标系转换到地球坐标系
    float hx = 2.0f * mx * (0.5f - q2*q2 - q3*q3) + 2.0f * my * (q1*q2 - q0*q3) + 2.0f * mz * (q1*q3 + q0*q2);
    float hy = 2.0f * mx * (q1*q2 + q0*q3) + 2.0f * my * (0.5f - q1*q1 - q3*q3) + 2.0f * mz * (q2*q3 - q0*q1);
    
    // 计算参考磁场方向（水平分量）
    float bx = sqrtf(hx * hx + hy * hy);
    float bz = 2.0f * mx * (q1*q3 - q0*q2) + 2.0f * my * (q2*q3 + q0*q1) + 2.0f * mz * (0.5f - q1*q1 - q2*q2);
    
    // 估计的磁场方向（机体坐标系）
    float wx = 2.0f * bx * (0.5f - q2*q2 - q3*q3) + 2.0f * bz * (q1*q3 - q0*q2);
    float wy = 2.0f * bx * (q1*q2 - q0*q3) + 2.0f * bz * (q0*q1 + q2*q3);
    float wz = 2.0f * bx * (q0*q2 + q1*q3) + 2.0f * bz * (0.5f - q1*q1 - q2*q2);
    
    // 磁力计误差
    ex += s_kp_mag * (my * wz - mz * wy);
    ey += s_kp_mag * (mz * wx - mx * wz);
    ez += s_kp_mag * (mx * wy - my * wx);
    
    // 积分误差
    if (s_ki > 0.0f) {
        integral_fbx += ex * s_sample_period;
        integral_fby += ey * s_sample_period;
        integral_fbz += ez * s_sample_period;
        
        // 限制积分项
        if (integral_fbx > 1.0f) integral_fbx = 1.0f;
        if (integral_fbx < -1.0f) integral_fbx = -1.0f;
        if (integral_fby > 1.0f) integral_fby = 1.0f;
        if (integral_fby < -1.0f) integral_fby = -1.0f;
        if (integral_fbz > 1.0f) integral_fbz = 1.0f;
        if (integral_fbz < -1.0f) integral_fbz = -1.0f;
        
        gx += s_ki * integral_fbx;
        gy += s_ki * integral_fby;
        gz += s_ki * integral_fbz;
    }
    
    // PI控制器修正陀螺仪
    gx += s_kp * ex;
    gy += s_kp * ey;
    gz += s_kp * ez;
    
    // 四元数微分方程
    float q0t = q0, q1t = q1, q2t = q2, q3t = q3;
    
    q0 = q0t + (-q1t * gx - q2t * gy - q3t * gz) * 0.5f * s_sample_period;
    q1 = q1t + ( q0t * gx + q2t * gz - q3t * gy) * 0.5f * s_sample_period;
    q2 = q2t + ( q0t * gy - q1t * gz + q3t * gx) * 0.5f * s_sample_period;
    q3 = q3t + ( q0t * gz + q1t * gy - q2t * gx) * 0.5f * s_sample_period;
    
    // 归一化四元数
    quaternion_normalize(&q0, &q1, &q2, &q3);
}

// ========== 公共API ==========

void mahony_ahrs_init(float kp, float ki)
{
    s_kp = kp;
    s_ki = ki;
    s_sample_period = 0.002f;
    s_debug_stage = DEBUG_STAGE_0_SENSOR_CHECK;  // 默认从第0步开始
    mahony_ahrs_reset();
}

void mahony_ahrs_reset(void)
{
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
    
    s_roll_deg = 0.0f;
    s_pitch_deg = 0.0f;
    s_yaw_deg = 0.0f;
    
    integral_fbx = 0.0f;
    integral_fby = 0.0f;
    integral_fbz = 0.0f;
    
    memset(&s_sensor_data, 0, sizeof(s_sensor_data));
}

void mahony_set_debug_stage(AHRS_Debug_Stage_t stage)
{
    s_debug_stage = stage;
    mahony_ahrs_reset();  // 切换阶段时重置状态
    printf("[AHRS] 切换到调试阶段 %d\r\n", stage);
}

AHRS_Debug_Stage_t mahony_get_debug_stage(void)
{
    return s_debug_stage;
}

void mahony_set_coordinate_transform(const coordinate_transform_t *transform)
{
    if (transform) {
        memcpy(&s_coord_transform, transform, sizeof(coordinate_transform_t));
        printf("[AHRS] 坐标系变换参数已更新\r\n");
    }
}

void mahony_get_coordinate_transform(coordinate_transform_t *transform)
{
    if (transform) {
        memcpy(transform, &s_coord_transform, sizeof(coordinate_transform_t));
    }
}

void mahony_set_params(float kp, float ki, float kp_mag)
{
    s_kp = kp;
    s_ki = ki;
    s_kp_mag = kp_mag;
    printf("[AHRS] 参数已更新: Kp=%.3f, Ki=%.4f, Kp_mag=%.3f\r\n", kp, ki, kp_mag);
}

void mahony_get_sensor_check_data(sensor_check_data_t *data)
{
    if (data) {
        memcpy(data, &s_sensor_data, sizeof(sensor_check_data_t));
    }
}

void mahony_ahrs_update(const ICM42688_Acc_Raw_Data_t *acc_raw, 
                        const ICM42688_Gyro_Raw_Data_t *gyro_raw)
{
    mahony_ahrs_update_mag(acc_raw, gyro_raw, NULL);
}

void mahony_ahrs_update_mag(const ICM42688_Acc_Raw_Data_t *acc_raw, 
                             const ICM42688_Gyro_Raw_Data_t *gyro_raw,
                             const QMC5883P_Raw_Data_t *mag_raw)
{
    if (!acc_raw || !gyro_raw) {
        return;
    }
    
    // ========== 步骤0：保存原始数据用于检查 ==========
    int16_t acc_raw_arr[3] = {acc_raw->accel_x, acc_raw->accel_y, acc_raw->accel_z};
    int16_t gyro_raw_arr[3] = {gyro_raw->gyro_x, gyro_raw->gyro_y, gyro_raw->gyro_z};
    int16_t mag_raw_arr[3] = {0, 0, 0};
    if (mag_raw) {
        mag_raw_arr[0] = mag_raw->X;
        mag_raw_arr[1] = mag_raw->Y;
        mag_raw_arr[2] = mag_raw->Z;
    }
    
    // 保存原始数据
    memcpy(s_sensor_data.acc_raw, acc_raw_arr, sizeof(acc_raw_arr));
    memcpy(s_sensor_data.gyro_raw, gyro_raw_arr, sizeof(gyro_raw_arr));
    memcpy(s_sensor_data.mag_raw, mag_raw_arr, sizeof(mag_raw_arr));
    
    // ========== 步骤1：应用坐标系变换 ==========
    float acc_body[3], gyro_body[3], mag_body[3];
    
    apply_coordinate_transform(acc_raw_arr, acc_body, 
                              s_coord_transform.acc_axis_map,
                              s_coord_transform.acc_x_sign,
                              s_coord_transform.acc_y_sign,
                              s_coord_transform.acc_z_sign,
                              accel_curr_resolution);
    
    apply_coordinate_transform(gyro_raw_arr, gyro_body,
                              s_coord_transform.gyro_axis_map,
                              s_coord_transform.gyro_x_sign,
                              s_coord_transform.gyro_y_sign,
                              s_coord_transform.gyro_z_sign,
                              gyro_curr_resolution);
    
        
    apply_coordinate_transform(mag_raw_arr, mag_body,
                            s_coord_transform.mag_axis_map,
                            s_coord_transform.mag_x_sign,
                            s_coord_transform.mag_y_sign,
                            s_coord_transform.mag_z_sign,
                            MAG_RESOLUTION);
           
    // 保存转换后的数据
    memcpy(s_sensor_data.acc_g, acc_body, sizeof(acc_body));
    s_sensor_data.gyro_dps[0] = gyro_body[0];
    s_sensor_data.gyro_dps[1] = gyro_body[1];
    s_sensor_data.gyro_dps[2] = gyro_body[2];
    memcpy(s_sensor_data.mag_gauss, mag_body, sizeof(mag_body));
    
    float ax = acc_body[0];
    float ay = acc_body[1];
    float az = acc_body[2];
    float gx_dps = gyro_body[0];
    float gy_dps = gyro_body[1];
    float gz_dps = gyro_body[2];
    float mx = mag_body[0];
    float my = mag_body[1];
    float mz = mag_body[2];
    
    // 陀螺仪转换为rad/s
    float gx = gx_dps * DEG2RAD;
    float gy = gy_dps * DEG2RAD;
    float gz = gz_dps * DEG2RAD;
    

    // // 诊断：打印磁力计数据（每秒打印一次）
    // static uint32_t debug_counter = 0;
    // if (debug_counter++ % 500 == 0 && mag_raw) {
    //     printf("[AHRS诊断] mx=%.4f, my=%.4f, mz=%.4f\r\n", mx, my, mz);
    //     float mag_strength = sqrtf(mx*mx + my*my + mz*mz);
    //     printf("          磁场强度=%.4f (已归一化)\r\n", mag_strength);
    // }

    if (mag_raw) {
        mahony_update_mag(gx, gy, gz, ax, ay, az, mx, my, mz);
    } else {
        mahony_update_imu(gx, gy, gz, ax, ay, az);
    }
    quaternion_to_euler(q0, q1, q2, q3, &s_roll_deg, &s_pitch_deg, &s_yaw_deg);
}

void mahony_get_euler(float *roll_deg, float *pitch_deg, float *yaw_deg)
{
    if (roll_deg)  *roll_deg  = s_roll_deg;
    if (pitch_deg) *pitch_deg = s_pitch_deg;
    if (yaw_deg)   *yaw_deg   = s_yaw_deg;
}

void mahony_get_quaternion(float *oq0, float *oq1, float *oq2, float *oq3)
{
    if (oq0) *oq0 = q0;
    if (oq1) *oq1 = q1;
    if (oq2) *oq2 = q2;
    if (oq3) *oq3 = q3;
}

void mahony_print_debug_info(void)
{
//    printf("\r\n========== AHRS 调试信息 ==========\r\n");
//    printf("当前调试阶段: %d\r\n", s_debug_stage);
    
//    printf("\r\n--- 原始传感器数据 ---\r\n");
//    printf("加速度计原始值: X=%d, Y=%d, Z=%d\r\n", 
//           s_sensor_data.acc_raw[0], s_sensor_data.acc_raw[1], s_sensor_data.acc_raw[2]);
//    printf("陀螺仪原始值:   X=%d, Y=%d, Z=%d\r\n",
//           s_sensor_data.gyro_raw[0], s_sensor_data.gyro_raw[1], s_sensor_data.gyro_raw[2]);
//    printf("磁力计原始值:   X=%d, Y=%d, Z=%d\r\n",
//           s_sensor_data.mag_raw[0], s_sensor_data.mag_raw[1], s_sensor_data.mag_raw[2]);
//    
//    printf("\r\n--- 转换后的传感器数据（机体坐标系） ---\r\n");
//    printf("加速度计 (g):   X=%.3f, Y=%.3f, Z=%.3f\r\n",
//           s_sensor_data.acc_g[0], s_sensor_data.acc_g[1], s_sensor_data.acc_g[2]);
//    printf("陀螺仪 (dps):   X=%.2f, Y=%.2f, Z=%.2f\r\n",
//           s_sensor_data.gyro_dps[0], s_sensor_data.gyro_dps[1], s_sensor_data.gyro_dps[2]);
//    printf("磁力计 (Gauss): X=%.4f, Y=%.4f, Z=%.4f\r\n",
//           s_sensor_data.mag_gauss[0], s_sensor_data.mag_gauss[1], s_sensor_data.mag_gauss[2]);
    
    printf("\r\n--- 姿态角 ---\r\n");
		printf("Roll Pitch Yaw: %.2f° %.2f° %.2f°\n", s_roll_deg,s_pitch_deg,s_yaw_deg);

    
//    printf("\r\n--- 四元数 ---\r\n");
//    printf("q0=%.4f, q1=%.4f, q2=%.4f, q3=%.4f\r\n", q0, q1, q2, q3);
//    
//    printf("\r\n--- 参数 ---\r\n");
//    printf("Kp=%.3f, Ki=%.4f, Kp_mag=%.3f\r\n", s_kp, s_ki, s_kp_mag);
    
//    printf("==================================\r\n\r\n");
}

