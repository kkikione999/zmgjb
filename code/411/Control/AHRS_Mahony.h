#ifndef AHRS_MAHONY_H__
#define AHRS_MAHONY_H__
#include <stdint.h>
#include <math.h>
#include "main.h"

typedef struct icm42688_acc_raw_data_t ICM42688_Acc_Raw_Data_t;
typedef struct icm42688_gyro_raw_data_t ICM42688_Gyro_Raw_Data_t;
typedef struct qmc_raw_data QMC5883P_Raw_Data_t;

// ========== 调试阶段定义 ==========
typedef enum {
    DEBUG_STAGE_0_SENSOR_CHECK = 0,    // 第0步：检查传感器原始数据
    DEBUG_STAGE_1_COORDINATE_CHECK,     // 第1步：确定坐标系关系
    DEBUG_STAGE_2_ACCEL_ONLY,          // 第2步：仅用加速度计计算Roll/Pitch
    DEBUG_STAGE_3_ACCEL_MAG_YAW,       // 第3步：加速度计+磁力计计算Yaw
    DEBUG_STAGE_4_FULL_MAHONY          // 第4步：完整Mahony融合算法
} AHRS_Debug_Stage_t;

// Mahony算法参数结构
typedef struct {
    float kp;          // 比例增益（用于加速度计融合）
    float ki;          // 积分增益（用于消除陀螺仪漂移）
    float kp_mag;      // 磁力计比例增益（用于磁力计融合）
} mahony_params_t;

// 坐标系变换矩阵（用于第1步调试）
typedef struct {
    int8_t acc_x_sign;   // 加速度计X轴符号 (+1 或 -1)
    int8_t acc_y_sign;
    int8_t acc_z_sign;
    int8_t gyro_x_sign;  // 陀螺仪X轴符号
    int8_t gyro_y_sign;
    int8_t gyro_z_sign;
    int8_t mag_x_sign;   // 磁力计X轴符号
    int8_t mag_y_sign;
    int8_t mag_z_sign;
    // 坐标轴映射：0=X, 1=Y, 2=Z
    uint8_t acc_axis_map[3];   // [机体X对应的传感器轴, 机体Y, 机体Z]
    uint8_t gyro_axis_map[3];
    uint8_t mag_axis_map[3];
} coordinate_transform_t;

// 传感器校验数据结构
typedef struct {
    int16_t acc_raw[3];   // 加速度计原始数据
    int16_t gyro_raw[3];  // 陀螺仪原始数据
    int16_t mag_raw[3];   // 磁力计原始数据
    float acc_g[3];       // 加速度计（g单位）
    float gyro_dps[3];    // 陀螺仪（deg/s单位）
    float mag_gauss[3];   // 磁力计（Gauss单位）
} sensor_check_data_t;

// ========== 公共API ==========

/** 初始化Mahony AHRS算法 */
void mahony_ahrs_init(float kp, float ki);

/** 重置AHRS状态 */
void mahony_ahrs_reset(void);

/** 设置调试阶段（0-4） */
void mahony_set_debug_stage(AHRS_Debug_Stage_t stage);

/** 获取当前调试阶段 */
AHRS_Debug_Stage_t mahony_get_debug_stage(void);

/** 设置坐标系变换参数（用于第1步调试） */
void mahony_set_coordinate_transform(const coordinate_transform_t *transform);

/** 获取坐标系变换参数 */
void mahony_get_coordinate_transform(coordinate_transform_t *transform);

/** 更新AHRS（包含磁力计） */
void mahony_ahrs_update_mag(const ICM42688_Acc_Raw_Data_t *acc_raw, 
                             const ICM42688_Gyro_Raw_Data_t *gyro_raw,
                             const QMC5883P_Raw_Data_t *mag_raw);

/** 更新AHRS（不含磁力计，仅IMU） */
void mahony_ahrs_update(const ICM42688_Acc_Raw_Data_t *acc_raw, 
                       const ICM42688_Gyro_Raw_Data_t *gyro_raw);

/** 获取欧拉角（单位：度） */
void mahony_get_euler(float *roll_deg, float *pitch_deg, float *yaw_deg);

/** 获取四元数 */
void mahony_get_quaternion(float *oq0, float *oq1, float *oq2, float *oq3);

/** 设置Mahony参数（用于第4步调试） */
void mahony_set_params(float kp, float ki, float kp_mag);

/** 获取传感器校验数据（用于第0步调试） */
void mahony_get_sensor_check_data(sensor_check_data_t *data);

/** 打印调试信息（需要实现printf） */
void mahony_print_debug_info(void);

#endif

