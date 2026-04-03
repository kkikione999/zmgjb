#ifndef __QMC5883P_H__
#define __QMC5883P_H__
#include "main.h"
#include "i2c.h"
#include "QMC5883P_reg.h" 



typedef enum 
{
  QMC_OK       = 0x00U,
  QMC_ERROR    = 0x01U
} QMC_StatusTypeDef;

typedef struct qmc_raw_data
{
	int16_t X;
	int16_t Y;
	int16_t Z;
}QMC5883P_Raw_Data_t;
typedef struct qmc_data
{
	float X_Guass;
	float Y_Guass;
	float Z_Guass;
}QMC5883P_Data_t;

// 磁力计硬铁校准数据结构
typedef struct qmc_calibration
{
	int16_t offset_x;   // X轴硬铁偏移
	int16_t offset_y;   // Y轴硬铁偏移
	int16_t offset_z;   // Z轴硬铁偏移
	uint8_t is_calibrated;  // 校准标志
}QMC5883P_Calibration_t;

// 磁力计软铁校准数据结构（椭球校正）
typedef struct qmc_soft_iron_calibration
{
	float scale_x;      // X轴缩放因子
	float scale_y;      // Y轴缩放因子
	float scale_z;      // Z轴缩放因子
	uint8_t is_calibrated;  // 软铁校准标志
}QMC5883P_SoftIron_Calibration_t;

// 完整校准数据结构（硬铁+软铁矩阵）
typedef struct qmc_full_calibration
{
	float offset[3];               // 硬铁偏移 (X, Y, Z)
	float soft_matrix[3][3];       // 软铁校准矩阵
	uint8_t is_hard_iron_calibrated;
	uint8_t is_soft_iron_calibrated;
}QMC5883P_Full_Calibration_t;



QMC_StatusTypeDef QMC_ReadChipID(uint8_t* ChipID);
QMC_StatusTypeDef QMC_Init(void);
QMC_StatusTypeDef QMC_Read_Raw_DATA(QMC5883P_Raw_Data_t* data);
QMC_StatusTypeDef QMC_Read_DATA(QMC5883P_Data_t* data);
void QMC_Read_REG_DATA(uint8_t REG_ADDR, uint8_t *data);

// ========== 磁力计校准功能 ==========
/**
 * @brief 执行磁力计校准（硬铁校准）
 * @param duration_seconds 校准持续时间（秒），建议30-60秒
 * @note 在校准期间，需要缓慢旋转飞机做8字形运动，覆盖所有方向
 */
void QMC_Calibrate(uint32_t duration_seconds);

/**
 * @brief 获取当前校准数据
 * @param calib 校准数据结构指针
 */
void QMC_Get_Calibration(QMC5883P_Calibration_t* calib);

/**
 * @brief 设置校准数据（用于加载保存的校准值）
 * @param calib 校准数据结构指针
 */
void QMC_Set_Calibration(const QMC5883P_Calibration_t* calib);

/**
 * @brief 读取校准后的原始数据（已应用偏移）
 * @param data 原始数据结构指针
 */
QMC_StatusTypeDef QMC_Read_Calibrated_Raw_DATA(QMC5883P_Raw_Data_t* data);

/**
 * @brief 重置校准数据
 */
void QMC_Reset_Calibration(void);

/**
 * @brief 打印当前校准状态
 */
void QMC_Print_Calibration_Info(void);

// ========== 软铁校准功能（高级） ==========
/**
 * @brief 执行完整校准（硬铁+软铁）
 * @param duration_seconds 校准持续时间（秒），建议60秒
 * @note 需要充分旋转飞机覆盖所有方向
 */
void QMC_Calibrate_Full(uint32_t duration_seconds);

/**
 * @brief 获取完整校准数据（硬铁+软铁）
 */
void QMC_Get_Full_Calibration(QMC5883P_Full_Calibration_t* calib);

/**
 * @brief 设置完整校准数据
 */
void QMC_Set_Full_Calibration(const QMC5883P_Full_Calibration_t* calib);

/**
 * @brief 读取完整校准后的数据（硬铁+软铁）
 */
QMC_StatusTypeDef QMC_Read_Full_Calibrated_DATA(QMC5883P_Raw_Data_t* data);

/**
 * @brief 打印完整校准信息
 */
void QMC_Print_Full_Calibration_Info(void);



#endif
