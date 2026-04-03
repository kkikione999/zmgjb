#ifndef ICM42688_H__
#define ICM42688_H__

#include "main.h"
#include "i2c.h"
#include "stdio.h"
#include "ICM42688_reg.h"

// I2C通信句柄定义，使用hi2c1进行通信
#define ICM_I2C_HANDLE hi2c1

// 测试模式定义
#define TEST_MODE 0
#define TEST_MODE_2 1
#define TEST_VOFA 0
#define TESR_FILTER 0

/**
 * @brief 陷波滤波器配置结构体
 * @note 用于配置陀螺仪三轴的陷波滤波器参数
 */
typedef struct notch_filter_fdesire
{
	// 陀螺仪X轴需要滤除的频段中心频率，范围[1,3]kHz
	float gyro_x_FdesireKHz;
	uint8_t gyro_x_coswz_sel;	// 无需手工配置，由算法自动计算
	
	// 陀螺仪Y轴需要滤除的频段中心频率，范围[1,3]kHz
	float gyro_y_FdesireKHz;
	uint8_t gyro_y_coswz_sel;	// 无需手工配置，由算法自动计算
	
	// 陀螺仪Z轴需要滤除的频段中心频率，范围[1,3]kHz
	float gyro_z_FdesireKHz;
	uint8_t gyro_z_coswz_sel; // 无需手工配置，由算法自动计算
	
	// 陷波滤波器带宽选择
	uint8_t GYRO_NF_BW_SEL;
}NFF_Config_st;

/**
 * @brief 抗混叠滤波器配置结构体
 * @note 用于配置加速度计和陀螺仪的抗混叠滤波器参数
 */
typedef struct {
    uint16_t bw;        // 滤波器带宽
    uint8_t delt;       // 延迟参数
    uint16_t deltsqr;   // 延迟平方参数
    uint8_t bitshift;   // 位偏移参数
} AAF_Config_st;

/**
 * @brief 可编程偏移量结构体
 * @note 用于存储传感器校准后的偏移量
 */
typedef struct {
	int16_t X_gyro_offset;  // 陀螺仪X轴偏移量
	int16_t Y_gyro_offset;  // 陀螺仪Y轴偏移量
	int16_t Z_gyro_offset;  // 陀螺仪Z轴偏移量
	int16_t X_accel_offset; // 加速度计X轴偏移量
	int16_t Y_accel_offset; // 加速度计Y轴偏移量
	int16_t Z_accel_offset; // 加速度计Z轴偏移量
} programmable_Offset_st;

/**
 * @brief 用户界面滤波器配置结构体
 * @note 用于配置陀螺仪和加速度计的用户界面滤波器参数
 */
typedef struct {
	uint8_t GYRO_UI_FILT_ORD;   // 陀螺仪UI滤波器阶数
	uint8_t GYRO_UI_FILT_BW;    // 陀螺仪UI滤波器带宽
	uint8_t ACCEL_UI_FILT_ORD;  // 加速度计UI滤波器阶数
	uint8_t ACCEL_UI_FILT_BW;   // 加速度计UI滤波器带宽
} UI_FILTER_Block_st;

/**
 * @brief 输出数据率和满量程范围配置结构体
 * @note 用于配置陀螺仪和加速度计的输出数据率和满量程范围
 */
typedef struct {
	uint8_t GYRO_ODR;   // 陀螺仪输出数据率
	uint8_t GYRO_FSR;   // 陀螺仪满量程范围
	uint8_t ACCEL_ODR;  // 加速度计输出数据率
	uint8_t ACCEL_FSR;  // 加速度计满量程范围
} ODR_SFR_st;

/**
 * @brief ICM42688原始数据结构体（包含加速度和角速度）
 * @note 用于存储从传感器读取的原始数据
 */
typedef struct icm42688_raw_data_t{
    int16_t accel_x;    // 加速度计X轴原始数据
    int16_t accel_y;    // 加速度计Y轴原始数据
    int16_t accel_z;    // 加速度计Z轴原始数据
    int16_t gyro_x;     // 陀螺仪X轴原始数据
    int16_t gyro_y;     // 陀螺仪Y轴原始数据
    int16_t gyro_z;     // 陀螺仪Z轴原始数据
} ICM42688_Raw_Data_t;

/**
 * @brief ICM42688加速度原始数据结构体
 * @note 仅包含加速度计数据
 */
typedef struct icm42688_acc_raw_data_t{
    int16_t accel_x;    // 加速度计X轴原始数据
    int16_t accel_y;    // 加速度计Y轴原始数据
    int16_t accel_z;    // 加速度计Z轴原始数据
} ICM42688_Acc_Raw_Data_t;

/**
 * @brief ICM42688陀螺仪原始数据结构体
 * @note 仅包含陀螺仪数据
 */
typedef struct icm42688_gyro_raw_data_t{
    int16_t gyro_x;     // 陀螺仪X轴原始数据
    int16_t gyro_y;     // 陀螺仪Y轴原始数据
    int16_t gyro_z;     // 陀螺仪Z轴原始数据
} ICM42688_Gyro_Raw_Data_t;

// 全局变量声明
extern float gyro_curr_resolution;     // 当前陀螺仪分辨率
extern float accel_curr_resolution;    // 当前加速度计分辨率

// 函数声明

/**
 * @brief 读取加速度计原始数据
 * @param data 指向加速度原始数据结构的指针
 * @return HAL状态类型，表示操作是否成功
 */
HAL_StatusTypeDef ICM42688_ReadAccRawData(ICM42688_Acc_Raw_Data_t *data);

/**
 * @brief 读取陀螺仪原始数据
 * @param data 指向陀螺仪原始数据结构的指针
 * @return HAL状态类型，表示操作是否成功
 */
HAL_StatusTypeDef ICM42688_ReadGyroRawData(ICM42688_Gyro_Raw_Data_t *data);

/**
 * @brief 读取传感器原始数据（加速度和陀螺仪）
 * @param data 指向原始数据结构的指针
 * @return HAL状态类型，表示操作是否成功
 */
HAL_StatusTypeDef ICM42688_ReadSensorRawData(ICM42688_Raw_Data_t *data);

/**
 * @brief 读取设备ID（WhoAmI寄存器）
 * @return 设备ID值
 */
uint8_t ICM42688_Read_WhoAmI(void);

/**
 * @brief 读取单个寄存器值
 * @param reg_address 寄存器地址
 * @return 寄存器值
 */
uint8_t ICM42688_Read_Single_Reg(uint8_t reg_address);

/**
 * @brief 配置传感器电源模式
 */
void ICM_cfg_PWR(void);

/**
 * @brief 获取平均原始数据（多次采样取平均）
 * @param data 指向原始数据结构的指针
 * @param times 采样次数
 */
void ICM_GET_Average_Raw_data(ICM42688_Raw_Data_t* data, uint16_t times);

/**
 * @brief 获取陀螺仪平均原始数据（多次采样取平均）
 * @param data 指向陀螺仪原始数据结构的指针
 * @param times 采样次数
 */
void ICM_GET_Average_Gyro_Raw_data(ICM42688_Gyro_Raw_Data_t* data, uint16_t times);

/**
 * @brief 配置抗混叠滤波器
 * @param aff_accel 指向加速度计抗混叠滤波器配置的指针
 * @param aff_gyro 指向陀螺仪抗混叠滤波器配置的指针
 */
void ICM_cfg_AFFfilter(AAF_Config_st* aff_accel, AAF_Config_st* aff_gyro);

/**
 * @brief ICM42688传感器初始化函数
 */
void ICM42688_init(void);

/**
 * @brief 校准陀螺仪并写入偏移
 * @param OSs 当前 ODR/FSR 配置
 */
void ICM42688_CalibrateGyroAndWriteOffset(ODR_SFR_st* OSs);

/**
 * @brief 读取温度数据
 */
void ICM_GET_TEMP(void);

#endif
