/* icm42688_reg.h - Complete register and bitfield definitions for TDK ICM-42688-P 
 * 该头文件定义了ICM-42688-P 6轴IMU传感器的所有寄存器地址和位域掩码
 * ICM-42688-P是一款高性能6轴运动跟踪器件，包含3轴加速度计和3轴陀螺仪
 * 寄存器访问需要通过I2C或SPI接口，并注意寄存器分Bank（0-4）管理
 * Bank 0为默认访问区域，其他Bank需通过REG_BANK_SEL寄存器切换
 */
#ifndef __ICM42688_REG_H__
#define __ICM42688_REG_H__

#include <stdint.h>

// ================= Original Definitions =================
#define ICM42688_ADDRESS 				   					  0x68
																						 
/* WHO_AM_I register */                      
#define ICM42688_REG_WHO_AM_I                 0x75
#define ICM42688_WHO_AM_I_EXPECTED            0x47  // ICM-42688-P的固定设备ID值

/* DEVICE CONFIG */                          
#define ICM42688_REG_DEVICE_CONFIG            0x11  // 设备配置寄存器，用于软复位、SPI模式设置等

/*================================= BANK 0 ===========================================
 * Bank 0: 主要包含基础配置、数据输出、中断配置等常用寄存器
 * 默认上电后处于Bank 0，无需切换即可访问
 * 包含：设备ID、电源管理、传感器配置、数据输出等关键寄存器
 */

/*================================= BANK 1 ===========================================
 * Bank 1: 主要包含陀螺仪静态配置寄存器
 * 用于配置陀螺仪的抗混叠滤波器(AAF)和陷波滤波器(NF)
 * 需要先切换到Bank 1才能访问这些寄存器
 */

/*================================= BANK 2 ===========================================
 * Bank 2: 主要包含加速度计静态配置寄存器
 * 用于配置加速度计的抗混叠滤波器(AAF)
 * 需要先切换到Bank 2才能访问这些寄存器
 */

/*================================= BANK 4 ===========================================*/
// SENSOR_CONFIG0 (0x03) 寄存器地址
#define ICM42688_REG_SENSOR_CONFIG0          0x03
//默认6轴启用
// 各轴加速度计和陀螺仪禁用位（1=禁用，0=启用）
#define ICM42688_SENSOR_CONFIG0_XA_DISABLE_Pos    0
#define ICM42688_SENSOR_CONFIG0_YA_DISABLE_Pos    1
#define ICM42688_SENSOR_CONFIG0_ZA_DISABLE_Pos    2
#define ICM42688_SENSOR_CONFIG0_XG_DISABLE_Pos    3
#define ICM42688_SENSOR_CONFIG0_YG_DISABLE_Pos    4
#define ICM42688_SENSOR_CONFIG0_ZG_DISABLE_Pos    5

#define ICM42688_SENSOR_CONFIG0_XA_DISABLE_Msk    (1U << ICM42688_SENSOR_CONFIG0_XA_DISABLE_Pos)
#define ICM42688_SENSOR_CONFIG0_YA_DISABLE_Msk    (1U << ICM42688_SENSOR_CONFIG0_YA_DISABLE_Pos)
#define ICM42688_SENSOR_CONFIG0_ZA_DISABLE_Msk    (1U << ICM42688_SENSOR_CONFIG0_ZA_DISABLE_Pos)
#define ICM42688_SENSOR_CONFIG0_XG_DISABLE_Msk    (1U << ICM42688_SENSOR_CONFIG0_XG_DISABLE_Pos)
#define ICM42688_SENSOR_CONFIG0_YG_DISABLE_Msk    (1U << ICM42688_SENSOR_CONFIG0_YG_DISABLE_Pos)
#define ICM42688_SENSOR_CONFIG0_ZG_DISABLE_Msk    (1U << ICM42688_SENSOR_CONFIG0_ZG_DISABLE_Pos)


// Register bank 1: GYRO_CONFIG_STATIC2
#define ICM42688_REG_GYRO_CONFIG_STATIC2         0x0B
#define ICM42688_BIT_GYRO_AAF_DIS                (1 << 1)    // 1: disable anti-alias filter
#define ICM42688_BIT_GYRO_NF_DIS                 (1 << 0)    // 1: disable notch filter

// Register bank 1: GYRO_CONFIG_STATIC3
#define ICM42688_REG_GYRO_CONFIG_STATIC3         0x0C
#define ICM42688_BITMASK_GYRO_AAF_DELT           0x3F        // bits 5:0
        
// Register bank 1: GYRO_CONFIG_STATIC4
#define ICM42688_REG_GYRO_CONFIG_STATIC4         0x0D
#define ICM42688_MASK_GYRO_AAF_DELTSQR_LSB       0xFF        // bits 7:0

// Register bank 1: GYRO_CONFIG_STATIC5
#define ICM42688_REG_GYRO_CONFIG_STATIC5         0x0E
#define ICM42688_MASK_GYRO_AAF_BITSHIFT          0xF0        // bits 7:4
#define ICM42688_MASK_GYRO_AAF_DELTSQR_MSB       0x0F        // bits 3:0
        
// Register bank 1: GYRO_CONFIG_STATIC6/7/8 - COSWZ LSB
#define ICM42688_REG_GYRO_X_NF_COSWZ_LSB         0x0F
#define ICM42688_REG_GYRO_Y_NF_COSWZ_LSB         0x10
#define ICM42688_REG_GYRO_Z_NF_COSWZ_LSB         0x11
        
// Register bank 1: GYRO_CONFIG_STATIC9 - COSWZ MSB & SEL bits
#define ICM42688_REG_GYRO_NF_COSWZ_MSB_SEL       0x12
#define ICM42688_BIT_GYRO_X_NF_COSWZ_SEL         (1 << 3)
#define ICM42688_BIT_GYRO_Y_NF_COSWZ_SEL         (1 << 4)
#define ICM42688_BIT_GYRO_Z_NF_COSWZ_SEL         (1 << 5)
#define ICM42688_BIT_GYRO_X_NF_COSWZ_MSB         (1 << 0)    // Use: (val >> 8) & 0x01
#define ICM42688_BIT_GYRO_Y_NF_COSWZ_MSB         (1 << 1)
#define ICM42688_BIT_GYRO_Z_NF_COSWZ_MSB         (1 << 2)
        
// Register bank 1: GYRO_CONFIG_STATIC10 - NF_BW_SEL
#define ICM42688_REG_GYRO_CONFIG_STATIC10        0x13
#define ICM42688_MASK_GYRO_NF_BW_SEL             0x70        // bits 6:4


/* ========== ACCEL_CONFIG_STATIC2 (0x03) ========== */
#define ICM42688_REG_ACCEL_CONFIG_STATIC2        0x03
        
#define ICM42688_ACCEL_AAF_DELT_POS                   1
#define ICM42688_ACCEL_AAF_DELT_MASK                  (0x3F << ACCEL_AAF_DELT_POS)  // bits 6:1

#define ICM42688_ACCEL_AAF_DIS_BIT                    0
#define ICM42688_ACCEL_AAF_DIS_MASK                   (1 << ACCEL_AAF_DIS_BIT)

/* ========== ACCEL_CONFIG_STATIC3 (0x04) ========== */
#define ICM42688_REG_ACCEL_CONFIG_STATIC3        0x04
        
#define ICM42688_ACCEL_AAF_DELTSQR_LO_POS             0
#define ICM42688_ACCEL_AAF_DELTSQR_LO_MASK            (0xFF << ACCEL_AAF_DELTSQR_LO_POS)  // bits 7:0

/* ========== ACCEL_CONFIG_STATIC4 (0x05) ========== */
#define ICM42688_REG_ACCEL_CONFIG_STATIC4        0x05
#define ICM42688_ACCEL_AAF_BITSHIFT_POS               4
#define ICM42688_ACCEL_AAF_BITSHIFT_MASK              (0x0F << ACCEL_AAF_BITSHIFT_POS)  // bits 7:4

#define ICM42688_ACCEL_AAF_DELTSQR_HI_POS             0
#define ICM42688_ACCEL_AAF_DELTSQR_HI_MASK            (0x0F << ACCEL_AAF_DELTSQR_HI_POS)  // bits 3:0


/* DRIVE CONFIG - 驱动配置寄存器 (0x13)
 * 该寄存器用于配置I2C和SPI接口的驱动强度和转换速率
 * 位[5:3]：I2C_SLEW_RATE，I2C模式下引脚14的转换速率设置
 * 位[2:0]：SPI_SLEW_RATE，SPI/I3C模式下所有输出引脚的转换速率设置
 * 较低的转换速率可减少EMI，但会限制最大通信速率
 */
#define ICM42688_REG_DRIVE_CONFIG              0x13

/* I2C_SLEW_RATE选项（仅I2C模式下引脚14有效）
 * 值越大，转换速率越快，但EMI可能增加
 */
#define ICM42688_I2C_SLEW_RATE_Pos             3
#define ICM42688_SPI_SLEW_RATE_Pos             0
#define ICM42688_DRIVE_I2C_SLEW_20_60ns        (0x0 << ICM42688_I2C_SLEW_RATE_Pos)  // 最慢
#define ICM42688_DRIVE_I2C_SLEW_12_36ns        (0x1 << ICM42688_I2C_SLEW_RATE_Pos)
#define ICM42688_DRIVE_I2C_SLEW_6_18ns         (0x2 << ICM42688_I2C_SLEW_RATE_Pos)
#define ICM42688_DRIVE_I2C_SLEW_4_12ns         (0x3 << ICM42688_I2C_SLEW_RATE_Pos)
#define ICM42688_DRIVE_I2C_SLEW_2_6ns          (0x4 << ICM42688_I2C_SLEW_RATE_Pos)
#define ICM42688_DRIVE_I2C_SLEW_LT_2ns         (0x5 << ICM42688_I2C_SLEW_RATE_Pos)  // 最快

/* SPI_SLEW_RATE选项（SPI/I3C模式下所有输出引脚）
 * 值越大，转换速率越快，但EMI可能增加
 */
#define ICM42688_DRIVE_SPI_SLEW_20_60ns        (0x0 << ICM42688_SPI_SLEW_RATE_Pos)  // 最慢
#define ICM42688_DRIVE_SPI_SLEW_12_36ns        (0x1 << ICM42688_SPI_SLEW_RATE_Pos)
#define ICM42688_DRIVE_SPI_SLEW_6_18ns         (0x2 << ICM42688_SPI_SLEW_RATE_Pos)
#define ICM42688_DRIVE_SPI_SLEW_4_12ns         (0x3 << ICM42688_SPI_SLEW_RATE_Pos)
#define ICM42688_DRIVE_SPI_SLEW_2_6ns          (0x4 << ICM42688_SPI_SLEW_RATE_Pos)
#define ICM42688_DRIVE_SPI_SLEW_LT_2ns         (0x5 << ICM42688_SPI_SLEW_RATE_Pos)  // 最快

#define ICM42688_REG_GYRO_CONFIG_STATIC2         0x0B

#define ICM42688_GYRO_AAF_DIS                    (1 << 1)  // 1: Disable anti-alias filter
#define ICM42688_GYRO_NF_DIS                     (1 << 0)  // 1: Disable notch filter


#define ICM42688_REG_GYRO_CONFIG_STATIC3         0x0C

#define ICM42688_GYRO_AAF_DELT_MASK              0x3F    // [5:0]
#define ICM42688_GYRO_AAF_DELT_POS               0

#define ICM42688_REG_GYRO_CONFIG_STATIC4         0x0D

#define ICM42688_GYRO_AAF_DELTSQR_LSB_MASK       0xFF    // [7:0]
#define ICM42688_GYRO_AAF_DELTSQR_LSB_POS        0


#define ICM42688_REG_GYRO_CONFIG_STATIC5         0x0E

#define ICM42688_GYRO_AAF_BITSHIFT_MASK          0xF0    // [7:4]
#define ICM42688_GYRO_AAF_BITSHIFT_POS           4

#define ICM42688_GYRO_AAF_DELTSQR_MSB_MASK       0x0F    // [3:0]
#define ICM42688_GYRO_AAF_DELTSQR_MSB_POS        0


#define ICM42688_REG_GYRO_CONFIG_STATIC6         0x0F    // GYRO_X_NF_COSWZ[7:0]
#define ICM42688_REG_GYRO_CONFIG_STATIC7         0x10    // GYRO_Y_NF_COSWZ[7:0]
#define ICM42688_REG_GYRO_CONFIG_STATIC8         0x11    // GYRO_Z_NF_COSWZ[7:0]



#define ICM42688_REG_GYRO_CONFIG_STATIC9         0x12
#define ICM42688_GYRO_X_NF_COSWZ_SEL             (1 << 3)
#define ICM42688_GYRO_Y_NF_COSWZ_SEL             (1 << 4)
#define ICM42688_GYRO_Z_NF_COSWZ_SEL             (1 << 5)


#define ICM42688_REG_GYRO_CONFIG_STATIC10        0x13
#define ICM42688_GYRO_NF_BW_SEL_MASK             0x70  // [6:4]
#define ICM42688_GYRO_NF_BW_SEL_POS              4

// 可选值（单位Hz）：
#define ICM42688_GYRO_NF_BW_1449HZ               (0 << ICM42688_GYRO_NF_BW_SEL_POS)
#define ICM42688_GYRO_NF_BW_680HZ                (1 << ICM42688_GYRO_NF_BW_SEL_POS)
#define ICM42688_GYRO_NF_BW_329HZ                (2 << ICM42688_GYRO_NF_BW_SEL_POS)
#define ICM42688_GYRO_NF_BW_162HZ                (3 << ICM42688_GYRO_NF_BW_SEL_POS)
#define ICM42688_GYRO_NF_BW_80HZ                 (4 << ICM42688_GYRO_NF_BW_SEL_POS)
#define ICM42688_GYRO_NF_BW_40HZ                 (5 << ICM42688_GYRO_NF_BW_SEL_POS)
#define ICM42688_GYRO_NF_BW_20HZ                 (6 << ICM42688_GYRO_NF_BW_SEL_POS)
#define ICM42688_GYRO_NF_BW_10HZ                 (7 << ICM42688_GYRO_NF_BW_SEL_POS)

/* POWER MANAGEMENT 0 */                     
#define ICM42688_PWR_REG_PWR_MGMT0     				0x4E
#define ICM42688_PWR_ACCEL_MODE_Pos						0
#define ICM42688_PWR_GYRO_MODE_Pos						2
#define ICM42688_PWR_IDLE_Pos 								4
#define ICM42688_PWR_TEMP_DIS_Pos							5

#define ICM42688_PWR_TEMP_ON									(0x0 << ICM42688_PWR_TEMP_DIS_Pos)
#define ICM42688_PWR_TEMP_OFF									(0x1 << ICM42688_PWR_TEMP_DIS_Pos)
#define ICM42688_PWR_IDLE_ON									(0x1 << ICM42688_PWR_IDLE_Pos)
#define ICM42688_PWR_IDLE_OFF									(0x0 << ICM42688_PWR_IDLE_Pos)
#define ICM42688_PWR_GYRO_OFF									(0x0 << ICM42688_PWR_GYRO_MODE_Pos)
#define ICM42688_PWR_GYRO_LN									(0x3 << ICM42688_PWR_GYRO_MODE_Pos)
#define ICM42688_PWR_ACCEL_OFF								(0x0 << ICM42688_PWR_ACCEL_MODE_Pos)
#define ICM42688_PWR_ACCEL_LP									(0x2 << ICM42688_PWR_ACCEL_MODE_Pos)
#define ICM42688_PWR_ACCEL_LN									(0x3 << ICM42688_PWR_ACCEL_MODE_Pos)

/* ACCEL CONFIG 0 - 加速度计配置寄存器0 (0x50)
 * 该寄存器用于配置加速度计的量程和输出数据率(ODR)
 * 位[7:6]：保留，必须写0
 * 位[5:3]：ACCEL_FS_SEL，加速度计量程选择
 * 位[2:0]：ACCEL_ODR，加速度计输出数据率
 */
#define ICM42688_REG_ACCEL_CONFIG0             0x50

/* 加速度计量程选择（FS = Full Scale）
 * 量程越大，测量范围越大，但分辨率越低
 */
#define ICM42688_ACCEL_FS_SEL_Pos              5
#define ICM42688_ACCEL_FS_16G                  0x0  // ±16g
#define ICM42688_ACCEL_FS_8G                   0x1  // ±8g
#define ICM42688_ACCEL_FS_4G                   0x2  // ±4g
#define ICM42688_ACCEL_FS_2G                   0x3  // ±2g

/* 加速度计输出数据率(ODR)
 * 数据率越高，延迟越低，但功耗越高
 */
#define ICM42688_ACCEL_ODR_Pos                 0
#define ICM42688_ACCEL_ODR_32KHZ               0x1  // 32kHz
#define ICM42688_ACCEL_ODR_16KHZ               0x2  // 16kHz
#define ICM42688_ACCEL_ODR_8KHZ                0x3  // 8kHz
#define ICM42688_ACCEL_ODR_4KHZ                0x4  // 4kHz
#define ICM42688_ACCEL_ODR_2KHZ                0x5  // 2kHz
#define ICM42688_ACCEL_ODR_1KHZ                0x6  // 1kHz
#define ICM42688_ACCEL_ODR_200HZ               0x7  // 200Hz
#define ICM42688_ACCEL_ODR_100HZ               0x8  // 100Hz
#define ICM42688_ACCEL_ODR_50HZ                0x9  // 50Hz
#define ICM42688_ACCEL_ODR_25HZ                0xA  // 25Hz
#define ICM42688_ACCEL_ODR_12_5HZ              0xB  // 12.5Hz
#define ICM42688_ACCEL_ODR_6_25HZ              0xC  // 6.25Hz
#define ICM42688_ACCEL_ODR_3_125HZ             0xD  // 3.125Hz
#define ICM42688_ACCEL_ODR_1_5625HZ            0xE  // 1.5625Hz
#define ICM42688_ACCEL_ODR_500HZ               0xF  // 500Hz

#define ICM42688_REG_GYRO_CONFIG1             0x51

#define ICM42688_TEMP_FILT_BW_Pos             5
#define ICM42688_TEMP_FILT_BW_Msk             (0x07 << ICM42688_TEMP_FILT_BW_Pos)
// TEMP_FILT_BW options:
#define ICM42688_TEMP_FILT_BW_4000HZ          (0x00 << ICM42688_TEMP_FILT_BW_Pos)  // default
#define ICM42688_TEMP_FILT_BW_170HZ           (0x01 << ICM42688_TEMP_FILT_BW_Pos)
#define ICM42688_TEMP_FILT_BW_82HZ            (0x02 << ICM42688_TEMP_FILT_BW_Pos)
#define ICM42688_TEMP_FILT_BW_40HZ            (0x03 << ICM42688_TEMP_FILT_BW_Pos)
#define ICM42688_TEMP_FILT_BW_20HZ            (0x04 << ICM42688_TEMP_FILT_BW_Pos)
#define ICM42688_TEMP_FILT_BW_10HZ            (0x05 << ICM42688_TEMP_FILT_BW_Pos)
#define ICM42688_TEMP_FILT_BW_5HZ             (0x06 << ICM42688_TEMP_FILT_BW_Pos)

#define ICM42688_GYRO_UI_FILT_ORD_Pos         2
#define ICM42688_GYRO_UI_FILT_ORD_Msk         (0x03 << ICM42688_GYRO_UI_FILT_ORD_Pos)

#define ICM42688_GYRO_DEC2_M2_ORD_Pos         0
#define ICM42688_GYRO_DEC2_M2_ORD_Msk         (0x03 << ICM42688_GYRO_DEC2_M2_ORD_Pos)


#define ICM42688_REG_GYRO_ACCEL_CONFIG0       0x52

#define ICM42688_ACCEL_UI_FILT_BW_Pos         4
#define ICM42688_ACCEL_UI_FILT_BW_Msk         (0x0F << ICM42688_ACCEL_UI_FILT_BW_Pos)
// Mode-specific meaning: see datasheet (LN/LP filter)

#define ICM42688_GYRO_UI_FILT_BW_Pos          0
#define ICM42688_GYRO_UI_FILT_BW_Msk          (0x0F << ICM42688_GYRO_UI_FILT_BW_Pos)


#define ICM42688_REG_ACCEL_CONFIG1            0x53

#define ICM42688_ACCEL_UI_FILT_ORD_Pos        3
#define ICM42688_ACCEL_UI_FILT_ORD_Msk        (0x03 << ICM42688_ACCEL_UI_FILT_ORD_Pos)

#define ICM42688_ACCEL_DEC2_M2_ORD_Pos        1
#define ICM42688_ACCEL_DEC2_M2_ORD_Msk        (0x03 << ICM42688_ACCEL_DEC2_M2_ORD_Pos)



/* GYRO CONFIG 0 - 陀螺仪配置寄存器0 (0x4F)
 * 该寄存器用于配置陀螺仪的量程和输出数据率(ODR)
 * 位[7:6]：保留，必须写0
 * 位[5:3]：GYRO_FS_SEL，陀螺仪量程选择
 * 位[2:0]：GYRO_ODR，陀螺仪输出数据率
 */
#define ICM42688_REG_GYRO_CONFIG0             0x4F

/* 陀螺仪量程选择（FS = Full Scale）
 * 量程越大，可测量的角速度范围越大，但分辨率越低
 */
#define ICM42688_GYRO_FS_SEL_Pos              5
#define ICM42688_GYRO_FS_2000DPS              0x0  // ±2000°/s
#define ICM42688_GYRO_FS_1000DPS              0x1  // ±1000°/s
#define ICM42688_GYRO_FS_500DPS               0x2  // ±500°/s
#define ICM42688_GYRO_FS_250DPS               0x3  // ±250°/s
#define ICM42688_GYRO_FS_125DPS               0x4  // ±125°/s
#define ICM42688_GYRO_FS_62_5DPS              0x5  // ±62.5°/s
#define ICM42688_GYRO_FS_31_25DPS             0x6  // ±31.25°/s
#define ICM42688_GYRO_FS_15_625DPS            0x7  // ±15.625°/s

/* 陀螺仪输出数据率(ODR)
 * 数据率越高，延迟越低，但功耗越高
 */
#define ICM42688_GYRO_ODR_Pos                 0
#define ICM42688_GYRO_ODR_32KHZ               0x1  // 32kHz
#define ICM42688_GYRO_ODR_16KHZ               0x2  // 16kHz
#define ICM42688_GYRO_ODR_8KHZ                0x3  // 8kHz
#define ICM42688_GYRO_ODR_4KHZ                0x4  // 4kHz
#define ICM42688_GYRO_ODR_2KHZ                0x5  // 2kHz
#define ICM42688_GYRO_ODR_1KHZ                0x6  // 1kHz
#define ICM42688_GYRO_ODR_200HZ               0x7  // 200Hz
#define ICM42688_GYRO_ODR_100HZ               0x8  // 100Hz
#define ICM42688_GYRO_ODR_50HZ                0x9  // 50Hz
#define ICM42688_GYRO_ODR_25HZ                0xA  // 25Hz
#define ICM42688_GYRO_ODR_12_5HZ              0xB  // 12.5Hz
#define ICM42688_GYRO_ODR_RESERVED_C          0xC 
#define ICM42688_GYRO_ODR_RESERVED_D          0xD 
#define ICM42688_GYRO_ODR_RESERVED_E          0xE 
#define ICM42688_GYRO_ODR_500HZ               0xF 

																						 


/* Anti-Aliasing Filter (AAF) 配置表
 * 这些结构体定义了不同采样率下的AAF滤波器参数
 * 每个条目包含：
 *   - 采样率(Hz)
 *   - DELT值（用于AAF计算）
 *   - DELTSQR值（用于AAF计算）
 *   - BITSHIFT值（用于AAF计算）
 * 使用时根据所需的ODR选择合适的配置
 * 注意：这些参数直接影响传感器的带宽和噪声性能
 */
#define ICM42688_AAF_ENTRY_42     { 42,  1,   1,  15 }
#define ICM42688_AAF_ENTRY_84     { 84,  2,   4,  13 }
#define ICM42688_AAF_ENTRY_126    { 126, 3,   9,  12 }
#define ICM42688_AAF_ENTRY_170    { 170, 4,  16,  11 }
#define ICM42688_AAF_ENTRY_213    { 213, 5,  25,  10 }
#define ICM42688_AAF_ENTRY_258    { 258, 6,  36,  10 }
#define ICM42688_AAF_ENTRY_303    { 303, 7,  49,  9  }
#define ICM42688_AAF_ENTRY_348    { 348, 8,  64,  9  }
#define ICM42688_AAF_ENTRY_394    { 394, 9,  81,  9  }
#define ICM42688_AAF_ENTRY_441    { 441,10, 100,  8  }
#define ICM42688_AAF_ENTRY_488    { 488,11, 122,  8  }
#define ICM42688_AAF_ENTRY_536    { 536,12, 144,  8  }
#define ICM42688_AAF_ENTRY_585    { 585,13, 170,  8  }
#define ICM42688_AAF_ENTRY_634    { 634,14, 196,  7  }
#define ICM42688_AAF_ENTRY_684    { 684,15, 224,  7  }
#define ICM42688_AAF_ENTRY_734    { 734,16, 256,  7  }
#define ICM42688_AAF_ENTRY_785    { 785,17, 288,  7  }
#define ICM42688_AAF_ENTRY_837    { 837,18, 324,  7  }
#define ICM42688_AAF_ENTRY_890    { 890,19, 360,  6  }
#define ICM42688_AAF_ENTRY_943    { 943,20, 400,  6  }
#define ICM42688_AAF_ENTRY_997    { 997,21, 440,  6  }
#define ICM42688_AAF_ENTRY_1051   {1051,22, 488,  6  }
#define ICM42688_AAF_ENTRY_1107   {1107,23, 528,  6  }
#define ICM42688_AAF_ENTRY_1163   {1163,24, 576,  6  }
#define ICM42688_AAF_ENTRY_1220   {1220,25, 624,  6  }
#define ICM42688_AAF_ENTRY_1277   {1277,26, 680,  6  }
#define ICM42688_AAF_ENTRY_1336   {1336,27, 736,  5  }
#define ICM42688_AAF_ENTRY_1395   {1395,28, 784,  5  }
#define ICM42688_AAF_ENTRY_1454   {1454,29, 848,  5  }
#define ICM42688_AAF_ENTRY_1515   {1515,30, 896,  5  }
#define ICM42688_AAF_ENTRY_1577   {1577,31, 960,  5  }
#define ICM42688_AAF_ENTRY_1639   {1639,32,1024,  5  }
#define ICM42688_AAF_ENTRY_1702   {1702,33,1088,  5  }
#define ICM42688_AAF_ENTRY_1766   {1766,34,1152,  5  }
#define ICM42688_AAF_ENTRY_1830   {1830,35,1232,  5  }
#define ICM42688_AAF_ENTRY_1896   {1896,36,1296,  5  }
#define ICM42688_AAF_ENTRY_1962   {1962,37,1376,  4  }
#define ICM42688_AAF_ENTRY_2029   {2029,38,1440,  4  }
#define ICM42688_AAF_ENTRY_2097   {2097,39,1536,  4  }
#define ICM42688_AAF_ENTRY_2166   {2166,40,1600,  4  }
#define ICM42688_AAF_ENTRY_2235   {2235,41,1696,  4  }
#define ICM42688_AAF_ENTRY_2306   {2306,42,1760,  4  }
#define ICM42688_AAF_ENTRY_2377   {2377,43,1856,  4  }
#define ICM42688_AAF_ENTRY_2449   {2449,44,1952,  4  }
#define ICM42688_AAF_ENTRY_2522   {2522,45,2016,  4  }
#define ICM42688_AAF_ENTRY_2596   {2596,46,2112,  4  }
#define ICM42688_AAF_ENTRY_2671   {2671,47,2208,  4  }
#define ICM42688_AAF_ENTRY_2746   {2746,48,2304,  4  }
#define ICM42688_AAF_ENTRY_2823   {2823,49,2400,  4  }
#define ICM42688_AAF_ENTRY_2900   {2900,50,2496,  4  }
#define ICM42688_AAF_ENTRY_2978   {2978,51,2592,  4  }
#define ICM42688_AAF_ENTRY_3057   {3057,52,2720,  4  }
#define ICM42688_AAF_ENTRY_3137   {3137,53,2816,  3  }
#define ICM42688_AAF_ENTRY_3217   {3217,54,2944,  3  }
#define ICM42688_AAF_ENTRY_3299   {3299,55,3008,  3  }
#define ICM42688_AAF_ENTRY_3381   {3381,56,3136,  3  }
#define ICM42688_AAF_ENTRY_3464   {3464,57,3264,  3  }
#define ICM42688_AAF_ENTRY_3548   {3548,58,3392,  3  }
#define ICM42688_AAF_ENTRY_3633   {3633,59,3456,  3  }
#define ICM42688_AAF_ENTRY_3718   {3718,60,3584,  3  }
#define ICM42688_AAF_ENTRY_3805   {3805,61,3712,  3  }
#define ICM42688_AAF_ENTRY_3892   {3892,62,3840,  3  }
#define ICM42688_AAF_ENTRY_3979   {3979,63,3968,  3  }


/* ACCEL DATA OUT - 加速度计数据输出寄存器
 * 16位有符号整数，MSB在前，LSB在后
 * 数据格式：X轴高8位 -> X轴低8位 -> Y轴高8位 -> Y轴低8位 -> Z轴高8位 -> Z轴低8位
 */
#define ICM42688_REG_ACCEL_DATA_X1            0x1F  // X轴加速度高8位
#define ICM42688_REG_ACCEL_DATA_X0            0x20  // X轴加速度低8位
#define ICM42688_REG_ACCEL_DATA_Y1            0x21  // Y轴加速度高8位
#define ICM42688_REG_ACCEL_DATA_Y0            0x22  // Y轴加速度低8位
#define ICM42688_REG_ACCEL_DATA_Z1            0x23  // Z轴加速度高8位
#define ICM42688_REG_ACCEL_DATA_Z0            0x24  // Z轴加速度低8位

/* GYRO DATA OUT - 陀螺仪数据输出寄存器
 * 16位有符号整数，MSB在前，LSB在后
 * 数据格式同加速度计
 */
#define ICM42688_REG_GYRO_DATA_X1            	0x25  // X轴角速度高8位
#define ICM42688_REG_GYRO_DATA_X0             0x26  // X轴角速度低8位
#define ICM42688_REG_GYRO_DATA_Y1             0x27  // Y轴角速度高8位
#define ICM42688_REG_GYRO_DATA_Y0             0x28  // Y轴角速度低8位
#define ICM42688_REG_GYRO_DATA_Z1             0x29  // Z轴角速度高8位
#define ICM42688_REG_GYRO_DATA_Z0             0x2A  // Z轴角速度低8位
																						  
/* TEMP OUT - 温度传感器数据输出寄存器
 * 16位有符号整数，表示传感器内部温度
 * 转换公式：温度(°C) = 25 + (读数 - 25°C时的偏移)/温度灵敏度
 */
#define ICM42688_REG_TEMP_DATA1               0x1D  // 温度高8位
#define ICM42688_REG_TEMP_DATA0               0x1E  // 温度低8位

																						  
/* FIFO config and data */                    
#define ICM42688_REG_FIFO_COUNT               0x70
#define ICM42688_REG_FIFO_DATA                0x72
																						  
/* INT config */                              
#define ICM42688_REG_INT_CONFIG               0x14
#define ICM42688_REG_INT_SOURCE0              0x3F
#define ICM42688_REG_INT_SOURCE1              0x40
#define ICM42688_REG_INT_STATUS_DRDY          0x3A

// ================= OCR-Enhanced Additions ================
// 以下内容基于手册截图自动补全，请再次核验

/* DEVICE_CONFIG */
#define ICM42688_REG_DEVICE_CONFIG             0x11
#define ICM42688_DEVICE_CONFIG_SOFT_RESET_Pos  0
#define ICM42688_DEVICE_CONFIG_SPI_MODE_Pos    4
#define ICM42688_DEVICE_CONFIG_SOFT_RESET_Msk  (1 << ICM42688_DEVICE_CONFIG_SOFT_RESET_Pos)
#define ICM42688_DEVICE_CONFIG_SPI_MODE_Msk    (1 << ICM42688_DEVICE_CONFIG_SPI_MODE_Pos)

/* WHO_AM_I */
#define ICM42688_REG_WHO_AM_I                  0x75
#define ICM42688_WHO_AM_I_EXPECTED             0x47

/* REG_BANK_SEL */
#define ICM42688_REG_BANK_SEL                  0x76
#define ICM42688_BANK_SEL_Msk                  0x07
#define ICM42688_BANK_SEL_0                    0x00
#define ICM42688_BANK_SEL_1                    0x01
#define ICM42688_BANK_SEL_2                    0x02
#define ICM42688_BANK_SEL_3                    0x03
#define ICM42688_BANK_SEL_4                    0x04

/* SELF_TEST_CONFIG */
#define ICM42688_REG_SELF_TEST_CONFIG          0x70
#define ICM42688_SELFTEST_ACCEL_POWER_Pos      6
#define ICM42688_SELFTEST_EN_AZ_Pos            5
#define ICM42688_SELFTEST_EN_AY_Pos            4
#define ICM42688_SELFTEST_EN_AX_Pos            3
#define ICM42688_SELFTEST_EN_GZ_Pos            2
#define ICM42688_SELFTEST_EN_GY_Pos            1
#define ICM42688_SELFTEST_EN_GX_Pos            0

/* OFFSET_USER0-8 */
#define ICM42688_REG_OFFSET_USER0              0x77
#define ICM42688_REG_OFFSET_USER1              0x78
#define ICM42688_REG_OFFSET_USER2              0x79
#define ICM42688_REG_OFFSET_USER3              0x7A
#define ICM42688_REG_OFFSET_USER4              0x7B
#define ICM42688_REG_OFFSET_USER5              0x7C
#define ICM42688_REG_OFFSET_USER6              0x7D
#define ICM42688_REG_OFFSET_USER7              0x7E
#define ICM42688_REG_OFFSET_USER8              0x7F

/* FIFO_LOST_PKT */
#define ICM42688_REG_FIFO_LOST_PKT0            0x6C
#define ICM42688_REG_FIFO_LOST_PKT1            0x6D


#endif /* __ICM42688_REG_H__ */
