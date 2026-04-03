#ifndef LPS22HBTR_H__
#define LPS22HBTR_H__

#include "stm32f4xx_hal.h"
#include "stdio.h"
#include <stdbool.h>
#include <stdint.h>

#define hide 1	//是否隐藏内部函数

/* ------------ 寄存器地址（节选，SPI/I2C 公用） ------------ */
#define LPS22HB_REG_INTERRUPT_CFG   0x0B
#define LPS22HB_REG_WHO_AM_I        0x0F
#define LPS22HB_REG_CTRL_REG1       0x10
#define LPS22HB_REG_CTRL_REG2       0x11
#define LPS22HB_REG_CTRL_REG3       0x12
#define LPS22HB_REG_FIFO_CTRL       0x14
#define LPS22HB_REG_STATUS          0x27
#define LPS22HB_REG_PRESS_OUT_XL    0x28
#define LPS22HB_REG_PRESS_OUT_L     0x29
#define LPS22HB_REG_PRESS_OUT_H     0x2A
#define LPS22HB_REG_TEMP_OUT_L      0x2B
#define LPS22HB_REG_TEMP_OUT_H      0x2C
#define LPS22HB_REG_FIFO_STATUS     0x26

/* ------------ 识别码 ------------ */
#define LPS22HB_WHO_AM_I_VAL        0xB1  /* datasheet */

/* ------------ SPI 指令位（LPS22HB：bit7=1 表示读） ------------ */
#define LPS22HB_SPI_READ            0x80
#define LPS22HB_SPI_WRITE           0x00

/* ------------ CTRL_REG1 位域 ------------ */
/* ODR[2:0] = {0:PD/oneshot,1:1Hz,2:10Hz,3:25Hz,4:50Hz,5:75Hz} */
typedef enum {
    LPS22HB_ODR_POWER_DOWN = 0,
    LPS22HB_ODR_1HZ        = 1,
    LPS22HB_ODR_10HZ       = 2,
    LPS22HB_ODR_25HZ       = 3,
    LPS22HB_ODR_50HZ       = 4,
    LPS22HB_ODR_75HZ       = 5,
} lps22hb_odr_t;

typedef enum {
    LPS22HB_LPFP_OFF = 0,      /* 关闭附加低通，带宽=ODR/2 */
    LPS22HB_LPFP_ODR_9,        /* EN_LPFP=1, LPFP_CFG=0 -> 带宽≈ODR/9 */
    LPS22HB_LPFP_ODR_20        /* EN_LPFP=1, LPFP_CFG=1 -> 带宽≈ODR/20 */
} lps22hb_lpf_t;

/* ------------ FIFO 模式（FMODE[2:0] in FIFO_CTRL） ------------ */
typedef enum {
    LPS22HB_FIFO_BYPASS   = 0,
    LPS22HB_FIFO_FIFO     = 1,
    LPS22HB_FIFO_STREAM   = 2,
    LPS22HB_FIFO_STREAM2FIFO = 3,
    LPS22HB_FIFO_BYPASS2STREAM = 4,
    LPS22HB_FIFO_DYNAMIC_STREAM = 6,
    LPS22HB_FIFO_BYPASS2FIFO = 7,
} lps22hb_fifo_mode_t;

/* ------------ 句柄 ------------ */
typedef struct {
    SPI_HandleTypeDef *hspi;              /* 你的 SPI 句柄（例：&hspi3） */
    GPIO_TypeDef      *cs_gpio_port;      /* 软件片选 GPIO 端口 */
    uint16_t           cs_gpio_pin;       /* 软件片选 GPIO 引脚 */
    GPIO_TypeDef      *drdy_gpio_port;    /* 可选：DRDY 引脚端口（中断用） */
    uint16_t           drdy_gpio_pin;     /* 可选：DRDY 引脚 */
    uint32_t           spi_timeout;       /* HAL SPI 超时（ms） */
} lps22hb_t;

#if hide // 隐藏内部函数

/* ------------ 公开 API ------------ */
/* 基础 SPI 片选封装（如你已有宏，可改为宏） */
void     LPS22HB_CS_Select(lps22hb_t *dev);
void     LPS22HB_CS_Deselect(lps22hb_t *dev);

/* 初始化：复位 -> 配置 ODR/BDU/LPF -> 使能地址自增（IF_ADD_INC） */
HAL_StatusTypeDef LPS22HB_Init(lps22hb_t *dev,
                               lps22hb_odr_t odr,
                               bool bdu_lock,
                               lps22hb_lpf_t lpf_cfg);

/* 软复位 / Reboot */
HAL_StatusTypeDef LPS22HB_SoftwareReset(lps22hb_t *dev);
HAL_StatusTypeDef LPS22HB_RebootMemory(lps22hb_t *dev);

/* 读芯片 ID */
HAL_StatusTypeDef LPS22HB_ReadWhoAmI(lps22hb_t *dev, uint8_t *id);

/* ODR / BDU / LPF 独立设置 */
HAL_StatusTypeDef LPS22HB_SetODR(lps22hb_t *dev, lps22hb_odr_t odr);
HAL_StatusTypeDef LPS22HB_SetBDU(lps22hb_t *dev, bool enable);
HAL_StatusTypeDef LPS22HB_SetLPF(lps22hb_t *dev, lps22hb_lpf_t lpf_cfg);

/* One-shot 单次转换（仅在 PD 下有效）；返回数据就绪 */
HAL_StatusTypeDef LPS22HB_OneShot(lps22hb_t *dev);

/* 原始数据读取 */
HAL_StatusTypeDef LPS22HB_ReadRawPressure(lps22hb_t *dev, int32_t *raw); /* 24-bit 2’s comp */
HAL_StatusTypeDef LPS22HB_ReadRawTemperature(lps22hb_t *dev, int16_t *raw); /* 16-bit 2’s comp */

/* 物理量换算：压力 hPa，温度 °C */
HAL_StatusTypeDef LPS22HB_ReadPressure_hPa(lps22hb_t *dev, float *p_hPa);
HAL_StatusTypeDef LPS22HB_ReadTemperature_C(lps22hb_t *dev, float *t_C);

/* FIFO（可选） */
HAL_StatusTypeDef LPS22HB_SetFIFO(lps22hb_t *dev, bool enable, lps22hb_fifo_mode_t mode, uint8_t watermark);
HAL_StatusTypeDef LPS22HB_ReadFIFO_Level(lps22hb_t *dev, uint8_t *level);

/* 低级寄存器读写（SPI 4-wire） */
HAL_StatusTypeDef LPS22HB_ReadReg(lps22hb_t *dev, uint8_t reg, uint8_t *data, uint16_t len);
HAL_StatusTypeDef LPS22HB_WriteReg(lps22hb_t *dev, uint8_t reg, const uint8_t *data, uint16_t len);

/* ---------- RPDS: Pressure offset (OPC) ---------- */
/* 写：以 “hPa” 为单位写入 RPDS（内部按 4096 LSB/hPa 转换并限幅） */
HAL_StatusTypeDef LPS22HB_WriteRPDS_hPa(lps22hb_t *dev, float offset_hPa);

/* 读：以 “hPa” 为单位读出 RPDS */
HAL_StatusTypeDef LPS22HB_ReadRPDS_hPa(lps22hb_t *dev, float *offset_hPa);

/* 清零：将 RPDS 置 0（恢复“无偏移”） */
HAL_StatusTypeDef LPS22HB_ClearRPDS(lps22hb_t *dev);

/* 标定：以参考气压（hPa）为准，测 N 次求均值，写入使输出贴合参考 */
HAL_StatusTypeDef LPS22HB_CalibrateRPDS_ToReference(lps22hb_t *dev,
                                                    float reference_hPa,
                                                    uint8_t average_N);

typedef struct {
    float pressure_hPa;
    float temp_C;
    uint32_t tick;
} lps22hb_sample_t;
#endif 

HAL_StatusTypeDef Baro_Init(void);// 初始化气压计
HAL_StatusTypeDef Baro_RelativePressure_hPa(float* r_hPa);// 读取气压计相对大气压: 以初始化时g_P0_hpa的气压为基准
HAL_StatusTypeDef Baro_ReadTemperature_C(float* t_C);// 读取气压计温度

#endif /* __LPS22HB_H__ */
