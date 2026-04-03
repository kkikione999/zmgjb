#include "LPS22HBTR.h"
#include "cmsis_os.h"   /* 若未用 FreeRTOS，可删此行 */
#include "main.h"
#include <math.h>
extern SPI_HandleTypeDef hspi3;

lps22hb_t g_lps = {
    .hspi = &hspi3,
    .cs_gpio_port = Barometer_SPI_Software_NSS_GPIO_Port,
    .cs_gpio_pin  = Barometer_SPI_Software_NSS_Pin,
    .spi_timeout  = 50,
};

#define average_N 50
float g_P0_hpa;

/* 便捷宏：幂指数 */
#ifndef POWF1903
#define POWF1903(x) powf((x), 0.1903f)
#endif

#define BIT(n)                 (1u<<(n))

/* CTRL_REG1 位 */
#define CTRL1_ODR_POS          4  /* ODR2:ODR0 位于 [6:4]，便于统一写入 */
#define CTRL1_ODR_MASK         (BIT(6)|BIT(5)|BIT(4))
#define CTRL1_EN_LPFP          BIT(3)
#define CTRL1_LPFP_CFG         BIT(2)
#define CTRL1_BDU              BIT(1)
#define CTRL1_SIM              BIT(0)

/* CTRL_REG2 位 */
#define CTRL2_BOOT             BIT(7)
#define CTRL2_FIFO_EN          BIT(6)
#define CTRL2_STOP_ON_FTH      BIT(5)
#define CTRL2_IF_ADD_INC       BIT(4)
#define CTRL2_I2C_DIS          BIT(3)
#define CTRL2_SWRESET          BIT(2)
#define CTRL2_ONE_SHOT         BIT(0)

/* STATUS 位（datasheet 标准语义：P_DA/T_DA 新数据就绪） */
#define STATUS_P_DA            BIT(1)
#define STATUS_T_DA            BIT(0)

/* FIFO_CTRL 位 */
#define FIFO_WTM_MASK          0x1F
#define FIFO_MODE_MASK         (BIT(7)|BIT(6)|BIT(5))

/* 超时辅助：等待某位自清或置位 */
static HAL_StatusTypeDef wait_flag(lps22hb_t *dev, uint8_t reg, uint8_t mask, uint8_t expect_set, uint32_t timeout_ms);

/* 片选控制 */
void LPS22HB_CS_Select(lps22hb_t *dev)   { HAL_GPIO_WritePin(dev->cs_gpio_port, dev->cs_gpio_pin, GPIO_PIN_RESET); }
void LPS22HB_CS_Deselect(lps22hb_t *dev) { HAL_GPIO_WritePin(dev->cs_gpio_port, dev->cs_gpio_pin, GPIO_PIN_SET);   }

/* 低级寄存器读写 */
HAL_StatusTypeDef LPS22HB_ReadReg(lps22hb_t *dev, uint8_t reg, uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef st;
    uint8_t addr = reg | LPS22HB_SPI_READ; /* bit7=1 -> read */
    LPS22HB_CS_Select(dev);
    st = HAL_SPI_Transmit(dev->hspi, &addr, 1, dev->spi_timeout);
    if (st == HAL_OK) {
        st = HAL_SPI_Receive(dev->hspi, data, len, dev->spi_timeout);
    }
    LPS22HB_CS_Deselect(dev);
    return st;
}

HAL_StatusTypeDef LPS22HB_WriteReg(lps22hb_t *dev, uint8_t reg, const uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef st;
    uint8_t addr = reg | LPS22HB_SPI_WRITE; /* bit7=0 -> write */
    LPS22HB_CS_Select(dev);
    st = HAL_SPI_Transmit(dev->hspi, &addr, 1, dev->spi_timeout);
    if (st == HAL_OK && len>0) {
        st = HAL_SPI_Transmit(dev->hspi, (uint8_t*)data, len, dev->spi_timeout);
    }
    LPS22HB_CS_Deselect(dev);
    return st;
}

/* 读芯片 ID */
HAL_StatusTypeDef LPS22HB_ReadWhoAmI(lps22hb_t *dev, uint8_t *id)
{
    return LPS22HB_ReadReg(dev, LPS22HB_REG_WHO_AM_I, id, 1);
}

/* 软件复位：SWRESET=1 自清 */
HAL_StatusTypeDef LPS22HB_SoftwareReset(lps22hb_t *dev)
{
    uint8_t v = 0;
    if (LPS22HB_ReadReg(dev, LPS22HB_REG_CTRL_REG2, &v, 1) != HAL_OK) return HAL_ERROR;
    v |= CTRL2_SWRESET;
    if (LPS22HB_WriteReg(dev, LPS22HB_REG_CTRL_REG2, &v, 1) != HAL_OK) return HAL_ERROR;
    /* 等待 SWRESET 清零 */
    return wait_flag(dev, LPS22HB_REG_CTRL_REG2, CTRL2_SWRESET, 0, 20);
}

/* Reboot memory：BOOT=1 自清 */
HAL_StatusTypeDef LPS22HB_RebootMemory(lps22hb_t *dev)
{
    uint8_t v = 0;
    if (LPS22HB_ReadReg(dev, LPS22HB_REG_CTRL_REG2, &v, 1) != HAL_OK) return HAL_ERROR;
    v |= CTRL2_BOOT;
    if (LPS22HB_WriteReg(dev, LPS22HB_REG_CTRL_REG2, &v, 1) != HAL_OK) return HAL_ERROR;
    return wait_flag(dev, LPS22HB_REG_CTRL_REG2, CTRL2_BOOT, 0, 20);
}

/* 设置 ODR（若设为 PD，可配合 OneShot） */
HAL_StatusTypeDef LPS22HB_SetODR(lps22hb_t *dev, lps22hb_odr_t odr)
{
    uint8_t v = 0;
    if (LPS22HB_ReadReg(dev, LPS22HB_REG_CTRL_REG1, &v, 1) != HAL_OK) return HAL_ERROR;
    v &= ~CTRL1_ODR_MASK;
    v |= ((uint8_t)odr << CTRL1_ODR_POS) & CTRL1_ODR_MASK;
    return LPS22HB_WriteReg(dev, LPS22HB_REG_CTRL_REG1, &v, 1);
}

/* BDU：整组锁存，避免跨样本读 */
HAL_StatusTypeDef LPS22HB_SetBDU(lps22hb_t *dev, bool enable)
{
    uint8_t v = 0;
    if (LPS22HB_ReadReg(dev, LPS22HB_REG_CTRL_REG1, &v, 1) != HAL_OK) return HAL_ERROR;
    if (enable) v |= CTRL1_BDU; else v &= ~CTRL1_BDU;
    return LPS22HB_WriteReg(dev, LPS22HB_REG_CTRL_REG1, &v, 1);
}

/* 低通配置（EN_LPFP/LPFP_CFG） */
HAL_StatusTypeDef LPS22HB_SetLPF(lps22hb_t *dev, lps22hb_lpf_t lpf_cfg)
{
    uint8_t v = 0;
    if (LPS22HB_ReadReg(dev, LPS22HB_REG_CTRL_REG1, &v, 1) != HAL_OK) return HAL_ERROR;
    /* 清空 */
    v &= ~(CTRL1_EN_LPFP | CTRL1_LPFP_CFG);
    if (lpf_cfg == LPS22HB_LPFP_ODR_9) {
        v |= CTRL1_EN_LPFP; /* cfg=0 */
    } else if (lpf_cfg == LPS22HB_LPFP_ODR_20) {
        v |= CTRL1_EN_LPFP | CTRL1_LPFP_CFG;
    } /* OFF 时维持清零 */
    return LPS22HB_WriteReg(dev, LPS22HB_REG_CTRL_REG1, &v, 1);
}

/* One-shot：在 PD 下触发一次转换，等待 STATUS */
HAL_StatusTypeDef LPS22HB_OneShot(lps22hb_t *dev)
{
    uint8_t v = 0;
    /* 确保 ODR=0（PowerDown） */
    if (LPS22HB_ReadReg(dev, LPS22HB_REG_CTRL_REG1, &v, 1) != HAL_OK) return HAL_ERROR;
    if (((v & CTRL1_ODR_MASK) >> CTRL1_ODR_POS) != LPS22HB_ODR_POWER_DOWN) {
        v = (v & ~CTRL1_ODR_MASK);
        if (LPS22HB_WriteReg(dev, LPS22HB_REG_CTRL_REG1, &v, 1) != HAL_OK) return HAL_ERROR;
    }
    /* 触发 ONE_SHOT */
    if (LPS22HB_ReadReg(dev, LPS22HB_REG_CTRL_REG2, &v, 1) != HAL_OK) return HAL_ERROR;
    v |= CTRL2_ONE_SHOT;
    if (LPS22HB_WriteReg(dev, LPS22HB_REG_CTRL_REG2, &v, 1) != HAL_OK) return HAL_ERROR;
    /* 等待 STATUS P_DA/T_DA 置位 */
    return wait_flag(dev, LPS22HB_REG_STATUS, (STATUS_P_DA|STATUS_T_DA), 1, 20);
}

/* 初始化：复位 -> IF_ADD_INC=1 -> 配置 ODR/BDU/LPF */
HAL_StatusTypeDef LPS22HB_Init(lps22hb_t *dev,
                               lps22hb_odr_t odr,
                               bool bdu_lock,
                               lps22hb_lpf_t lpf_cfg)
{
    if (!dev || !dev->hspi) return HAL_ERROR;
    if (dev->spi_timeout == 0) dev->spi_timeout = 50;

    /* 芯片在手否？ */
    uint8_t id = 0;
    if (LPS22HB_ReadWhoAmI(dev, &id) != HAL_OK) return HAL_ERROR;
    if (id != LPS22HB_WHO_AM_I_VAL) return HAL_ERROR; /* 不是目标器件 */

    /* 软件复位并重启 NVM 参数 */
    if (LPS22HB_SoftwareReset(dev) != HAL_OK) return HAL_ERROR;
    if (LPS22HB_RebootMemory(dev) != HAL_OK) return HAL_ERROR;

    /* 使能地址自增（IF_ADD_INC=1），关闭 I2C（可选，SPI 项目常关） */
    uint8_t c2 = 0;
    if (LPS22HB_ReadReg(dev, LPS22HB_REG_CTRL_REG2, &c2, 1) != HAL_OK) return HAL_ERROR;
    c2 |= CTRL2_IF_ADD_INC;
    c2 |= CTRL2_I2C_DIS; /* 纯 SPI 项目可打开，避免线路噪声 */
    c2 &= ~CTRL2_FIFO_EN; /* 初始禁用 FIFO */
    if (LPS22HB_WriteReg(dev, LPS22HB_REG_CTRL_REG2, &c2, 1) != HAL_OK) return HAL_ERROR;

    /* 配置 CTRL_REG1：ODR/BDU/LPF */
    if (LPS22HB_SetODR(dev, odr) != HAL_OK) return HAL_ERROR;
    if (LPS22HB_SetBDU(dev, bdu_lock) != HAL_OK) return HAL_ERROR;
    if (LPS22HB_SetLPF(dev, lpf_cfg) != HAL_OK) return HAL_ERROR;

    return HAL_OK;
}

/* 
状态类型，通常返回 HAL_OK (0) 或 HAL_ERROR (非0)。它表示函数执行的成功或失败。
读取原始压力：24-bit 两补，低位在 XL（28h）
*/
HAL_StatusTypeDef LPS22HB_ReadRawPressure(lps22hb_t *dev, int32_t *raw)
{
    uint8_t buf[3];
    if (LPS22HB_ReadReg(dev, LPS22HB_REG_PRESS_OUT_XL, buf, 3) != HAL_OK) return HAL_ERROR;
    int32_t v = ((int32_t)buf[2]<<16) | ((int32_t)buf[1]<<8) | buf[0];
    /* 扩展到 32bit 两补 */
    if (v & 0x00800000) v |= 0xFF000000;
    *raw = v;
    return HAL_OK;
}

/* 读取原始温度：16-bit 两补，低位在 L（2Bh） */
HAL_StatusTypeDef LPS22HB_ReadRawTemperature(lps22hb_t *dev, int16_t *raw)
{
    uint8_t buf[2];
    if (LPS22HB_ReadReg(dev, LPS22HB_REG_TEMP_OUT_L, buf, 2) != HAL_OK) return HAL_ERROR;
    int16_t v = (int16_t)((buf[1]<<8) | buf[0]);
    *raw = v;
    return HAL_OK;
}

/* 压力换算：hPa = raw / 4096.0 （datasheet） */
HAL_StatusTypeDef LPS22HB_ReadPressure_hPa(lps22hb_t *dev, float *p_hPa)
{
    int32_t raw;
    if (LPS22HB_ReadRawPressure(dev, &raw) != HAL_OK) return HAL_ERROR;
    *p_hPa = ((float)raw) / 4096.0f;
    return HAL_OK;
}

/* 温度换算：°C = raw / 100.0  （Tsens=100 LSB/°C） */
HAL_StatusTypeDef LPS22HB_ReadTemperature_C(lps22hb_t *dev, float *t_C)
{
    int16_t raw;
    if (LPS22HB_ReadRawTemperature(dev, &raw) != HAL_OK) return HAL_ERROR;
    *t_C = ((float)raw) / 100.0f;
    return HAL_OK;
}

/* FIFO 配置：enable + 模式 + 水位（0~31 表示 level-1） */
HAL_StatusTypeDef LPS22HB_SetFIFO(lps22hb_t *dev, bool enable, lps22hb_fifo_mode_t mode, uint8_t watermark)
{
    uint8_t c2 = 0;
    if (LPS22HB_ReadReg(dev, LPS22HB_REG_CTRL_REG2, &c2, 1) != HAL_OK) return HAL_ERROR;

    if (enable) c2 |= CTRL2_FIFO_EN;
    else        c2 &= ~CTRL2_FIFO_EN;

    if (LPS22HB_WriteReg(dev, LPS22HB_REG_CTRL_REG2, &c2, 1) != HAL_OK) return HAL_ERROR;

    uint8_t fifo = 0;
    watermark &= FIFO_WTM_MASK;
    fifo = (uint8_t)((mode<<5) & FIFO_MODE_MASK) | (watermark & FIFO_WTM_MASK);

    return LPS22HB_WriteReg(dev, LPS22HB_REG_FIFO_CTRL, &fifo, 1);
}

HAL_StatusTypeDef LPS22HB_ReadFIFO_Level(lps22hb_t *dev, uint8_t *level)
{
    uint8_t st = 0;
    if (LPS22HB_ReadReg(dev, LPS22HB_REG_FIFO_STATUS, &st, 1) != HAL_OK) return HAL_ERROR;
    *level = st & 0x3F; /* FSS5:0 */
    return HAL_OK;
}



/* ---------- 工具：等待寄存器位 置位/清零 ---------- */
static HAL_StatusTypeDef wait_flag(lps22hb_t *dev, uint8_t reg, uint8_t mask, uint8_t expect_set, uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    uint8_t v = 0;
    do {
        if (LPS22HB_ReadReg(dev, reg, &v, 1) != HAL_OK) return HAL_ERROR;
        if (expect_set) {
            if ((v & mask) == mask) return HAL_OK;
        } else {
            if ((v & mask) == 0) return HAL_OK;
        }
    } while ((HAL_GetTick() - t0) < timeout_ms);
    return HAL_TIMEOUT;
}

/* 4096 LSB/hPa：datasheet Psens。用于 hPa <-> LSB 的换算 */
static inline int16_t rpds_hPa_to_raw(float hPa)
{
    /* RPDS 是 16 位两补，只适合小范围“一点校准”
       16bit 范围约 ±32768 LSB ≈ ±8.0 hPa */
    float lsb = hPa * 4096.0f;
    if (lsb >  32767.0f) lsb =  32767.0f;
    if (lsb < -32768.0f) lsb = -32768.0f;
    return (int16_t) (lsb);
}

static inline float rpds_raw_to_hPa(int16_t raw)
{
    return ((float)raw) / 4096.0f;
}

/* 写 RPDS（单位：hPa） -> RPDS_H/L 两补 */
HAL_StatusTypeDef LPS22HB_WriteRPDS_hPa(lps22hb_t *dev, float offset_hPa)
{
    int16_t raw = rpds_hPa_to_raw(offset_hPa);
    uint8_t buf[2] = { (uint8_t)(raw & 0xFF), (uint8_t)((raw >> 8) & 0xFF) }; /* L=18h, H=19h */
    /* 注意：芯片定义 PRESS_OUT = measured - RPDS（见手册），
       故写正值表示“减去正偏移”。*/
    if (LPS22HB_WriteReg(dev, 0x18, &buf[0], 1) != HAL_OK) return HAL_ERROR; /* RPDS_L */
    if (LPS22HB_WriteReg(dev, 0x19, &buf[1], 1) != HAL_OK) return HAL_ERROR; /* RPDS_H */
    return HAL_OK;
}

/* 读 RPDS（返回单位：hPa） */
HAL_StatusTypeDef LPS22HB_ReadRPDS_hPa(lps22hb_t *dev, float *offset_hPa)
{
    uint8_t buf[2];
    if (LPS22HB_ReadReg(dev, 0x18, &buf[0], 1) != HAL_OK) return HAL_ERROR; /* RPDS_L */
    if (LPS22HB_ReadReg(dev, 0x19, &buf[1], 1) != HAL_OK) return HAL_ERROR; /* RPDS_H */
    int16_t raw = (int16_t)((buf[1] << 8) | buf[0]);
    *offset_hPa = rpds_raw_to_hPa(raw);
    return HAL_OK;
}

/* 清零 RPDS */
HAL_StatusTypeDef LPS22HB_ClearRPDS(lps22hb_t *dev)
{
    uint8_t zero[2] = {0, 0};
    if (LPS22HB_WriteReg(dev, 0x18, &zero[0], 1) != HAL_OK) return HAL_ERROR; /* RPDS_L */
    if (LPS22HB_WriteReg(dev, 0x19, &zero[1], 1) != HAL_OK) return HAL_ERROR; /* RPDS_H */
    return HAL_OK;
}

/* Baro_Init:
 * - 每次调用都会对 LPS22HB 做一次 SoftwareReset + Reboot
 * - 常规 ODR/BDU/LPF 配置：50Hz、BDU=1、LPF=ODR/20（你可按需改）
 */
HAL_StatusTypeDef Baro_Init(void)
{
    if (LPS22HB_Init(&g_lps, LPS22HB_ODR_50HZ, true, LPS22HB_LPFP_ODR_20) != HAL_OK)
		{
			printf("Baro_Init error\n");
			return HAL_ERROR;
		}
		// 需要等待LPS22HB正常工作
		HAL_Delay(500);
		
		float sum = 0.0f;
    for (uint8_t i = 0; i < average_N; i++) {
        float p;
        if (LPS22HB_ReadPressure_hPa(&g_lps, &p) != HAL_OK) return HAL_ERROR;
        sum += p;
        /* 若在 FreeRTOS 任务里调用，可适当延时 5~10ms */
        HAL_Delay(5);
    }
		g_P0_hpa = sum / (float)average_N;
    return HAL_OK;
}

// 读取气压计相对大气压: 以初始化时g_P0_hpa的气压为基准
HAL_StatusTypeDef Baro_RelativePressure_hPa(float* r_hPa)
{
	float p;
	if (LPS22HB_ReadPressure_hPa(&g_lps, &p) != HAL_OK) return HAL_ERROR;
	*r_hPa = p - g_P0_hpa;
	return HAL_OK;
}

// 读取气压计温度
HAL_StatusTypeDef Baro_ReadTemperature_C(float* t_C)
{
	if (LPS22HB_ReadPressure_hPa(&g_lps, t_C) != HAL_OK) return HAL_ERROR;
	return HAL_OK;
}


//void Baro_read_pre
