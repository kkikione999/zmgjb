# 芯片手册索引

按当前代码和主控原理图里实际用到、需要寄存器配置/编程的器件整理。

## 代码与原理图依据

- `STM32F411CEU6`
  - `code/411/Flyer_v1.1.0.ioc`
  - `code/411/Makefile`
  - `hardware/原理图/主控/SCH_主控_1-P1_2026-03-11.pdf`
- `ESP32-C3`
  - `code/C3/platformio.ini`
- `ICM-42688-P`
  - `code/411/Hardware/ICM42688.h`
  - `hardware/原理图/主控/SCH_主控_1-P1_2026-03-11.pdf`
- `LPS22HBTR`
  - `code/411/Hardware/LPS22HBTR.c`
  - `hardware/原理图/主控/SCH_主控_1-P1_2026-03-11.pdf`
- `QMC5883P`
  - `code/411/Hardware/QMC5883P_reg.h`
  - `hardware/原理图/主控/SCH_主控_1-P1_2026-03-11.pdf`

## 已下载 PDF

### `stm32`

- `stm32f411ceu6_datasheet.pdf`
  - 来源: Alldatasheet 镜像页重建
- `rm0383_stm32f411_reference_manual.pdf`
  - 来源: `https://www.stmcu.jp/wp/wp-content/uploads/2017/08/RM0383_Rev3.pdf`
- `pm0214_stm32_cortex_m4_programming_manual.pdf`
  - 来源: `https://www.stmcu.jp/wp/wp-content/uploads/2017/08/PM0214_Rev9.pdf`

### `esp32`

- `esp32-c3_datasheet_en.pdf`
  - 来源: `https://documentation.espressif.com/esp32-c3_datasheet_en.pdf`
- `esp32-c3_technical_reference_manual_en.pdf`
  - 来源: `https://documentation.espressif.com/esp32-c3_technical_reference_manual_en.pdf`

### `icm42688`

- `icm-42688-p_datasheet.pdf`
  - 来源: TDK InvenSense 官方 CloudFront 分发链接

### `气压计`

- `lps22hb_datasheet.pdf`
  - 来源: Alldatasheet 镜像页重建

### `磁力计`

- `qmc5883p_datasheet.pdf`
  - 来源: QST 官方链接

## 说明

- `st.com` 的若干 PDF 在当前终端网络环境下无法稳定直连下载，所以 `STM32F411` 数据手册和 `LPS22HB` 数据手册使用镜像页逐页重建成 PDF。
- 其余文档均为可直接获取的原始 PDF。
