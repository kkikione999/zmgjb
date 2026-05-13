# STM32 Runtime Overview

This file is a short map of the current STM32F411 firmware runtime structure for AI/code navigation.

## Scope

- Target: `code/411`
- Main MCU firmware: STM32F411
- Runtime model: HAL + FreeRTOS

## Main Entry

- Entry file: [/Users/ll/fly/zmgjb/code/411/Core/Src/main.c](/Users/ll/fly/zmgjb/code/411/Core/Src/main.c)
- Entry function: `main()`

Startup order in `main()`:

1. `HAL_Init()`
2. `SystemClock_Config()`
3. Peripheral init:
   - `MX_GPIO_Init()`
   - `MX_DMA_Init()`
   - `MX_ADC1_Init()`
   - `MX_I2C1_Init()`
   - `MX_SPI3_Init()`
   - `MX_USART1_UART_Init()`
   - `MX_USART2_UART_Init()`
   - `MX_TIM1_Init()`
   - `MX_TIM2_Init()`
   - `MX_TIM3_Init()`
4. User init:
   - `Baro_Init()`
   - `usart2_dma_rx_start()`
   - `ICM42688_init()`
   - `QMC_Init()`
   - `mahony_ahrs_init()`
   - `motor_all_start()`
5. RTOS start:
   - `osKernelInitialize()`
   - `MX_FREERTOS_Init()`
   - `osKernelStart()`

## Core Runtime Model

Main runtime file:

- [/Users/ll/fly/zmgjb/code/411/Core/Src/freertos.c](/Users/ll/fly/zmgjb/code/411/Core/Src/freertos.c)

Important queues created in `MX_FREERTOS_Init()`:

- `sensorQueueHandle`: filtered sensor packets
- `rcCmdQueueHandle`: latest RC command from ESP32
- `pidUpdateQueueHandle`: PID update messages
- `g_txQueueHandle`: TX messages to ESP32

Currently created tasks:

- `posture_task()`
- `sensor_task()`
- `esp32_rx_task()`
- `maneuver_task()`

Currently defined but not started:

- `esp32_tx_task()`
- `pid_update_task()`

## Actual Data Flow

### 1. Sensor path

- Sensor read task: `sensor_task()`
- Files:
  - [/Users/ll/fly/zmgjb/code/411/Core/Src/freertos.c](/Users/ll/fly/zmgjb/code/411/Core/Src/freertos.c)
  - [/Users/ll/fly/zmgjb/code/411/Control/filter.c](/Users/ll/fly/zmgjb/code/411/Control/filter.c)
  - [/Users/ll/fly/zmgjb/code/411/Hardware/ICM42688.c](/Users/ll/fly/zmgjb/code/411/Hardware/ICM42688.c)
  - [/Users/ll/fly/zmgjb/code/411/Hardware/QMC5883P.c](/Users/ll/fly/zmgjb/code/411/Hardware/QMC5883P.c)
  - [/Users/ll/fly/zmgjb/code/411/Hardware/LPS22HBTR.c](/Users/ll/fly/zmgjb/code/411/Hardware/LPS22HBTR.c)

Flow:

- `sensor_task()` reads ICM42688 raw accel/gyro
- applies low-pass filtering
- reads QMC5883P magnetometer, preferring calibrated data
- reads LPS22HB pressure
- packs `sensor_data_t`
- pushes to `sensorQueueHandle`

### 2. Attitude path

- Attitude task: `posture_task()`
- AHRS file: [/Users/ll/fly/zmgjb/code/411/Control/AHRS_Mahony.c](/Users/ll/fly/zmgjb/code/411/Control/AHRS_Mahony.c)

Flow:

- `posture_task()` gets `sensor_data_t` from `sensorQueueHandle`
- calls `mahony_ahrs_update_mag()`
- reads result through `mahony_get_euler()`

Important note:

- AHRS is running, but its output is not currently fed into motor closed-loop control.

### 3. RC / communication path

- UART/DMA files:
  - [/Users/ll/fly/zmgjb/code/411/Core/Src/usart.c](/Users/ll/fly/zmgjb/code/411/Core/Src/usart.c)
  - [/Users/ll/fly/zmgjb/code/411/Core/Src/dma.c](/Users/ll/fly/zmgjb/code/411/Core/Src/dma.c)

Flow:

- `usart2_dma_rx_start()` starts USART2 circular DMA RX
- `esp32_rx_task()` pulls bytes with `uart2_rx_getc()`
- frame parser reconstructs protocol: `HEADER + CMD + LEN + DATA + SUM + TAIL`
- dispatch happens through `cmd_table`

Important handlers in `usart.c`:

- `parseJoystick()`: converts ESP32 joystick floats into `rc_cmd_t`
- `setPitchPIDparameter()`
- `setRollPIDparameter()`
- `setYawPIDparameter()`

### 4. Motor output path

- Motor control task: `maneuver_task()`
- PWM file: [/Users/ll/fly/zmgjb/code/411/Core/Src/tim.c](/Users/ll/fly/zmgjb/code/411/Core/Src/tim.c)

Flow:

- `maneuver_task()` reads latest `rc_cmd_t` from `rcCmdQueueHandle`
- applies simple failsafe if RC timeout is too long
- performs direct X-frame mixing
- writes PWM with `__HAL_TIM_SET_COMPARE()`

Motor PWM startup function:

- `motor_all_start()`

Current state:

- motor output is driven by direct RC mixing
- no real attitude PID loop is connected yet

## Important Drivers

### IMU

- File: [/Users/ll/fly/zmgjb/code/411/Hardware/ICM42688.c](/Users/ll/fly/zmgjb/code/411/Hardware/ICM42688.c)
- Init function: `ICM42688_init()`

Current configuration in code:

- gyro ODR: 1kHz
- accel ODR: 1kHz
- gyro FSR: +-2000dps
- accel FSR: +-8g

### Magnetometer

- File: [/Users/ll/fly/zmgjb/code/411/Hardware/QMC5883P.c](/Users/ll/fly/zmgjb/code/411/Hardware/QMC5883P.c)
- Init function: `QMC_Init()`

### Barometer

- File: [/Users/ll/fly/zmgjb/code/411/Hardware/LPS22HBTR.c](/Users/ll/fly/zmgjb/code/411/Hardware/LPS22HBTR.c)
- Init function: `Baro_Init()`

## Interrupt / Time Base

- IRQ file: [/Users/ll/fly/zmgjb/code/411/Core/Src/stm32f4xx_it.c](/Users/ll/fly/zmgjb/code/411/Core/Src/stm32f4xx_it.c)
- HAL tick source: [/Users/ll/fly/zmgjb/code/411/Core/Src/stm32f4xx_hal_timebase_tim.c](/Users/ll/fly/zmgjb/code/411/Core/Src/stm32f4xx_hal_timebase_tim.c)

Key point:

- HAL time base uses `TIM10`
- current control and sensor flow are mainly task-driven, not interrupt-driven

## Parameter / PID Storage

- Files:
  - [/Users/ll/fly/zmgjb/code/411/system_param/system_params.c](/Users/ll/fly/zmgjb/code/411/system_param/system_params.c)
  - [/Users/ll/fly/zmgjb/code/411/PID/pid.c](/Users/ll/fly/zmgjb/code/411/PID/pid.c)

Relevant functions:

- `SystemParams_Init()`
- `SystemParams_Load()`
- `SystemParams_Save()`
- `PID_LoadFromParams()`

Current note:

- these parameter load paths exist, but should be checked before assuming they are active in startup
- `pid_update_task()` exists, but is currently not started in `MX_FREERTOS_Init()`

## Files AI Should Read First

If you need to understand STM32 behavior quickly, start here:

1. [/Users/ll/fly/zmgjb/code/411/Core/Src/main.c](/Users/ll/fly/zmgjb/code/411/Core/Src/main.c)
2. [/Users/ll/fly/zmgjb/code/411/Core/Src/freertos.c](/Users/ll/fly/zmgjb/code/411/Core/Src/freertos.c)
3. [/Users/ll/fly/zmgjb/code/411/Core/Src/usart.c](/Users/ll/fly/zmgjb/code/411/Core/Src/usart.c)
4. [/Users/ll/fly/zmgjb/code/411/Core/Src/dma.c](/Users/ll/fly/zmgjb/code/411/Core/Src/dma.c)
5. [/Users/ll/fly/zmgjb/code/411/Control/AHRS_Mahony.c](/Users/ll/fly/zmgjb/code/411/Control/AHRS_Mahony.c)
6. [/Users/ll/fly/zmgjb/code/411/Hardware/ICM42688.c](/Users/ll/fly/zmgjb/code/411/Hardware/ICM42688.c)
7. [/Users/ll/fly/zmgjb/code/411/Hardware/QMC5883P.c](/Users/ll/fly/zmgjb/code/411/Hardware/QMC5883P.c)
8. [/Users/ll/fly/zmgjb/code/411/Hardware/LPS22HBTR.c](/Users/ll/fly/zmgjb/code/411/Hardware/LPS22HBTR.c)

## One-Line Summary

Current STM32 runtime is: sensor polling + AHRS estimation + ESP32 joystick RX + direct motor mixing under FreeRTOS, with some PID/parameter infrastructure present but not fully wired into the active control loop.
