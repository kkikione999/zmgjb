# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Mini drone project with dual-MCU architecture: STM32F411 (flight controller) + ESP32-C3 (WiFi bridge). The drone is connected to the Mac via USB. Current goal: **drone debugging**.

**Constitution**: Hardware is always correct. If code doesn't work, it's a software problem. Never question pin connections, wiring, or hardware.

## Build & Flash Commands

### STM32F411 (primary target)
```bash
cd code/411
pio run                    # build firmware
pio run -t upload          # flash via ST-Link
make all                   # alternate GCC build
make flash                 # alternate OpenOCD flash
pio device monitor -b 460800  # serial monitor (USART1)
```

### ESP32-C3 (WiFi bridge, secondary)
```bash
cd code/C3
pio run                    # build
pio run -t upload          # flash via USB CDC
pio run -t uploadfs        # upload web UI to SPIFFS
pio device monitor -b 115200  # serial monitor
```

## Architecture

### STM32F411 Runtime (FreeRTOS + HAL)

**Startup sequence** (`Core/Src/main.c`): HAL init → peripheral init (GPIO, DMA, ADC, I2C, SPI, USART1/2, TIM1/2/3) → sensor init (baro, ICM42688, QMC5883P, AHRS) → motor PWM start → FreeRTOS kernel start.

**Active tasks** (created in `freertos.c` → `MX_FREERTOS_Init()`):
- `sensor_task` — reads IMU/mag/baro, filters, pushes to `sensorQueueHandle`
- `posture_task` — pulls sensor data, runs Mahony AHRS (`mahony_ahrs_update_mag`)
- `esp32_rx_task` — USART2 DMA circular RX, frame parser (HEADER+CMD+LEN+DATA+SUM+TAIL), dispatches via `cmd_table`
- `maneuver_task` — reads `rcCmdQueueHandle`, X-frame mixing, writes PWM (direct RC → motor, no PID loop yet)

**Defined but not started**: `esp32_tx_task`, `pid_update_task`

**Key queues**: `sensorQueueHandle`, `rcCmdQueueHandle`, `pidUpdateQueueHandle`, `g_txQueueHandle`, `joystickQueueHandle`

### Inter-MCU Communication

- USART2 (STM32 ↔ ESP32): 115200 baud, DMA RX with circular buffer
- USART1 (STM32 ↔ Computer): 460800 baud, DMA TX for VOFA+ debug output
- Protocol frame: `HEADER + CMD + LEN + DATA + SUM + TAIL`
- ESP32 bridges STM32 to WiFi: web UI control, VOFA+ telemetry, joystick input

### Sensors
| Sensor | Bus | File | Config |
|--------|-----|------|--------|
| ICM-42688-P (IMU) | SPI3 | `Hardware/ICM42688.c` | gyro/accel 1kHz, ±2000dps/±8g |
| QMC5883P (magnetometer) | I2C1 | `Hardware/QMC5883P.c` | — |
| LPS22HBTR (barometer) | I2C1 | `Hardware/LPS22HBTR.c` | — |

### Motor Output
- TIM1 CH1-4 PWM drives 4 coreless motors via X-frame mixing
- PWM range: 0–80 (ARR=999, prescaler=1)
- Motor start: `motor_all_start()` in `tim.c`

### Control Loop Status
- AHRS estimation runs but output is **not connected** to motor control
- PID infrastructure exists (`PID/pid.c`, `system_param/system_params.c`) but `pid_update_task` is not started
- Motor output is direct RC stick mixing — no attitude hold

### ESP32-C3 Role
- WiFi AP/STA (SSID "whc", password "12345678")
- WebSocket-based web UI for joystick control (`data/index.html`)
- VOFA+ data relay for telemetry
- Serial bridge to STM32 via UART

## Code Organization

```
code/411/
├── Core/Src/       # HAL/CubeMX generated: main.c, freertos.c, usart.c, tim.c, dma.c, etc.
├── src/            # PlatformIO build entry (symlinks to Core/Src and module dirs)
├── Hardware/        # Sensor drivers: ICM42688.c, QMC5883P.c, LPS22HBTR.c
├── Control/         # AHRS_Mahony.c, filter.c, IMUsolution.c, process.c
├── PID/             # pid.c — cascaded PID structures
├── system_param/    # system_params.c — Flash persistence
├── Drivers/         # HAL + CMSIS (vendor, don't modify)
├── Middlewares/      # FreeRTOS source (vendor)
├── Flyer_v1.1.0.ioc # STM32CubeMX project config
code/C3/
├── src/             # main.cpp, mywifi, myserial, vofa, webserver, led, task
├── data/            # index.html — web UI
```

## Key Files to Read First

1. `411/Core/Src/main.c` — startup and peripheral init
2. `411/Core/Src/freertos.c` — all task definitions, queues, and RTOS setup
3. `411/Core/Src/usart.c` — DMA RX/TX, frame protocol, joystick parsing, PID parameter commands
4. `411/Control/AHRS_Mahony.c` — attitude estimation
5. `411/Hardware/ICM42688.c` — IMU driver

## Network Config

- Computer WiFi: connected to router SSID "whc", password "12345678"

## Coding Conventions

- Match surrounding file style; don't reformat CubeMX generated sections
- 4-space indentation in C/C++
- Module prefixes: `mahony_`, `system_params_`, `QMC_`, `ICM42688_`, `Baro_`
- Task names: `xxx_task` pattern
- Macros: `UPPER_SNAKE_CASE`
- New filenames: lowercase with underscores

## Debugging Notes

- No automated test suite — verify by building and checking serial output / VOFA+ / motor response
- For hardware-facing changes, provide bench evidence: serial logs, VOFA output, or web UI screenshots
- STM32 printf uses double-buffered DMA TX on USART1 (see `usart.c`)
- ESP32 USB CDC at 115200; STM32 USART1 at 460800 for VOFA+

## Gotchas

- `411/src/` and `411/Core/Src/` contain symlinks (`Control/`, `Hardware/`, `PID/`, `system_param/`) for PlatformIO build — edit source files in the top-level module directories
- CubeMX project file: `411/Flyer_v1.1.0.ioc` — use STM32CubeMX to regenerate HAL code
- PlatformIO uses `-mfloat-abi=softfp`, Makefile uses hard float — binaries from the two build systems are incompatible
- C3 deps: ArduinoJson 7.x, ESPAsyncWebServer 3.x, AsyncTCP 3.x (managed via `lib_deps`)

## Reference Documents

- Chip datasheets and reference manuals: `../hardware/芯片手册/`
- Schematics: `../hardware/原理图/`
- STM32 runtime overview: `../docs/stm32_runtime_overview.md`
