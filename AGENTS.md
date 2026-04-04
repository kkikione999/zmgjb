# Repository Guidelines

## Backgrand
- this is a mini drone. the drone is connected to the computer with USB. the drone includes 4 coreless motor and every components below.
- this computer with a mac M4 chip, is connected with WiFi and a Router which shares the network by it's WiFi: SSID is "whc" and Password is "12345678"
- lines connection is fine and don't doubt that.

## Project Structure & Module Organization
- `code/411`: STM32F411 flight-controller firmware. Core CubeMX/HAL code lives in `Core/`, sensors and board drivers in `Hardware/`, attitude/control logic in `Control/`, PID code in `PID/`, and persistent parameters in `system_param/`.
- `code/C3`: ESP32-C3 bridge and UI firmware. Runtime code is in `src/`, web assets for SPIFFS are in `data/`, and PlatformIO test scaffolding is in `test/`.
- `hardware/原理图` and `hardware/芯片手册`: schematics and reference manuals for bring-up and debugging.

## Build, Test, and Development Commands
- `cd code/411 && pio run`: build STM32 firmware.
- `cd code/411 && pio run -t upload`: flash STM32 over ST-Link.
- `cd code/411 && make all` or `make flash`: alternate GCC/OpenOCD flow for the STM32 target.
- `cd code/C3 && pio run`: build ESP32-C3 firmware.
- `cd code/C3 && pio run -t upload`: flash ESP32-C3 over USB CDC.
- `cd code/C3 && pio run -t uploadfs`: upload `data/` assets to SPIFFS after web UI changes.
- `cd code/C3 && pio device monitor -b 115200`: monitor ESP32 logs.

## Coding Style & Naming Conventions
- Match the surrounding file style; do not reformat generated STM32 CubeMX sections without need.
- Prefer 4-space indentation in C/C++ and keep spacing consistent with the edited file.
- Preserve existing naming patterns: module helpers such as `mahony_` and `system_params_`, task-style names such as `Task_init`, and uppercase snake case for macros like `FRAME_HEADER`.
- Keep new filenames lowercase with underscores unless extending an existing module with established naming.

## Testing Guidelines
- There is no repository-wide automated test suite today.
- For every firmware change, build each affected target and record the command used.
- Add isolated ESP32 tests under `code/C3/test/` as `test_<feature>.cpp` when logic can be exercised without hardware.
- For hardware-facing changes, include bench evidence in the PR: serial logs, VOFA output, or web UI screenshots.

## Commit & Pull Request Guidelines
- Keep commit subjects short and imperative, consistent with existing history such as `Update README`.
- PRs should state affected target(s), hardware used, build/test evidence, and any required config changes.
- Call out changes to local settings such as Wi-Fi credentials, VOFA IP/port, upload ports, UART pins, or DMA behavior.

## Configuration & Hardware Notes
- Computer-side baud rates: STM32 `USART1` uses `460800`; ESP32 USB CDC / `Serial` uses `115200`.
