# zmgjb

咱们搞机吧

Embedded project workspace for an STM32F411 + ESP32-C3 hardware platform.

## Repository Layout

- `code/411`: STM32F411 firmware project, drivers, middleware, and build system files.
- `code/C3`: ESP32-C3 firmware project based on PlatformIO, including web UI assets.
- `hardware/原理图`: hardware schematic source files and exported PDFs.
- `hardware/芯片手册`: collected chip manuals and reference PDFs used during development.

## Notes

- Build artifacts and local IDE files are ignored at the repository root.
- Large reference PDFs are kept in the repository because they are part of the hardware bring-up context.
