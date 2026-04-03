# STM32F411 Makefile
# Target: STM32F411CEU6

TARGET = Flyer_v1.1.0
BUILD_DIR = build

# Toolchain
PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size

# MCU settings (STM32F411CEU6)
MCU = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard

# Directories
PROJECT_DIR = /Users/ll/fly/zmgjb/code/411
CORE_DIR = $(PROJECT_DIR)/Core
HARDWARE_DIR = $(PROJECT_DIR)/Hardware
CONTROL_DIR = $(PROJECT_DIR)/Control
PID_DIR = $(PROJECT_DIR)/PID
SYSTEM_PARAM_DIR = $(PROJECT_DIR)/system_param
DRIVERS_DIR = $(PROJECT_DIR)/Drivers
MIDDLEWARES_DIR = $(PROJECT_DIR)/Middlewares

# Source files
C_SOURCES = \
	$(CORE_DIR)/Src/main.c \
	$(CORE_DIR)/Src/freertos.c \
	$(CORE_DIR)/Src/gpio.c \
	$(CORE_DIR)/Src/i2c.c \
	$(CORE_DIR)/Src/spi.c \
	$(CORE_DIR)/Src/tim.c \
	$(CORE_DIR)/Src/usart.c \
	$(CORE_DIR)/Src/adc.c \
	$(CORE_DIR)/Src/dma.c \
	$(CORE_DIR)/Src/stm32f4xx_hal_msp.c \
	$(CORE_DIR)/Src/stm32f4xx_hal_timebase_tim.c \
	$(CORE_DIR)/Src/stm32f4xx_it.c \
	$(CORE_DIR)/Src/system_stm32f4xx.c \
	$(HARDWARE_DIR)/ICM42688.c \
	$(HARDWARE_DIR)/QMC5883P.c \
	$(HARDWARE_DIR)/LPS22HBTR.c \
	$(CONTROL_DIR)/AHRS_Mahony.c \
	$(CONTROL_DIR)/filter.c \
	$(CONTROL_DIR)/IMUsolution.c \
	$(CONTROL_DIR)/process.c \
	$(PID_DIR)/pid.c \
	$(SYSTEM_PARAM_DIR)/system_params.c \
	$(MIDDLEWARES_DIR)/Third_Party/FreeRTOS/Source/croutine.c \
	$(MIDDLEWARES_DIR)/Third_Party/FreeRTOS/Source/event_groups.c \
	$(MIDDLEWARES_DIR)/Third_Party/FreeRTOS/Source/list.c \
	$(MIDDLEWARES_DIR)/Third_Party/FreeRTOS/Source/queue.c \
	$(MIDDLEWARES_DIR)/Third_Party/FreeRTOS/Source/stream_buffer.c \
	$(MIDDLEWARES_DIR)/Third_Party/FreeRTOS/Source/tasks.c \
	$(MIDDLEWARES_DIR)/Third_Party/FreeRTOS/Source/timers.c \
	$(MIDDLEWARES_DIR)/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c \
	$(MIDDLEWARES_DIR)/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c \
	$(MIDDLEWARES_DIR)/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c \
	$(DRIVERS_DIR)/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c \
	$(DRIVERS_DIR)/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc.c \
	$(DRIVERS_DIR)/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc_ex.c \
	$(DRIVERS_DIR)/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c \
	$(DRIVERS_DIR)/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c \
	$(DRIVERS_DIR)/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c \
	$(DRIVERS_DIR)/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c \
	$(DRIVERS_DIR)/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c \
	$(DRIVERS_DIR)/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c \
	$(DRIVERS_DIR)/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c \
	$(DRIVERS_DIR)/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c \
	$(DRIVERS_DIR)/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c \
	$(DRIVERS_DIR)/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c \
	$(DRIVERS_DIR)/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c \
	$(DRIVERS_DIR)/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c \
	$(DRIVERS_DIR)/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c \
	$(DRIVERS_DIR)/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c \
	$(DRIVERS_DIR)/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c \
	$(DRIVERS_DIR)/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c \
	$(DRIVERS_DIR)/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c \
	$(DRIVERS_DIR)/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_usart.c

# Assembly files
ASM_SOURCES =  \
	$(DRIVERS_DIR)/CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc/startup_stm32f411xe.s

# Include paths
INCLUDES = \
	-ICore/Inc \
	-IHardware \
	-IControl \
	-IPID \
	-Isystem_param \
	-IDrivers/STM32F4xx_HAL_Driver/Inc \
	-IDrivers/CMSIS/Device/ST/STM32F4xx/Include \
	-IDrivers/CMSIS/Core/Include \
	-IDrivers/CMSIS/RTOS2/Include \
	-IMiddlewares/Third_Party/FreeRTOS/Source/include \
	-IMiddlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 \
	-IMiddlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F

# Defines
DEFS = -DSTM32F411xE -DUSE_HAL_DRIVER

# Compiler flags
CFLAGS = $(MCU) $(DEFS) $(INCLUDES) \
	-Wall -Wextra -Wpedantic \
	-Og -g3 \
	-ffunction-sections -fdata-sections \
	-fno-strict-aliasing

ASFLAGS = $(MCU) $(DEFS) $(INCLUDES) -c -x assembler-with-cpp -Wall

# Linker flags
LDSCRIPT = $(PROJECT_DIR)/Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc/linker/STM32F411XE_FLASH.ld
LDFLAGS = $(MCU) -T$(LDSCRIPT) \
	-Wl,--gc-sections \
	-Wl,-Map=$(BUILD_DIR)/$(TARGET).map \
	--specs=nano.specs \
	--specs=nosys.specs \
	-lm

# Object files
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))

vpath %.c $(dir $(C_SOURCES))
vpath %.s $(dir $(ASM_SOURCES))

# Targets
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).bin $(BUILD_DIR)/$(TARGET).hex

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) $(ASFLAGS) -c $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf
	$(CP) -O binary $< $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf
	$(CP) -O ihex $< $@

$(BUILD_DIR):
	mkdir -p $@

clean:
	rm -rf $(BUILD_DIR)

flash: $(BUILD_DIR)/$(TARGET).bin
	openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
		-c "program $< verify reset exit 0x08000000"

.PHONY: all clean flash
