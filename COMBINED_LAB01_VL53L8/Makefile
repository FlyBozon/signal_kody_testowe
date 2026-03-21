##############################################################################
# Makefile dla STM32L476RG
# Projekt: COMBINED_LAB01_VL53L8
#   - IKS4A1: czujniki ruchu MEMS (LSM6DSV16X) + NanoEdge AI
#   - X-NUCLEO-53L8A1: czujnik ToF VL53L8CX (macierz 8x8)
#
# Użycie:
#   make              – buduj firmware
#   make flash        – wgraj przez ST-Link (st-flash)
#   make flash-openocd – wgraj przez OpenOCD
#   make clean        – usuń artefakty
#   make size         – rozmiar firmware
##############################################################################

PREFIX  = arm-none-eabi-
CC      = $(PREFIX)gcc
AS      = $(PREFIX)gcc -x assembler-with-cpp
OBJCOPY = $(PREFIX)objcopy
OBJDUMP = $(PREFIX)objdump
SIZE    = $(PREFIX)size

TARGET  = Combined_IKS4A1_VL53L8
BUILD   = build

CPU    = -mcpu=cortex-m4
FPU    = -mfpu=fpv4-sp-d16
FLOAT  = -mfloat-abi=hard
THUMB  = -mthumb
MCU    = $(CPU) $(FPU) $(FLOAT) $(THUMB)

LDSCRIPT = STM32L476RGTX_FLASH.ld

DEFS = \
  -DDEBUG \
  -DUSE_HAL_DRIVER \
  -DSTM32L476xx

# ---------------------------------------------------------------------------
# Ścieżki include
# ---------------------------------------------------------------------------
INCS = \
  -ICore/Inc \
  -IDrivers/STM32L4xx_HAL_Driver/Inc \
  -IDrivers/STM32L4xx_HAL_Driver/Inc/Legacy \
  -IDrivers/CMSIS/Device/ST/STM32L4xx/Include \
  -IDrivers/CMSIS/Include \
  \
  -IX-CUBE-MEMS1/Target \
  -IDrivers/BSP/IKS4A1 \
  -IDrivers/BSP/Components/Common \
  -IDrivers/BSP/Components/lsm6dsv16x \
  -IDrivers/BSP/Components/lis2duxs12 \
  -IDrivers/BSP/Components/lis2mdl \
  -IDrivers/BSP/Components/lsm6dso16is \
  -IDrivers/BSP/Components/sht40ad1b \
  -IDrivers/BSP/Components/lps22df \
  -IDrivers/BSP/Components/stts22h \
  -INEAI_Lib/Inc \
  \
  -IX-CUBE-TOF1/Target \
  -IDrivers/BSP/53L8A1 \
  -IDrivers/BSP/Components/vl53l8cx \
  -IDrivers/BSP/Components/vl53l8cx/modules \
  -IDrivers/BSP/Components/vl53l8cx/porting \
  -IMiddlewares/Third_Party/FatFs/src

CFLAGS = $(MCU) -std=gnu11 -g3 -O0 \
  -ffunction-sections -fdata-sections \
  -Wall -fstack-usage \
  --specs=nano.specs \
  $(DEFS) $(INCS)

ASFLAGS = $(MCU) -g3 --specs=nano.specs

LDFLAGS = $(MCU) \
  -T$(LDSCRIPT) \
  --specs=nosys.specs \
  --specs=nano.specs \
  -Wl,-Map=$(BUILD)/$(TARGET).map \
  -Wl,--gc-sections \
  -static \
  -LNEAI_Lib/Lib \
  -Wl,--start-group -lc -lm -lneai -Wl,--end-group

# ---------------------------------------------------------------------------
# Pliki źródłowe C
# ---------------------------------------------------------------------------
C_SRCS = \
  Core/Src/gpio.c \
  Core/Src/main.c \
  Core/Src/stm32l4xx_hal_msp.c \
  Core/Src/stm32l4xx_it.c \
  Core/Src/stm32l4xx_nucleo_bus.c \
  Core/Src/syscalls.c \
  Core/Src/sysmem.c \
  Core/Src/system_stm32l4xx.c \
  Core/Src/usart.c \
  Core/Src/spi.c \
  Core/Src/gps.c \
  Core/Src/user_diskio.c \
  Core/Src/stm32_adafruit_sd.c \
  \
  Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal.c \
  Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cortex.c \
  Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma.c \
  Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma_ex.c \
  Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_exti.c \
  Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash.c \
  Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ex.c \
  Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ramfunc.c \
  Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_gpio.c \
  Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c.c \
  Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c_ex.c \
  Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr.c \
  Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr_ex.c \
  Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc.c \
  Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc_ex.c \
  Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart.c \
  Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart_ex.c \
  Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_spi.c \
  Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_spi_ex.c \
  \
  Drivers/BSP/IKS4A1/iks4a1_env_sensors.c \
  Drivers/BSP/IKS4A1/iks4a1_env_sensors_ex.c \
  Drivers/BSP/IKS4A1/iks4a1_motion_sensors.c \
  Drivers/BSP/IKS4A1/iks4a1_motion_sensors_ex.c \
  Drivers/BSP/Components/lsm6dsv16x/lsm6dsv16x.c \
  Drivers/BSP/Components/lsm6dsv16x/lsm6dsv16x_reg.c \
  Drivers/BSP/Components/lis2duxs12/lis2duxs12.c \
  Drivers/BSP/Components/lis2duxs12/lis2duxs12_reg.c \
  Drivers/BSP/Components/lis2mdl/lis2mdl.c \
  Drivers/BSP/Components/lis2mdl/lis2mdl_reg.c \
  Drivers/BSP/Components/lsm6dso16is/lsm6dso16is.c \
  Drivers/BSP/Components/lsm6dso16is/lsm6dso16is_reg.c \
  Drivers/BSP/Components/sht40ad1b/sht40ad1b.c \
  Drivers/BSP/Components/sht40ad1b/sht40ad1b_reg.c \
  Drivers/BSP/Components/lps22df/lps22df.c \
  Drivers/BSP/Components/lps22df/lps22df_reg.c \
  Drivers/BSP/Components/stts22h/stts22h.c \
  Drivers/BSP/Components/stts22h/stts22h_reg.c \
  \
  Drivers/BSP/53L8A1/53l8a1_ranging_sensor.c \
  Drivers/BSP/Components/vl53l8cx/vl53l8cx.c \
  Drivers/BSP/Components/vl53l8cx/modules/vl53l8cx_api.c \
  Drivers/BSP/Components/vl53l8cx/modules/vl53l8cx_plugin_detection_thresholds.c \
  Drivers/BSP/Components/vl53l8cx/modules/vl53l8cx_plugin_motion_indicator.c \
  Drivers/BSP/Components/vl53l8cx/modules/vl53l8cx_plugin_xtalk.c \
  Drivers/BSP/Components/vl53l8cx/porting/platform.c \
  \
  Middlewares/Third_Party/FatFs/src/ff.c \
  Middlewares/Third_Party/FatFs/src/diskio.c \
  Middlewares/Third_Party/FatFs/src/ff_gen_drv.c \
  Middlewares/Third_Party/FatFs/src/option/unicode.c \
  Middlewares/Third_Party/FatFs/src/option/syscall.c

ASM_SRCS = \
  Core/Startup/startup_stm32l476rgtx.s

C_OBJS   = $(patsubst %.c, $(BUILD)/%.o, $(C_SRCS))
ASM_OBJS = $(patsubst %.s, $(BUILD)/%.o, $(ASM_SRCS))
OBJS     = $(C_OBJS) $(ASM_OBJS)
DEPS     = $(OBJS:.o=.d)

##############################################################################
# Cele
##############################################################################

.PHONY: all flash flash-openocd clean size

all: $(BUILD)/$(TARGET).elf $(BUILD)/$(TARGET).bin $(BUILD)/$(TARGET).hex
	@$(SIZE) $<

$(BUILD)/$(TARGET).elf: $(OBJS) $(LDSCRIPT)
	@echo "  LD   $@"
	@$(CC) -o $@ $(OBJS) $(LDFLAGS)

$(BUILD)/$(TARGET).bin: $(BUILD)/$(TARGET).elf
	@echo "  BIN  $@"
	@$(OBJCOPY) -O binary $< $@

$(BUILD)/$(TARGET).hex: $(BUILD)/$(TARGET).elf
	@echo "  HEX  $@"
	@$(OBJCOPY) -O ihex $< $@

$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	@echo "  CC   $<"
	@$(CC) $(CFLAGS) -MMD -MP -c $< -o $@

$(BUILD)/%.o: %.s
	@mkdir -p $(dir $@)
	@echo "  AS   $<"
	@$(AS) $(ASFLAGS) -MMD -MP -c $< -o $@

-include $(DEPS)

size: $(BUILD)/$(TARGET).elf
	$(SIZE) $<

flash: $(BUILD)/$(TARGET).bin
	st-flash --reset write $< 0x8000000

flash-openocd: $(BUILD)/$(TARGET).elf
	openocd \
	  -f interface/stlink.cfg \
	  -f target/stm32l4x.cfg \
	  -c "program $< verify reset exit"

clean:
	rm -rf $(BUILD)
