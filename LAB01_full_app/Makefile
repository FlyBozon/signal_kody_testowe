##############################################################################
# Makefile for STM32L476RG - AnomalyDetection (NanoEdgeAI)
# Target board: NUCLEO-L476RG + IKS4A1 MEMS expansion
#
# Usage:
#   make          - build the firmware
#   make flash    - flash via ST-Link (requires st-flash or openocd)
#   make clean    - remove build artifacts
#   make size     - show firmware size
#   make list     - generate disassembly listing
#
# Requirements:
#   arm-none-eabi-gcc (GNU Tools for STM32 13.3 or compatible)
#   st-flash  OR  openocd  (for 'make flash')
##############################################################################

# --- Toolchain ---------------------------------------------------------------
PREFIX  = arm-none-eabi-
CC      = $(PREFIX)gcc
AS      = $(PREFIX)gcc -x assembler-with-cpp
OBJCOPY = $(PREFIX)objcopy
OBJDUMP = $(PREFIX)objdump
SIZE    = $(PREFIX)size

# --- Project name and output directory ---------------------------------------
TARGET  = AnomalyDetection
BUILD   = build

# --- MCU flags ---------------------------------------------------------------
CPU     = -mcpu=cortex-m4
FPU     = -mfpu=fpv4-sp-d16
FLOAT   = -mfloat-abi=hard
THUMB   = -mthumb
MCU     = $(CPU) $(FPU) $(FLOAT) $(THUMB)

# --- Linker script -----------------------------------------------------------
LDSCRIPT = STM32L476RGTX_FLASH.ld

# --- Preprocessor defines ----------------------------------------------------
DEFS = \
  -DDEBUG \
  -DUSE_HAL_DRIVER \
  -DSTM32L476xx

# --- Include paths -----------------------------------------------------------
INCS = \
  -ICore/Inc \
  -IDrivers/STM32L4xx_HAL_Driver/Inc \
  -IDrivers/STM32L4xx_HAL_Driver/Inc/Legacy \
  -IDrivers/CMSIS/Device/ST/STM32L4xx/Include \
  -IDrivers/CMSIS/Include \
  -IX-CUBE-MEMS1/Target \
  -IDrivers/BSP/Components/lsm6dsv16x \
  -IDrivers/BSP/Components/lis2duxs12 \
  -IDrivers/BSP/Components/lis2mdl \
  -IDrivers/BSP/Components/lsm6dso16is \
  -IDrivers/BSP/Components/sht40ad1b \
  -IDrivers/BSP/Components/lps22df \
  -IDrivers/BSP/Components/stts22h \
  -IDrivers/BSP/IKS4A1 \
  -IDrivers/BSP/Components/Common \
  -INEAI_Lib/Inc

# --- Compiler flags ----------------------------------------------------------
CFLAGS = $(MCU) -std=gnu11 -g3 -O0 \
  -ffunction-sections -fdata-sections \
  -Wall -fstack-usage \
  --specs=nano.specs \
  $(DEFS) $(INCS)

# --- Assembler flags ---------------------------------------------------------
ASFLAGS = $(MCU) -g3 --specs=nano.specs

# --- Linker flags ------------------------------------------------------------
LDFLAGS = $(MCU) \
  -T$(LDSCRIPT) \
  --specs=nosys.specs \
  --specs=nano.specs \
  -Wl,-Map=$(BUILD)/$(TARGET).map \
  -Wl,--gc-sections \
  -static \
  -LNEAI_Lib/Lib \
  -Wl,--start-group -lc -lm -lneai -Wl,--end-group

# --- C source files ----------------------------------------------------------
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
  Drivers/BSP/Components/stts22h/stts22h_reg.c

# --- Assembly source files ---------------------------------------------------
ASM_SRCS = \
  Core/Startup/startup_stm32l476rgtx.s

# --- Object files (mirroring source tree under build/) -----------------------
C_OBJS   = $(patsubst %.c,   $(BUILD)/%.o, $(C_SRCS))
ASM_OBJS = $(patsubst %.s,   $(BUILD)/%.o, $(ASM_SRCS))
OBJS     = $(C_OBJS) $(ASM_OBJS)

# --- Dependency files (auto-generated) ---------------------------------------
DEPS = $(OBJS:.o=.d)

##############################################################################
# Build targets
##############################################################################

.PHONY: all flash flash-openocd clean size list

all: $(BUILD)/$(TARGET).elf $(BUILD)/$(TARGET).bin $(BUILD)/$(TARGET).hex
	@$(SIZE) $<

# --- Link --------------------------------------------------------------------
$(BUILD)/$(TARGET).elf: $(OBJS) $(LDSCRIPT)
	@echo "  LD   $@"
	@$(CC) -o $@ $(OBJS) $(LDFLAGS)

# --- Binary / Hex outputs ----------------------------------------------------
$(BUILD)/$(TARGET).bin: $(BUILD)/$(TARGET).elf
	@echo "  BIN  $@"
	@$(OBJCOPY) -O binary $< $@

$(BUILD)/$(TARGET).hex: $(BUILD)/$(TARGET).elf
	@echo "  HEX  $@"
	@$(OBJCOPY) -O ihex $< $@

# --- Compile C sources -------------------------------------------------------
$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	@echo "  CC   $<"
	@$(CC) $(CFLAGS) -MMD -MP -c $< -o $@

# --- Assemble startup file ---------------------------------------------------
$(BUILD)/%.o: %.s
	@mkdir -p $(dir $@)
	@echo "  AS   $<"
	@$(AS) $(ASFLAGS) -MMD -MP -c $< -o $@

# --- Include auto-generated dependency files ---------------------------------
-include $(DEPS)

##############################################################################
# Utility targets
##############################################################################

size: $(BUILD)/$(TARGET).elf
	$(SIZE) $<

list: $(BUILD)/$(TARGET).elf
	$(OBJDUMP) -h -S $< > $(BUILD)/$(TARGET).list
	@echo "Listing written to $(BUILD)/$(TARGET).list"

# --- Flash via st-flash (STML-LINK / stlink-tools) ---------------------------
flash: $(BUILD)/$(TARGET).bin
	st-flash --reset write $< 0x8000000

# --- Flash via OpenOCD (alternative) -----------------------------------------
flash-openocd: $(BUILD)/$(TARGET).elf
	openocd \
	  -f interface/stlink.cfg \
	  -f target/stm32l4x.cfg \
	  -c "program $< verify reset exit"

# --- Clean -------------------------------------------------------------------
clean:
	rm -rf $(BUILD)
