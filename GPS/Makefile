##############################################################################
# Makefile dla STM32L476RG – GPS NEO-6M
#
# Użycie:
#   make              – kompiluj firmware
#   make flash        – wgraj przez ST-Link (st-flash)
#   make flash-openocd – wgraj przez OpenOCD
#   make clean        – usuń pliki wynikowe
#   make size         – pokaż rozmiar firmware
#
# Wymagania:
#   arm-none-eabi-gcc
#   st-flash LUB openocd  (do make flash)
#
# Pliki do skopiowania z LAB01_full_app/ przed pierwszym make:
#   cp -r ../LAB01_full_app/Drivers            .
#   cp -r ../LAB01_full_app/Core/Startup       Core/
#   cp    ../LAB01_full_app/STM32L476RGTX_FLASH.ld .
#   cp    ../LAB01_full_app/Core/Src/syscalls.c        Core/Src/
#   cp    ../LAB01_full_app/Core/Src/sysmem.c          Core/Src/
#   cp    ../LAB01_full_app/Core/Src/system_stm32l4xx.c Core/Src/
#   cp    ../LAB01_full_app/Core/Inc/stm32l4xx_hal_conf.h Core/Inc/
##############################################################################

PREFIX  = arm-none-eabi-
CC      = $(PREFIX)gcc
AS      = $(PREFIX)gcc -x assembler-with-cpp
OBJCOPY = $(PREFIX)objcopy
OBJDUMP = $(PREFIX)objdump
SIZE    = $(PREFIX)size

TARGET  = GPS_NEO6M
BUILD   = build

CPU    = -mcpu=cortex-m4
FPU    = -mfpu=fpv4-sp-d16
FLOAT  = -mfloat-abi=hard
THUMB  = -mthumb
MCU    = $(CPU) $(FPU) $(FLOAT) $(THUMB)

LDSCRIPT = STM32L476RGTX_FLASH.ld

DEFS = \
  -DUSE_HAL_DRIVER \
  -DSTM32L476xx

INCS = \
  -ICore/Inc \
  -IDrivers/STM32L4xx_HAL_Driver/Inc \
  -IDrivers/STM32L4xx_HAL_Driver/Inc/Legacy \
  -IDrivers/CMSIS/Device/ST/STM32L4xx/Include \
  -IDrivers/CMSIS/Include

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
  -Wl,--start-group -lc -lm -Wl,--end-group

# ---------------------------------------------------------------------------
# Pliki źródłowe
# ---------------------------------------------------------------------------
C_SRCS = \
  Core/Src/main.c \
  Core/Src/gps.c \
  Core/Src/syscalls.c \
  Core/Src/sysmem.c \
  Core/Src/stm32l4xx_it.c \
  Core/Src/system_stm32l4xx.c \
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
  Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr.c \
  Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr_ex.c \
  Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc.c \
  Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc_ex.c \
  Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart.c \
  Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart_ex.c

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
