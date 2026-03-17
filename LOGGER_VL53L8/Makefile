BUILD_DIR := Debug
ELF := $(BUILD_DIR)/STM32L476_VL53L8CX.elf
BIN := $(BUILD_DIR)/STM32L476_VL53L8CX.bin
FLASH_ADDR := 0x08000000

.PHONY: all flash clean

all:
	$(MAKE) -C $(BUILD_DIR) all
	arm-none-eabi-objcopy -O binary $(ELF) $(BIN)

flash: all
	st-flash --reset write $(BIN) $(FLASH_ADDR)

clean:
	$(MAKE) -C $(BUILD_DIR) clean
	rm -f $(BIN)
