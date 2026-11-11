TARGET  := lora_spectrum_sweeper
BUILD   := build
CC      := arm-none-eabi-gcc
OBJCOPY := arm-none-eabi-objcopy
SIZE    := arm-none-eabi-size

CFLAGS  := -mcpu=cortex-m0 -mthumb -Os -ffreestanding -fdata-sections -ffunction-sections \
           -Wall -Wextra -Werror -std=c11
LDFLAGS := -mcpu=cortex-m0 -mthumb -nostdlib -T stm32f072xb.ld -Wl,--gc-sections

all: $(BUILD)/$(TARGET).elf $(BUILD)/$(TARGET).bin
	$(SIZE) $(BUILD)/$(TARGET).elf

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/startup.o: startup_stm32f072xb.s | $(BUILD)
	$(CC) -mcpu=cortex-m0 -mthumb -c $< -o $@

$(BUILD)/main.o: main.c | $(BUILD)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/$(TARGET).elf: $(BUILD)/startup.o $(BUILD)/main.o stm32f072xb.ld
	$(CC) $(LDFLAGS) $(BUILD)/startup.o $(BUILD)/main.o -lgcc -o $@

$(BUILD)/$(TARGET).bin: $(BUILD)/$(TARGET).elf
	$(OBJCOPY) -O binary $< $@

clean:
	rm -rf $(BUILD)

.PHONY: all clean
