TARGET   := walking_leg_ctrl
BUILD    := build

CC       := arm-none-eabi-gcc
OBJCOPY  := arm-none-eabi-objcopy
SIZE     := arm-none-eabi-size

CFLAGS   := -mcpu=cortex-m3 -mthumb -Os -Wall -Wextra -ffunction-sections -fdata-sections -std=c11
LDFLAGS  := -mcpu=cortex-m3 -mthumb -T stm32f103xb.ld -nostartfiles -Wl,--gc-sections -Wl,-Map=$(BUILD)/$(TARGET).map

OBJS     := $(BUILD)/startup_stm32f103xb.o $(BUILD)/main.o

.PHONY: all clean

all: $(BUILD)/$(TARGET).bin $(BUILD)/$(TARGET).hex

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/startup_stm32f103xb.o: startup_stm32f103xb.s | $(BUILD)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/main.o: main.c | $(BUILD)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/$(TARGET).elf: $(OBJS) stm32f103xb.ld
	$(CC) $(OBJS) $(LDFLAGS) -o $@
	$(SIZE) $@

$(BUILD)/$(TARGET).bin: $(BUILD)/$(TARGET).elf
	$(OBJCOPY) -O binary $< $@

$(BUILD)/$(TARGET).hex: $(BUILD)/$(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

clean:
	rm -rf $(BUILD)
