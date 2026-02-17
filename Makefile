# ============================================================================
# STM32F4xx Bare-Metal Driver Suite — Makefile
# ============================================================================
# Toolchain: ARM GCC (arm-none-eabi-gcc)
# Target:    STM32F401RE (Cortex-M4F, 512KB Flash, 96KB SRAM)
#
# Usage:
#   make            — Build all (drivers + protocol stacks)
#   make clean      — Remove build artifacts
#   make flash      — Flash using OpenOCD (adjust for your debugger)
#   make size       — Show section sizes
# ============================================================================

# ── Toolchain ───────────────────────────────────────────────────────────────
PREFIX = arm-none-eabi-
CC     = $(PREFIX)gcc
AS     = $(PREFIX)gcc -x assembler-with-cpp
LD     = $(PREFIX)gcc
OBJCOPY = $(PREFIX)objcopy
SIZE   = $(PREFIX)size

# ── Project ─────────────────────────────────────────────────────────────────
TARGET  = firmware
BUILD   = build

# ── MCU Flags ───────────────────────────────────────────────────────────────
CPU     = -mcpu=cortex-m4
FPU     = -mfpu=fpv4-sp-d16 -mfloat-abi=hard
MCU     = $(CPU) -mthumb $(FPU)

# ── Source Files ────────────────────────────────────────────────────────────
# Drivers
C_SOURCES += drivers/gpio/gpio_driver.c
C_SOURCES += drivers/uart/uart_driver.c
C_SOURCES += drivers/spi/spi_driver.c
C_SOURCES += drivers/i2c/i2c_driver.c
C_SOURCES += drivers/timer/timer_driver.c
C_SOURCES += drivers/adc/adc_driver.c
C_SOURCES += drivers/pwm/pwm_driver.c
C_SOURCES += drivers/dma/dma_driver.c
C_SOURCES += drivers/nvic/nvic_driver.c

# Protocol Stacks
C_SOURCES += protocol_stacks/modbus/modbus_rtu.c
C_SOURCES += protocol_stacks/can_bus/can_driver.c
C_SOURCES += protocol_stacks/usb_cdc/usb_cdc.c

# Startup (assembly)
ASM_SOURCES = startup_stm32f401xe.s

# ── Include Paths ───────────────────────────────────────────────────────────
C_INCLUDES  = -Idrivers/common
C_INCLUDES += -Idrivers/gpio
C_INCLUDES += -Idrivers/uart
C_INCLUDES += -Idrivers/spi
C_INCLUDES += -Idrivers/i2c
C_INCLUDES += -Idrivers/timer
C_INCLUDES += -Idrivers/adc
C_INCLUDES += -Idrivers/pwm
C_INCLUDES += -Idrivers/dma
C_INCLUDES += -Idrivers/nvic
C_INCLUDES += -Iprotocol_stacks/modbus
C_INCLUDES += -Iprotocol_stacks/can_bus
C_INCLUDES += -Iprotocol_stacks/usb_cdc

# ── Compiler Flags ──────────────────────────────────────────────────────────
CFLAGS  = $(MCU) $(C_INCLUDES)
CFLAGS += -std=gnu11
CFLAGS += -Wall -Wextra -Wshadow -Wdouble-promotion
CFLAGS += -Wformat=2 -Wformat-truncation
CFLAGS += -fdata-sections -ffunction-sections
CFLAGS += -fno-common
CFLAGS += -DSTM32F401xE

# Debug / Release
ifdef DEBUG
CFLAGS += -g3 -gdwarf-2 -Og -DDEBUG
else
CFLAGS += -Os -DNDEBUG
endif

# ── Linker Flags ────────────────────────────────────────────────────────────
LDSCRIPT = STM32F401RETx_FLASH.ld
LDFLAGS  = $(MCU)
LDFLAGS += -T$(LDSCRIPT)
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -Wl,-Map=$(BUILD)/$(TARGET).map,--cref
LDFLAGS += --specs=nano.specs
LDFLAGS += --specs=nosys.specs
LDFLAGS += -lm

# ── Build Rules ─────────────────────────────────────────────────────────────
OBJECTS  = $(addprefix $(BUILD)/,$(C_SOURCES:.c=.o))
OBJECTS += $(addprefix $(BUILD)/,$(ASM_SOURCES:.s=.o))
DEPS     = $(OBJECTS:.o=.d)

all: $(BUILD)/$(TARGET).elf $(BUILD)/$(TARGET).hex $(BUILD)/$(TARGET).bin size

$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@

$(BUILD)/%.o: %.s
	@mkdir -p $(dir $@)
	$(AS) $(MCU) -c $< -o $@

$(BUILD)/$(TARGET).elf: $(OBJECTS)
	$(LD) $(LDFLAGS) $(OBJECTS) -o $@

$(BUILD)/$(TARGET).hex: $(BUILD)/$(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

$(BUILD)/$(TARGET).bin: $(BUILD)/$(TARGET).elf
	$(OBJCOPY) -O binary -S $< $@

size: $(BUILD)/$(TARGET).elf
	$(SIZE) $<

clean:
	rm -rf $(BUILD)

flash: $(BUILD)/$(TARGET).bin
	openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
	        -c "program $(BUILD)/$(TARGET).bin verify reset exit 0x08000000"

# ── Dependencies ────────────────────────────────────────────────────────────
-include $(DEPS)

.PHONY: all clean flash size
