CC	:= avr-gcc
LD	:= avr-ld
OBJCOPY	:= avr-objcopy
OBJDUMP	:= avr-objdump
SIZE	:= avr-size

TARGET = funkboot
SOURCE = $(wildcard *.c)
BUILD_DIR = build

CONFIG = funkstuff168

AVRDUDE_PROG := -c avr910 -b 115200 -P /dev/ispprog
#AVRDUDE_PROG := -c dragon_isp -P usb

# ---------------------------------------------------------------------------

ifeq ($(CONFIG), funkstuff168)
# (ext. crystal 16MHz, 2.7V BOD)
MCU = atmega168
AVRDUDE_MCU=m168 -F

AVRDUDE_FUSES=lfuse:w:0xff:m hfuse:w:0xdd:m efuse:w:0x00:m
BOOTLOADER_START=0x3800
endif

# ---------------------------------------------------------------------------

CFLAGS = -pipe -g -Os -mmcu=$(MCU) -Wall -fdata-sections -ffunction-sections
CFLAGS += -Wa,-adhlns=$(BUILD_DIR)/$(*D)/$(*F).lst -MMD -MP -MF $(BUILD_DIR)/$(*D)/$(*F).d
CFLAGS += -DBOOTLOADER_START=$(BOOTLOADER_START) -DCONFIG_$(CONFIG)=1
LDFLAGS = -Wl,-Map,$(@:.elf=.map),--cref,--relax,--gc-sections,--section-start=.text=$(BOOTLOADER_START)

# ---------------------------------------------------------------------------

$(TARGET): $(BUILD_DIR)/$(TARGET).elf
	@$(SIZE) -B -x --mcu=$(MCU) $<

$(BUILD_DIR)/$(TARGET).elf: $(patsubst %,$(BUILD_DIR)/%,$(SOURCE:.c=.o))
	@echo " Linking file:  $@"
	@$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^
	@$(OBJDUMP) -h -S $@ > $(@:.elf=.lss)
	@$(OBJCOPY) -j .text -j .data -O ihex $@ $(@:.elf=.hex)
	@$(OBJCOPY) -j .text -j .data -O binary $@ $(@:.elf=.bin)

$(BUILD_DIR)/%.o: %.c $(MAKEFILE_LIST)
	@echo " Building file: $<"
	@$(shell test -d $(BUILD_DIR)/$(*D) || mkdir -p $(BUILD_DIR)/$(*D))
	@$(CC) $(CFLAGS) -o $@ -c $<

include $(shell find $(BUILD_DIR) -name \*.d 2> /dev/null)

clean:
	rm -rf $(BUILD_DIR)

install: $(BUILD_DIR)/$(TARGET).elf
	avrdude $(AVRDUDE_PROG) -p $(AVRDUDE_MCU) -U flash:w:$(<:.elf=.hex)

fuses:
	avrdude $(AVRDUDE_PROG) -p $(AVRDUDE_MCU) $(patsubst %,-U %, $(AVRDUDE_FUSES))
