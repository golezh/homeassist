# ============================================================
# Universal AVR Makefile
# ============================================================

# ---------- Project settings ----------
TARGET      ?= keytest
MCU         ?= atmega8535
F_CPU       ?= 4000000UL

# For avrdude. For ATmega8535 use m8535.
AVRDUDE_MCU ?= m8535

# ---------- Programmer settings ----------
PROGRAMMER  ?= stk500v2
PORT        ?= avrdoper

# Examples:
# PROGRAMMER = usbasp
# PROGRAMMER = stk500v2
# PORT       = avrdoper

# ---------- Directories ----------
BUILD_DIR   ?= build

# Source files.
# By default, all .c files in current folder and src/ folder are used.
SRC         ?= $(wildcard *.c) $(wildcard src/*.c)

# Include directories.
INCLUDE_DIRS ?= . include

# ---------- Tools ----------
AVR_PREFIX  ?= avr-

CC          := $(AVR_PREFIX)gcc
OBJCOPY     := $(AVR_PREFIX)objcopy
OBJDUMP     := $(AVR_PREFIX)objdump
SIZE        := $(AVR_PREFIX)size
AVRDUDE     := avrdude

# If you need absolute Windows paths, use for example:
# CC      := D:/program/gcc/bin/avr-gcc.exe
# OBJCOPY := D:/program/gcc/bin/avr-objcopy.exe
# SIZE    := D:/program/gcc/bin/avr-size.exe
# AVRDUDE := D:/program/avrdude/avrdude.exe

# ---------- Flags ----------
MCU_FLAGS   := -mmcu=$(MCU)

INCLUDES    := $(addprefix -I,$(INCLUDE_DIRS))

CPPFLAGS    := -DF_CPU=$(F_CPU) $(INCLUDES)

CFLAGS      ?= -Os -Wall -Wextra -std=gnu99
CFLAGS      += $(MCU_FLAGS) $(CPPFLAGS)
CFLAGS      += -ffunction-sections -fdata-sections
CFLAGS      += -MMD -MP

LDFLAGS     ?= $(MCU_FLAGS)
LDFLAGS     += -Wl,--gc-sections
LDFLAGS     += -Wl,-Map=$(BUILD_DIR)/$(TARGET).map

LDLIBS      ?=

# ---------- Generated files ----------
OBJ         := $(addprefix $(BUILD_DIR)/,$(SRC:.c=.o))
DEP         := $(OBJ:.o=.d)

ELF         := $(BUILD_DIR)/$(TARGET).elf
HEX         := $(BUILD_DIR)/$(TARGET).hex
EEP         := $(BUILD_DIR)/$(TARGET).eep
LSS         := $(BUILD_DIR)/$(TARGET).lss

# ---------- Main targets ----------
.PHONY: all clean flash size disasm help

all: $(HEX) size

# ---------- Compile C files ----------
$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

# ---------- Link ELF ----------
$(ELF): $(OBJ)
	@mkdir -p $(dir $@)
	$(CC) $^ $(LDFLAGS) $(LDLIBS) -o $@

# ---------- Create HEX ----------
$(HEX): $(ELF)
	$(OBJCOPY) -O ihex -R .eeprom $< $@

# ---------- Create EEPROM HEX ----------
$(EEP): $(ELF)
	$(OBJCOPY) -O ihex -j .eeprom \
		--set-section-flags=.eeprom=alloc,load \
		--change-section-lma .eeprom=0 \
		$< $@

# ---------- Disassembly ----------
disasm: $(ELF)
	$(OBJDUMP) -h -S $< > $(LSS)

# ---------- Size report ----------
size: $(ELF)
	$(SIZE) -B -d $<

# ---------- Flash ----------
flash: $(HEX)
	$(AVRDUDE) -c $(PROGRAMMER) $(if $(PORT),-P $(PORT),) -p $(AVRDUDE_MCU) -U flash:w:$<:i

# ---------- Clean ----------
clean:
	rm -rf $(BUILD_DIR)

# ---------- Help ----------
help:
	@echo "Universal AVR Makefile"
	@echo ""
	@echo "Common commands:"
	@echo "  make"
	@echo "  make clean"
	@echo "  make flash"
	@echo "  make size"
	@echo "  make disasm"
	@echo ""
	@echo "Configuration examples:"
	@echo "  make TARGET=keytest MCU=atmega8535 AVRDUDE_MCU=m8535"
	@echo "  make TARGET=test MCU=atmega8 AVRDUDE_MCU=m8 F_CPU=8000000UL"
	@echo "  make flash PROGRAMMER=usbasp"
	@echo "  make flash PROGRAMMER=stk500v2 PORT=avrdoper"

# Include automatic dependency files
-include $(DEP)