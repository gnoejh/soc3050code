# Makefile for Timer Library Test
# Comprehensive test of _timer.h library

# Project name
TARGET = main

# Source files
SOURCES = main.c

# MCU configuration
MCU = atmega128
F_CPU = 16000000UL

# Programmer configuration
PROGRAMMER = arduino
PORT = COM3

# Paths
WORKSPACE_ROOT = ../..
SHARED_LIBS = $(WORKSPACE_ROOT)/shared_libs
TOOLCHAIN = $(WORKSPACE_ROOT)/tools/avr-toolchain
SIMULIDE = $(WORKSPACE_ROOT)/tools/simulide

# Include directories
INCLUDES = -I$(SHARED_LIBS)

# Compiler flags
CFLAGS = -mmcu=$(MCU) -DF_CPU=$(F_CPU) -Os -Wall -Wextra
CFLAGS += $(INCLUDES)
CFLAGS += -std=gnu99

# Linker flags
LDFLAGS = -mmcu=$(MCU)

# Tools
CC = $(TOOLCHAIN)/bin/avr-gcc.exe
OBJCOPY = $(TOOLCHAIN)/bin/avr-objcopy.exe
SIZE = $(TOOLCHAIN)/bin/avr-size.exe
AVRDUDE = $(TOOLCHAIN)/bin/avrdude.exe
AVRDUDE_CONFIG = $(TOOLCHAIN)/etc/avrdude.conf

# Object files
OBJECTS = $(SOURCES:.c=.o)

# Shared library dependencies
SHARED_OBJECTS = \
	$(SHARED_LIBS)/_timer.o \
	$(SHARED_LIBS)/uart_enhanced.o

# Default target
all: $(TARGET).hex size

# Compile source files
%.o: %.c
	@echo Compiling $<...
	$(CC) $(CFLAGS) -c $< -o $@

# Compile shared libraries
$(SHARED_LIBS)/%.o: $(SHARED_LIBS)/%.c
	@echo Compiling shared library $<...
	$(CC) $(CFLAGS) -c $< -o $@

# Link
$(TARGET).elf: $(OBJECTS) $(SHARED_OBJECTS)
	@echo Linking...
	$(CC) $(LDFLAGS) $^ -o $@

# Create hex file
$(TARGET).hex: $(TARGET).elf
	@echo Creating hex file...
	$(OBJCOPY) -O ihex -R .eeprom $< $@

# Show size
size: $(TARGET).elf
	@echo
	@echo Size information:
	$(SIZE) --format=avr --mcu=$(MCU) $<

# Program hardware
program: $(TARGET).hex
	$(AVRDUDE) -C $(AVRDUDE_CONFIG) -c $(PROGRAMMER) -p $(MCU) -P $(PORT) -U flash:w:$<:i

# Clean
clean:
	rm -f $(OBJECTS) $(SHARED_OBJECTS)
	rm -f $(TARGET).elf $(TARGET).hex $(TARGET).map

.PHONY: all size program clean
