# Makefile for lifeguard_mipt (PlatformIO ESP32 project)

# Project directories
SRC_DIR = src
INC_DIR = inc include
LIB_DIR = lib

# PlatformIO environment (matches platformio.ini)
ENV = esp32doit-devkit-v1

# Default: build the project
all:
	platformio run

# Upload / flash to ESP32
flash:
	platformio run -t upload -e $(ENV)

# Build and then flash
build_flash:
	platformio run && platformio run -t upload -e $(ENV)

# Monitor serial output
monitor:
	platformio device monitor -e $(ENV)

# Clean project
clean:
	platformio run -t clean

# Compile only (without linking/upload)
compile:
	platformio run --target compile -e $(ENV)

.PHONY: all flash build_flash monitor clean compile
