# ARK G4 ESC - STM32G491REI6
# Reuses G431 HAL (G491 is same family, compatible headers/drivers)
MCU := ARKG4
PART := STM32G431xx

HAL_FOLDER_$(MCU) := $(HAL_FOLDER)/g431

# reuse G431 SVD and openocd config for debug
SVD_$(MCU) := $(wildcard $(HAL_FOLDER_$(MCU))/*.svd)

MCU_$(MCU) := -mcpu=cortex-m4 -mthumb
LDSCRIPT_$(MCU) := $(wildcard $(HAL_FOLDER_$(MCU))/*.ld)

SRC_BASE_DIR_$(MCU) := \
	$(HAL_FOLDER_$(MCU))/Startup \
	$(HAL_FOLDER_$(MCU))/Drivers/STM32G4xx_HAL_Driver/Src

CFLAGS_$(MCU) := \
	-I$(HAL_FOLDER_$(MCU))/Inc \
	-I$(HAL_FOLDER_$(MCU))/Drivers/STM32G4xx_HAL_Driver/Inc \
	-I$(HAL_FOLDER_$(MCU))/Drivers/CMSIS/Include \
	-I$(HAL_FOLDER_$(MCU))/Drivers/CMSIS/Device/ST/STM32G4xx/Include

CFLAGS_$(MCU) += \
	-DHSE_VALUE=8000000 \
	-D$(PART) \
	-DMCU_G431 \
	-DUSE_FULL_LL_DRIVER \
	-DPREFETCH_ENABLE=1 \
	-DRAM_LIMIT_KB=112

# RGB LED on PC6 (red), PC7 (green), PC8 (blue), active low
CFLAGS_$(MCU) += \
	-DUSE_RGB_LED \
	-DLED_RED_PIN=6 \
	-DLED_GREEN_PIN=7 \
	-DLED_BLUE_PIN=8

# 8MHz external oscillator (bypass mode)
CFLAGS_$(MCU) += -DUSE_HSE

# FDCAN1 pins: RX on PA11 AF9, TX on PB9 AF9
CFLAGS_$(MCU) += \
	-DFDCAN_RX_PORT=GPIOA -DFDCAN_RX_PIN=LL_GPIO_PIN_11 -DFDCAN_RX_AF=LL_GPIO_AF_9 \
	-DFDCAN_TX_PORT=GPIOB -DFDCAN_TX_PIN=LL_GPIO_PIN_9 -DFDCAN_TX_AF=LL_GPIO_AF_9

SRC_$(MCU)_BL := $(foreach dir,$(SRC_BASE_DIR_$(MCU)),$(wildcard $(dir)/*.[cs])) \
	$(wildcard $(HAL_FOLDER_$(MCU))/Src/*.c)

# additional CFLAGS and source for DroneCAN
CFLAGS_DRONECAN_$(MCU) += \
	-Ibootloader/DroneCAN \
	-Ibootloader/DroneCAN/libcanard \
	-Ibootloader/DroneCAN/dsdl_generated/include \
	-UAM32_MCU -DAM32_MCU=\"G431\" \
	-DUSE_DEBUG_UART

# main Makefile hardcodes CFLAGS_DRONECAN_L431 for CAN builds
CFLAGS_DRONECAN_L431 ?= $(CFLAGS_DRONECAN_$(MCU))

SRC_DIR_DRONECAN_$(MCU) += bootloader/DroneCAN \
		bootloader/DroneCAN/dsdl_generated/src \
		bootloader/DroneCAN/libcanard

SRC_DRONECAN_$(MCU) := $(foreach dir,$(SRC_DIR_DRONECAN_$(MCU)),$(wildcard $(dir)/*.[cs]))
