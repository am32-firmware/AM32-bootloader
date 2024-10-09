MCU := F415
PART := AT32F415K8U7_4

MCU_LC := $(call lc,$(MCU))

HAL_FOLDER_$(MCU) := $(HAL_FOLDER)/$(MCU_LC)

MCU_$(MCU) := -mcpu=cortex-m4 -mthumb
LDSCRIPT_$(MCU) := $(wildcard $(HAL_FOLDER_$(MCU))/*.ld)

SRC_BASE_DIR_$(MCU) := \
	$(HAL_FOLDER_$(MCU))/Startup \
	$(HAL_FOLDER_$(MCU))/Drivers/drivers/src

CFLAGS_$(MCU) := \
	-I$(HAL_FOLDER_$(MCU))/Inc \
	-I$(HAL_FOLDER_$(MCU))/Drivers/drivers/inc \
	-I$(HAL_FOLDER_$(MCU))/Drivers/CMSIS/cm4/core_support \
	-I$(HAL_FOLDER_$(MCU))/Drivers/CMSIS/cm4/device_support

CFLAGS_$(MCU) += \
	 -D$(PART) -DARTERY \
	 -DUSE_STDPERIPH_DRIVER

SRC_$(MCU) := $(foreach dir,$(SRC_DIR_$(MCU)),$(wildcard $(dir)/*.[cs]))
SRC_$(MCU)_BL := $(foreach dir,$(SRC_BASE_DIR_$(MCU)),$(wildcard $(dir)/*.[cs])) \
	$(wildcard $(HAL_FOLDER_$(MCU))/Src/*.c)

# additional CFLAGS and source for DroneCAN
CFLAGS_DRONECAN_$(MCU) += \
	-Ibootloader/DroneCAN \
	-Ibootloader/DroneCAN/libcanard \
	-Ibootloader/DroneCAN/dsdl_generated/include

SRC_DIR_DRONECAN_$(MCU) += bootloader/DroneCAN \
		bootloader/DroneCAN/dsdl_generated/src \
		bootloader/DroneCAN/libcanard

SRC_DRONECAN_$(MCU) := $(foreach dir,$(SRC_DIR_DRONECAN_$(MCU)),$(wildcard $(dir)/*.[cs]))
