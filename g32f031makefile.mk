MCU := G32F031
PART := G32F031xx

MCU_LC := $(call lc,$(MCU))

HAL_FOLDER_$(MCU) := $(HAL_FOLDER)/$(MCU_LC)

$(MCU)_LDSCRIPT := Mcu/g32f031/Link.ld
$(MCU)_LDSCRIPT_BLU := Mcu/g32f031/Link_BLU.ld

# FIRMWARE_RELATIVE_START must be forced: Makefile passes -DDRONECAN_SUPPORT=0,
# and main.c uses #if defined(DRONECAN_SUPPORT) which is true even when =0,
# wrongly selecting 0x4000. G32 APP starts at 0x1000 (4KB bootloader).
MCU_$(MCU) := -mcpu=cortex-m0 -mthumb -DMCU_FLASH_START=0x00000000 -DFIRMWARE_RELATIVE_START=0x1000
LDSCRIPT_$(MCU) := $(wildcard $(HAL_FOLDER_$(MCU))/*.ld)

SRC_BASE_DIR_$(MCU) := \
	$(HAL_FOLDER_$(MCU))/Startup \
	$(HAL_FOLDER_$(MCU))/Drivers/G32F031_DAL_Driver/Source

CFLAGS_$(MCU) := \
	-I$(HAL_FOLDER_$(MCU))/Inc \
	-I$(HAL_FOLDER_$(MCU))/Drivers/G32F031_DAL_Driver/Include \
	-I$(HAL_FOLDER_$(MCU))/Drivers/CMSIS/Include \
	-I$(HAL_FOLDER_$(MCU))/Drivers/CMSIS/Device/Geehy/G32F031/Include

CFLAGS_$(MCU) += \
	-D$(PART) \
	-DUSE_FULL_DDL_DRIVER

SRC_$(MCU)_BL := $(foreach dir,$(SRC_BASE_DIR_$(MCU)),$(wildcard $(dir)/*.[cs])) \
	$(wildcard $(HAL_FOLDER_$(MCU))/Src/*.c)
