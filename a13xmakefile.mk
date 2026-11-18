MCU := A13X
PART := MCXA13X

MCU_LC := $(call lc,$(MCU))

# OpenOCD has no MCXA13X support, so there is no openocd.cfg to copy.
# Debug with pyOCD, J-Link or NXP LinkServer instead.
NO_OPENOCD_$(MCU) := 1

HAL_FOLDER_$(MCU) := $(HAL_FOLDER)/$(MCU_LC)

MCU_$(MCU) := -mcpu=cortex-m33+nodsp -mthumb
$(MCU)_LDSCRIPT := Mcu/a13x/MCXA_FLASH.ld
$(MCU)_LDSCRIPT_BLU := Mcu/a13x/MCXA_FLASH_blu.ld

SRC_BASE_DIR_$(MCU) := \
	$(HAL_FOLDER_$(MCU))/Startup \
	$(HAL_FOLDER_$(MCU))/Drivers/src

CFLAGS_$(MCU) := \
	-I$(HAL_FOLDER_$(MCU))/Inc \
	-I$(HAL_FOLDER_$(MCU))/Drivers/inc \
	-I$(HAL_FOLDER_$(MCU))/CMSIS

CFLAGS_$(MCU) += \
	 -D$(PART) \
	 -DCPU_MCXA133VFM \
	 -DMCU_FLASH_START=0x0 \
	 -DEEPROM_MAX_SIZE=0x180 \
	 -DFLASH_SECTOR_SIZE=0x2000 \
	 -Wno-nonnull

SRC_$(MCU)_BL := $(foreach dir,$(SRC_BASE_DIR_$(MCU)),$(wildcard $(dir)/*.[cs])) \
	$(wildcard $(HAL_FOLDER_$(MCU))/Src/*.c)

BOOTLOADER_PINS_$(MCU) = PB2
