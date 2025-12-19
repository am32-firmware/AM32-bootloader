MCU := A153
PART := MCXA153

MCU_LC := $(call lc,$(MCU))

HAL_FOLDER_$(MCU) := $(HAL_FOLDER)/$(MCU_LC)

MCU_$(MCU) := -mcpu=cortex-m33+nodsp -mthumb
$(MCU)_LDSCRIPT := Mcu/a153/MCXA_FLASH.ld
$(MCU)_LDSCRIPT_BLU := Mcu/a153/MCXA_FLASH_blu.ld

SRC_BASE_DIR_$(MCU) := \
	$(HAL_FOLDER_$(MCU))/Startup \
	$(HAL_FOLDER_$(MCU))/Drivers/src

CFLAGS_$(MCU) := \
	-I$(HAL_FOLDER_$(MCU))/Inc \
	-I$(HAL_FOLDER_$(MCU))/Drivers/inc \
	-I$(HAL_FOLDER_$(MCU))/CMSIS

CFLAGS_$(MCU) += \
	 -D$(PART) \
	 -DCPU_MCXA153VLH \
	 -DMCU_FLASH_START=0x0 \
	 -DEEPROM_MAX_SIZE=0x180 \
	 -DFLASH_SECTOR_SIZE=0x2000 \
	 -Wno-nonnull

SRC_$(MCU)_BL := $(foreach dir,$(SRC_BASE_DIR_$(MCU)),$(wildcard $(dir)/*.[cs])) \
	$(wildcard $(HAL_FOLDER_$(MCU))/Src/*.c)
