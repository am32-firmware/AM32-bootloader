MCU := G431
PART := STM32G431xx

HAL_FOLDER_$(MCU) := $(HAL_FOLDER)/$(call lc,$(MCU))

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
	-DUSE_FULL_LL_DRIVER \
	-DPREFETCH_ENABLE=1

SRC_$(MCU)_BL := $(foreach dir,$(SRC_BASE_DIR_$(MCU)),$(wildcard $(dir)/*.[cs])) \
	$(wildcard $(HAL_FOLDER_$(MCU))/Src/*.c)
