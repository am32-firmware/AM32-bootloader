MCU := SITL

HAL_FOLDER_$(MCU) := $(HAL_FOLDER)/$(call lc,$(MCU))

# native build using the host compiler; ELF only, no hex/bin/updater
NATIVE_$(MCU) := 1
SITL_CC := gcc

MCU_$(MCU) :=
LDSCRIPT_$(MCU) :=

# only one (dummy) pin variant for the SITL
BOOTLOADER_PINS_$(MCU) := PB4

# host compiler flags replacing the ARM CFLAGS_BASE. MCU_FLASH_START as
# a UL literal keeps the absolute-address pointer casts in the
# unmodified bootloader sources valid on a 64 bit host. No -D_GNU_SOURCE
# here: DroneCAN.c defines its own static memmem() which would clash
# with the glibc declaration
CFLAGS_BASE_$(MCU) := \
	-I$(MAIN_INC_DIR) -g3 -O2 -funsigned-char -fno-strict-aliasing \
	-Wall -Wextra -Wundef -Werror -Wno-unused-parameter \
	-include $(MAIN_INC_DIR)/targets.h \
	-DMCU_FLASH_START=0x08000000UL \
	-DCANARD_64_BIT="(__SIZEOF_POINTER__ == 8)"

CFLAGS_$(MCU) := \
	-I$(HAL_FOLDER_$(MCU))/Inc \
	-DMCU_$(MCU)

# optional AddressSanitizer for the native bootloader build, matching
# the app's SITL_SANITIZE. -no-pie keeps the fixed flash mapping, and
# ASan tolerates it here since the bootloader's mapped regions are well
# clear of the shadow. Off by default.
ifneq ($(SITL_SANITIZE),)
SITL_SAN_FLAGS := -fsanitize=$(SITL_SANITIZE) -fno-omit-frame-pointer
CFLAGS_BASE_$(MCU) += $(SITL_SAN_FLAGS)
endif

# -no-pie so the fixed flash mapping at 0x08000000 and the devinfo
# address fit in the protocol's 32 bit addresses
LDFLAGS_COMMON_$(MCU) := -no-pie $(SITL_SAN_FLAGS)

SRC_$(MCU)_BL := $(wildcard $(HAL_FOLDER_$(MCU))/Src/*.c)

# additional CFLAGS and source for DroneCAN
CFLAGS_DRONECAN_$(MCU) += \
	-Ibootloader/DroneCAN \
	-Ibootloader/DroneCAN/libcanard \
	-Ibootloader/DroneCAN/dsdl_generated/include

SRC_DIR_DRONECAN_$(MCU) += bootloader/DroneCAN \
		bootloader/DroneCAN/dsdl_generated/src \
		bootloader/DroneCAN/libcanard

SRC_DRONECAN_$(MCU) := $(foreach dir,$(SRC_DIR_DRONECAN_$(MCU)),$(wildcard $(dir)/*.[cs]))
