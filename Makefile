# makefile for AM32 bootloader
QUIET = @

# tools
CC = $(ARM_SDK_PREFIX)gcc
OBJCOPY = $(ARM_SDK_PREFIX)objcopy
ECHO = echo

# common variables
IDENTIFIER := AM32

# Folders
HAL_FOLDER := Mcu
MAIN_INC_DIR := Inc

# Working directories
ROOT := $(patsubst %/,%,$(dir $(lastword $(MAKEFILE_LIST))))

# include the rules for OS independence
include $(ROOT)/make/tools.mk

# MCU builds, if with _xxK then adds build with given flash size
MCU_BUILDS := E230 F031 F051 F415 F415_128K F421 G071 G071_64K L431 L431_128K G431 V203 L431_CAN F415_CAN

# we support bootloader comms on a list of possible pins
BOOTLOADER_PINS = PB4 PA2 PA6 PA15 PA0

# filter out any unsupported MCUs for this host OS
filter_mcus = $(foreach w,$(MCU_NOBUILD),$(eval MCU_BUILDS := $(filter-out $w,$(MCU_BUILDS))))$(MCU_BUILDS)
MCU_BUILDS := $(call filter_mcus)

# extract the MCU from a build type of form MCU_nK
define base_mcu
$(word 1,$(subst _, ,$1))
endef

MCU_TYPES := $(sort $(foreach mcu,$(MCU_BUILDS),$(call base_mcu,$(mcu))))

# Function to include makefile for each MCU type
define INCLUDE_MCU_MAKEFILES
$(foreach MCU_TYPE,$(MCU_TYPES),$(eval include $(call lc,$(MCU_TYPE))makefile.mk))
endef
$(call INCLUDE_MCU_MAKEFILES)

# additional libs
LIBS := -lnosys

# Compiler options
CFLAGS_BASE := -fsingle-precision-constant -fomit-frame-pointer -ffast-math --specs=nosys.specs
CFLAGS_BASE += -I$(MAIN_INC_DIR) -g3 -Os -ffunction-sections -funsigned-char
CFLAGS_BASE += -Wall -Wextra -Wundef -Werror -Wno-unused-parameter

CFLAGS_COMMON := $(CFLAGS_BASE)

# Linker options
LDFLAGS_COMMON := -specs=nano.specs $(LIBS) -Wl,--gc-sections -Wl,--print-memory-usage

# configure some directories that are relative to wherever ROOT_DIR is located
OBJ := obj
BIN_DIR := $(ROOT)/$(OBJ)

# find the SVD files
$(foreach MCU,$(MCU_TYPES),$(eval SVD_$(MCU) := $(wildcard $(HAL_FOLDER_$(MCU))/*.svd)))

.PHONY : clean all
all : check_tools bootloaders

# Check if tools are installed
check_tools:
ifeq ($(OS),Windows_NT)
	@if not exist "$(CC).exe" ( \
		echo Error: please install tools first with target arm_sdk_install. & exit /B 1 \
	)
else
	@$(SHELL) -c 'command -v $(CC) >/dev/null 2>&1 || { echo "Error: please install tools first with target arm_sdk_install."; exit 1; }'
endif

clean :
	@echo Removing $(OBJ) directory
	@$(RM) -rf $(OBJ)

# get bootloader version
BOOTLOADER_VERSION := $(shell $(FGREP) "define BOOTLOADER_VERSION" $(MAIN_INC_DIR)/version.h | $(CUT) -d" " -f3 )

SRC_BL := $(foreach dir,bootloader,$(wildcard $(dir)/*.[cs]))
SRC_BLU := $(foreach dir,bl_update,$(wildcard $(dir)/*.[cs]))

LDSCRIPT_BL := bootloader/ldscript_bl.ld
LDSCRIPT_BL_CAN := bootloader/ldscript_bl_CAN.ld
LDSCRIPT_BLU := bl_update/ldscript_bl.ld

# Function to extract CFLAGS based on target name
get_flash_size = $(if $(filter %K,$(word 2,$(subst _, ,$1))),-DBOARD_FLASH_SIZE=$(subst K,,$(word 2,$(subst _, ,$1))))

# Function to check for _CAN suffix
has_can_suffix = $(findstring _CAN,$1)

# Function to return CFLAGS
define get_cflags
  $(call get_flash_size,$1) \
  $(if $(call has_can_suffix,$1),-DDRONECAN_SUPPORT=1 -DBOARD_FLASH_SIZE=128,-DDRONECAN_SUPPORT=0)
endef

# get a tag in the form _nK if the build has a _nK suffix
define get_k_tag
$(if $(findstring _,$1),_$(patsubst %_,%,$(word 2,$(subst _, ,$1))))
endef

# bootloader target names for example "make AM32_F421_BOOTLOADER" with optional _nK suffix
define BOOTLOADER_BASENAME
$(IDENTIFIER)_$(call base_mcu,$(1))_BOOTLOADER_$(2)$(call get_k_tag,$(1))
endef

define BOOTLOADER_UPDATE_BASENAME
$(IDENTIFIER)_$(call base_mcu,$(1))_BL_UPDATER_$(2)$(call get_k_tag,$(1))
endef

# bootloader target names with version for filename
define BOOTLOADER_BASENAME_VER
$(call BOOTLOADER_BASENAME,$(1),$(2))_V$(BOOTLOADER_VERSION)
endef

# bootloader target names with version for filename
define BOOTLOADER_UPDATE_BASENAME_VER
$(call BOOTLOADER_UPDATE_BASENAME,$(1),$(2))_V$(BOOTLOADER_VERSION)
endef

# list of targets formed using CREATE_BOOTLOADER_TARGET
ALL_BUILDS :=
BLU_BUILDS :=

# create a bootloader build target given a build in the form MCU or MCU_nK and a PIN
define CREATE_BOOTLOADER_TARGET
$(eval BUILD := $(1))
$(eval PIN := $(2))
$(eval MCU := $$(call base_mcu,$$(1)))
$(eval EXTRA_CFLAGS := $(call get_cflags,$(1)))
$(eval ELF_FILE := $(BIN_DIR)/$(call BOOTLOADER_BASENAME_VER,$(BUILD),$(PIN)).elf)
$(eval HEX_FILE := $(ELF_FILE:.elf=.hex))
$(eval DEP_FILE := $(ELF_FILE:.elf=.d))
$(eval BIN_FILE := $(ELF_FILE:.elf=.bin))
$(eval H_FILE := $(ELF_FILE:.elf=.h))
$(eval BLU_ELF_FILE := $(BIN_DIR)/$(call BOOTLOADER_UPDATE_BASENAME_VER,$(BUILD),$(PIN)).elf)
$(eval BLU_HEX_FILE := $(BLU_ELF_FILE:.elf=.hex))
$(eval BLU_DEP_FILE := $(BLU_ELF_FILE:.elf=.d))
$(eval BLU_BIN_FILE := $(BLU_ELF_FILE:.elf=.bin))
$(eval BLU_AMJ_FILE := $(BLU_ELF_FILE:.elf=.amj))
$(eval TARGET := $(call BOOTLOADER_BASENAME,$(BUILD),$(PIN)))
$(eval BLU_TARGET := $(call BOOTLOADER_UPDATE_BASENAME,$(BUILD),$(PIN)))

# get MCU specific compiler, objcopy and link script or use the ARM SDK one
$(eval xCC := $(if $($(MCU)_CC), $($(MCU)_CC), $(CC)))
$(eval xOBJCOPY := $(if $($(MCU)_OBJCOPY), $($(MCU)_OBJCOPY), $(OBJCOPY)))
$(eval xLDSCRIPT := $(if $($(MCU)_LDSCRIPT), $($(MCU)_LDSCRIPT), $$(if $$(call has_can_suffix,$$(BUILD)),$(LDSCRIPT_BL_CAN),$(LDSCRIPT_BL))))
$(eval xBLU_LDSCRIPT := $(if $($(MCU)_LDSCRIPT_BLU), $($(MCU)_LDSCRIPT_BLU), $(LDSCRIPT_BLU)))
$(eval CFLAGS_DRONECAN := $$(if $$(call has_can_suffix,$$(1)),$$(CFLAGS_DRONECAN_L431)))
$(eval SRC_DRONECAN := $(if $(call has_can_suffix,$(1)),$(SRC_DRONECAN_$(MCU))))

-include $(DEP_FILE)
-include $(BLU_DEP_FILE)

$(ELF_FILE): CFLAGS_BL := $$(MCU_$(MCU)) $$(CFLAGS_$(MCU)) $$(CFLAGS_BASE) -DBOOTLOADER -DUSE_$(PIN) $(EXTRA_CFLAGS) -DAM32_MCU=\"$(MCU)\" $$(CFLAGS_DRONECAN)
$(ELF_FILE): LDFLAGS_BL := $$(LDFLAGS_COMMON) $$(LDFLAGS_$(MCU)) -T$(xLDSCRIPT)
$(ELF_FILE): $$(SRC_$(MCU)_BL) $$(SRC_BL) $$(SRC_DRONECAN)
	$$(QUIET)echo building bootloader for $(BUILD) with pin $(PIN)
	$$(QUIET)$$(MKDIR) -p $(OBJ)
	$$(QUIET)echo Compiling $(notdir $$@)
	$$(QUIET)$(xCC) $$(CFLAGS_BL) $(CFLAGS_DRONECAN) $$(LDFLAGS_BL) -MMD -MP -MF $(DEP_FILE) -o $$(@) $$(SRC_$(MCU)_BL) $$(SRC_BL) $(SRC_DRONECAN)
	$$(QUIET)$$(CP) -f $$@ $$(OBJ)$$(DSEP)debug.elf
	$$(QUIET)$$(CP) -f $$(SVD_$(MCU)) $$(OBJ)/debug.svd
	$$(QUIET)$$(CP) -f Mcu$(DSEP)$(call lc,$(MCU))$(DSEP)openocd.cfg $$(OBJ)$$(DSEP)openocd.cfg > $$(NUL)

$(H_FILE): $(BIN_FILE)
	$$(QUIET)python3 bl_update/make_binheader.py $(BIN_FILE) $(H_FILE)

$(BLU_ELF_FILE): CFLAGS_BLU := $$(MCU_$(MCU)) $$(CFLAGS_$(MCU)) $$(CFLAGS_BASE) -DBOOTLOADER -DUSE_$(PIN) $(EXTRA_CFLAGS) -Wno-unused-variable -Wno-unused-function
$(BLU_ELF_FILE): LDFLAGS_BLU := $$(LDFLAGS_COMMON) $$(LDFLAGS_$(MCU)) -T$(xBLU_LDSCRIPT)
$(BLU_ELF_FILE): $$(SRC_$(MCU)_BL) $$(SRC_BLU) $(H_FILE)
	$$(QUIET)echo building bootloader updater for $(BUILD) with pin $(PIN)
	$$(QUIET)$$(MKDIR) -p $(OBJ)
	$$(QUIET)echo Compiling $(notdir $$@)
	$$(QUIET)$(xCC) $$(CFLAGS_BLU) $$(LDFLAGS_BLU) -MMD -MP -MF $(DEP_FILE) -o $$(@) $$(SRC_$(MCU)_BL) $$(SRC_BLU) -I. -DBL_HEADER_FILE=$(H_FILE)

# Generate bin and hex files
$(HEX_FILE): $$(ELF_FILE)
	$$(QUIET)echo Generating $(notdir $$@)
	$$(QUIET)$(xOBJCOPY) -O binary $$(<) $$(@:.hex=.bin)
	$$(QUIET)$(xOBJCOPY) $$(<) -O ihex $$(@:.bin=.hex)

$(BIN_FILE): $$(HEX_FILE)

# Generate bin and hex files for bl updater
$(BLU_HEX_FILE): $$(BLU_ELF_FILE)
	$$(QUIET)echo Generating $(notdir $$@)
	$$(QUIET)$(xOBJCOPY) -O binary $$(<) $$(@:.hex=.bin)
	$$(QUIET)$(xOBJCOPY) $$(<) -O ihex $$(@:.bin=.hex)

$(BLU_AMJ_FILE): $$(BLU_HEX_FILE)
	$$(QUIET)echo Generating $(notdir $$@)
	$$(QUIET)python3 bl_update/make_amj.py --type bl_update --githash $(shell git rev-parse HEAD) $(BLU_HEX_FILE) $(BLU_AMJ_FILE)

$(TARGET): $$(HEX_FILE)

$(BLU_TARGET): $$(BLU_AMJ_FILE)

# add to list
ALL_BUILDS := $(ALL_BUILDS) $(TARGET)
BLU_BUILDS := $(BLU_BUILDS) $(BLU_TARGET)
endef

$(foreach BUILD,$(MCU_BUILDS),$(foreach PIN,$(BOOTLOADER_PINS),$(eval $(call CREATE_BOOTLOADER_TARGET,$(BUILD),$(PIN)))))

bootloaders: $(ALL_BUILDS)

updaters: $(BLU_BUILDS)

# include the targets for installing tools
include $(ROOT)/make/tools_install.mk

# useful target to list all of the board targets so you can see what
# make target to use for your board
targets:
	$(QUIET)echo List of targets. To build a target use 'make TARGETNAME'
	$(QUIET)echo $(ALL_BUILDS)

updater_targets:
	$(QUIET)echo List of updater targets. To build a target use 'make TARGETNAME'
	$(QUIET)echo $(BLU_BUILDS)
