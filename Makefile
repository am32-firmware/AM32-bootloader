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
MAIN_SRC_DIR := Src
MAIN_INC_DIR := Inc

SRC_DIRS_COMMON := $(MAIN_SRC_DIR)

# Working directories
ROOT := $(patsubst %/,%,$(dir $(lastword $(MAKEFILE_LIST))))

# include the rules for OS independence
include $(ROOT)/make/tools.mk

# MCU builds, if with _xxK then adds build with given flash size
MCU_BUILDS := E230 F031 F051 F415 F415_128K F421 G071 G071_64K L431 G431

# we support bootloader comms on a list of possible pins
BOOTLOADER_PINS = PB4 PA2 PA15

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
CFLAGS_BASE := -fsingle-precision-constant -fomit-frame-pointer -ffast-math
CFLAGS_BASE += -I$(MAIN_INC_DIR) -g3 -O2 -Wall -ffunction-sections

CFLAGS_COMMON := $(CFLAGS_BASE)

# Linker options
LDFLAGS_COMMON := -specs=nano.specs $(LIBS) -Wl,--gc-sections -Wl,--print-memory-usage

# Search source files
SRC_COMMON := $(foreach dir,$(SRC_DIRS_COMMON),$(wildcard $(dir)/*.[cs]))

# configure some directories that are relative to wherever ROOT_DIR is located
OBJ := obj
BIN_DIR := $(ROOT)/$(OBJ)

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
LDSCRIPT_BL := bootloader/ldscript_bl.ld

# get a define in form -DBOARD_FLASH_SIZE=N if the target has a _NK suffix
define flash_size_flag
$(if $(findstring _,$1),-DBOARD_FLASH_SIZE=$(subst K,,$(word 2,$(subst _, ,$1))))
endef

# get a tag in the form _nK if the build has a _nK suffix
define get_k_tag
$(if $(findstring _,$1),_$(patsubst %_,%,$(word 2,$(subst _, ,$1))))
endef

# bootloader target names for example "make AM32_F421_BOOTLOADER" with optional _nK suffix
define BOOTLOADER_BASENAME
$(IDENTIFIER)_$(call base_mcu,$(1))_BOOTLOADER_$(2)$(call get_k_tag,$(1))
endef

# bootloader target names with version for filename
define BOOTLOADER_BASENAME_VER
$(call BOOTLOADER_BASENAME,$(1),$(2))_V$(BOOTLOADER_VERSION)
endef

# list of targets formed using CREATE_BOOTLOADER_TARGET
ALL_BUILDS :=

# create a bootloader build target given a build in the form MCU or MCU_nK and a PIN
define CREATE_BOOTLOADER_TARGET
$(eval BUILD := $(1))
$(eval PIN := $(2))
$(eval MCU := $$(call base_mcu,$$(1)))
$(eval EXTRA_CFLAGS := $(call flash_size_flag,$(1)))
$(eval ELF_FILE := $(BIN_DIR)/$(call BOOTLOADER_BASENAME_VER,$(BUILD),$(PIN)).elf)
$(eval HEX_FILE := $(ELF_FILE:.elf=.hex))
$(eval DEP_FILE := $(ELF_FILE:.elf=.d))
$(eval TARGET := $(call BOOTLOADER_BASENAME,$(BUILD),$(PIN)))

-include $(DEP_FILE)

$(ELF_FILE): CFLAGS_BL := $$(MCU_$(MCU)) $$(CFLAGS_$(MCU)) $$(CFLAGS_BASE) -DBOOTLOADER -DUSE_$(PIN) $(EXTRA_CFLAGS)
$(ELF_FILE): LDFLAGS_BL := $$(LDFLAGS_COMMON) $$(LDFLAGS_$(MCU)) -T$$(LDSCRIPT_BL)
$(ELF_FILE): $$(SRC_$(MCU)_BL) $$(SRC_BL)
	$$(QUIET)echo building bootloader for $(BUILD) with pin $(PIN)
	$$(QUIET)$$(MKDIR) -p $(OBJ)
	$$(QUIET)echo Compiling $(notdir $$@)
	$$(QUIET)$$(CC) $$(CFLAGS_BL) $$(LDFLAGS_BL) -MMD -MP -MF $(DEP_FILE) -o $$(@) $$(SRC_$(MCU)_BL) $$(SRC_BL) -Os
	$$(QUIET)$$(CP) -f $$@ $$(OBJ)$$(DSEP)debug.elf
	$$(QUIET)$$(CP) -f Mcu$(DSEP)$(call lc,$(MCU))$(DSEP)openocd.cfg $$(OBJ)$$(DSEP)openocd.cfg > $$(NUL)

# Generate bin and hex files
$(HEX_FILE): $(ELF_FILE)
	$$(QUIET)echo Generating $(notdir $$@)
	$$(QUIET)$$(OBJCOPY) -O binary $$(<) $$(@:.hex=.bin)
	$$(QUIET)$$(OBJCOPY) $$(<) -O ihex $$(@:.bin=.hex)

$(TARGET): $(HEX_FILE)

# add to list
ALL_BUILDS := $(ALL_BUILDS) $(TARGET)
endef

$(foreach BUILD,$(MCU_BUILDS),$(foreach PIN,$(BOOTLOADER_PINS),$(eval $(call CREATE_BOOTLOADER_TARGET,$(BUILD),$(PIN)))))

bootloaders: $(ALL_BUILDS)

# include the targets for installing tools
include $(ROOT)/make/tools_install.mk

# useful target to list all of the board targets so you can see what
# make target to use for your board
targets:
	$(QUIET)echo List of targets. To build a target use 'make TARGETNAME'
	$(QUIET)echo $(ALL_BUILDS)
