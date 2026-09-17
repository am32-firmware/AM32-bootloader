#pragma once
#include <stdint.h>

// Hardware dereferences MCU flash addresses directly. SITL can translate
// them when the host cannot map memory at the MCU's address (macOS).
#ifdef MCU_SITL
const void *sitl_bl_flash_ptr(uintptr_t address);
#define FLASH_READ_PTR(address) sitl_bl_flash_ptr(address)
#else
#define FLASH_READ_PTR(address) ((const void *)(uintptr_t)(address))
#endif
