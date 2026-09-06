/*
  bootloader update firmware

  this updates the bootloader on an AM32 ESC. It assumes the
  bootloader flash sectors are unlocked
 */
#include <main.h>
#include <stdio.h>

#include <version.h>
#include <stdbool.h>
#include "eeprom.h"

#pragma GCC optimize("O0")

#include <string.h>

#if defined(MCXA153)
#define GPIO_PORT_TYPE PORT_Type*
#else
#define GPIO_PORT_TYPE typeof(GPIOA)
#endif

// dummy pin and port so we can re-use blutil.h
static GPIO_PORT_TYPE input_port;
static uint32_t input_pin;

#ifndef MCU_FLASH_START
#define MCU_FLASH_START 0x08000000
#endif

#if !DRONECAN_SUPPORT
#ifdef MCXA153
#define FIRMWARE_RELATIVE_START 0x4000
#else
#define FIRMWARE_RELATIVE_START 0x1000
#endif // MCXA153
#else

#define FIRMWARE_RELATIVE_START 0x4000

#define APP_SIGNATURE_MAGIC1 0x68f058e6
#define APP_SIGNATURE_MAGIC2 0xafcee5a0

/*
  application signature, filled in by set_app_signature.py
 */
const struct {
  uint32_t magic1;
  uint32_t magic2;
  uint32_t fwlen; // total fw length in bytes
  uint32_t crc1; // crc32 up to start of app_signature
  uint32_t crc2; // crc32 from end of app_signature to end of fw
  char mcu[16];
  uint32_t unused[2];
} app_signature __attribute__((section(".app_signature"))) = {
  .magic1 = APP_SIGNATURE_MAGIC1,
  .magic2 = APP_SIGNATURE_MAGIC2,
#if BL_TRANSITION
  /*
    the transition updater is left behind in the application area once
    it has run. An impossible length keeps the new bootloader from ever
    treating it as a bootable application
   */
  .fwlen = 0xFFFFFFFF,
#else
  .fwlen = 0,
#endif
  .crc1 = 0,
  .crc2 = 0,
  .mcu = AM32_MCU,
};
#endif

/*
  use stringize to construct an include of the right bootloader header
 */
#define bl_header BL_HEADER_FILE
#define xstr(x) #x
#define str(x) xstr(x)
#include str(bl_header)

#define PORT_LETTER      0 // dummy

#include <blutil.h>

#if BL_TRANSITION
/*
  the transition updater is flashed by a legacy 4k bootloader, which
  jumps to the application start at 0x08001000. All of our code and the
  bootloader image we carry must be above the 16k we are about to erase,
  so the only thing at 0x08001000 is this two word vector table
 */
extern uint32_t _estack;
extern void Reset_Handler(void);

static const uint32_t stub_vector[2] __attribute__((section(".stub_vector"), used)) = {
  (uint32_t)(uintptr_t)&_estack,
  (uint32_t)(uintptr_t)Reset_Handler
};

#define DEVINFO_MAGIC1 0x5925e3da
#define DEVINFO_MAGIC2 0x4eb863d9

/*
  find the deviceInfo bytes of the devinfo structure in a bootloader
  image, or NULL if the image has none
 */
static const uint8_t *find_device_info(const uint8_t *base, uint32_t len)
{
  const uint32_t magic[2] = { DEVINFO_MAGIC1, DEVINFO_MAGIC2 };
  for (uint32_t ofs=0; ofs+sizeof(magic)+9 <= len; ofs += 4) {
    const uint8_t *deviceInfo = &base[ofs+sizeof(magic)];
    // the deviceInfo always starts with the ESC device type
    if (memcmp(&base[ofs], magic, sizeof(magic)) == 0 &&
        memcmp(deviceInfo, "471", 3) == 0) {
      return deviceInfo;
    }
  }
  return NULL;
}

/*
  eeprom address of a bootloader image, from the flash size code in its
  deviceInfo. Returns 0 if we can't work it out
 */
static uint32_t eeprom_address(const uint8_t *base, uint32_t len)
{
  const uint8_t *deviceInfo = find_device_info(base, len);
  if (deviceInfo == NULL) {
    return 0;
  }
  switch (deviceInfo[4]) {
  case 0x1f: return MCU_FLASH_START + 0x7c00;  // 32k layout
  case 0x35: return MCU_FLASH_START + 0xf800;  // 64k layout
  case 0x2b: return MCU_FLASH_START + 0x1f800; // 128k layout
  }
  return 0;
}

/*
  a CAN bootloader uses the 128k flash layout, so its eeprom is not where
  a legacy 4k bootloader kept it. Leave that page blank rather than
  carrying the old settings over: the new bootloader will not boot the
  application until the ESC is configured again, which is what we want
  after changing bootloader, and the user sets it up from scratch with
  the configurator or the DroneCAN GUI tool.
 */
static void blank_eeprom(void)
{
  const uint32_t addr = eeprom_address(bl_image, sizeof(bl_image));
  if (addr == 0) {
    return;
  }
  // a zero length write erases the page the address starts and programs
  // nothing, leaving the eeprom in its erased 0xFF state
  save_flash_nolib((const uint8_t *)MCU_FLASH_START, 0, addr);
}
#endif // BL_TRANSITION

static void delayMicroseconds(uint32_t micros)
{
  while (micros > 0) {
    uint16_t us = micros>10000?10000:micros;
    const uint16_t us_start = bl_timer_us();
    while ((uint16_t)(bl_timer_us() - us_start) < us) ;
    micros -= us;
  }
}

static void flash_bootloader(void)
{
  uint32_t length = sizeof(bl_image);
  uint32_t address = MCU_FLASH_START;
  const uint8_t *bl = &bl_image[0];

  while (length > 0) {
    uint32_t chunk = 256;
    if (chunk > length) {
      chunk = length;
    }
    // loop until the flash succeeds. We expect it to pass
    // first time, so this is paranoia
    while (!save_flash_nolib(bl, chunk, address)) {
    }
    length -= chunk;
    address += chunk;
    bl += chunk;
  }
}

int main(void)
{
#if BL_TRANSITION
  // we were entered through the stub vector table, point VTOR at the real one
  SCB->VTOR = MCU_FLASH_START + FIRMWARE_RELATIVE_START;
#endif
  bl_clock_config();
  bl_timer_init();

  // don't risk erasing the bootloader if it already matches
  if (memcmp((const void*)MCU_FLASH_START, bl_image, sizeof(bl_image)) != 0) {
    // give 1.5s for debugger to attach
    delayMicroseconds(1500000);

    /*
      disable interrupts for the whole flash sequence. The updater runs from
      the same flash bank that we are about to erase + reprogram; on dual-bank
      STM32G4 (e.g. STM32G491 / G4Axx in DBANK=1 mode) the bootloader live in
      bank 1 just like the updater code, and bank-1 reads stall during any
      bank-1 write. If an interrupt fires during a write, the CPU may try to
      fetch the handler from the half-erased bank and HardFault, leaving the
      chip stuck with a partially-written bootloader. Disabling interrupts
      avoids the race; we NVIC_SystemReset below so interrupt state is
      naturally restored on the next boot.
     */
    __disable_irq();

#if BL_TRANSITION
    blank_eeprom();
#endif

    // do the flash
    flash_bootloader();
  }

  // and reset
  NVIC_SystemReset();

  return 0;
}
