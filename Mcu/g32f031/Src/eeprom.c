#include "eeprom.h"

#include <string.h>

/*
  Same MAIN-FLASH sequence as AM32-CB APP / Geehy simple boot on G32F031.
  Only difference: return memcmp result for AM32 bootloader protocol.

  Do NOT use "BUSY==0 OR OPEND==1" as a combined wait: when both are 0
  (operation not started yet) that condition exits immediately and skips
  the real erase/program. Official text means either indicator is valid
  *after* the operation has run; simple/APP code correctly waits BUSY only.
 */
#define page_size 0x200U

bool save_flash_nolib(const uint8_t* data, uint32_t length, uint32_t add)
{
  if (data == NULL || length == 0U) {
    return false;
  }

  if ((add & 0x3U) != 0U || (length & 0x3U) != 0U) {
    return false;
  }

  uint32_t index = 0;
  uint32_t pAddress = add;
  const uint32_t data_length = length / 4U;

  /* stack buffer — bootloader must not depend on malloc/heap */
  uint32_t words[64];
  if (data_length > (sizeof(words) / sizeof(words[0]))) {
    return false;
  }

  for (uint32_t i = 0; i < data_length; i++) {
    words[i] =
      ((uint32_t)data[i * 4U + 3U] << 24) |
      ((uint32_t)data[i * 4U + 2U] << 16) |
      ((uint32_t)data[i * 4U + 1U] << 8) |
      ((uint32_t)data[i * 4U + 0U]);
  }

  __disable_irq();

  DDL_FLASH_RKEY_Unlock();
  DDL_FLASH_MKEY_Unlock();

  if ((pAddress % page_size) == 0U) {
    while (DDL_FLASH_IsActiveFlag_BUSY()) {
    }
    DDL_FLASH_SetOperationMode(DDL_FLASH_OPERATE_SECTORERASE);
    *((volatile uint32_t *)pAddress) = 0xA5A5U;
    while (DDL_FLASH_IsActiveFlag_BUSY()) {
    }
  }

  while (index < data_length) {
    if (*((__IO uint32_t *)pAddress) == 0xFFFFFFFFU) {
      DDL_FLASH_SetOperationMode(DDL_FLASH_OPERATE_WRITE);
      *((volatile uint32_t *)pAddress) = words[index];
      while (DDL_FLASH_IsActiveFlag_BUSY()) {
      }
    }

    index++;
    pAddress += 4U;
  }

  DDL_FLASH_MKEY_Lock();
  DDL_FLASH_RKEY_Lock();

  __enable_irq();

  return memcmp(data, (const void *)add, length) == 0;
}

void read_flash_bin(uint8_t* data, uint32_t add, int out_buff_len)
{
  for (int i = 0; i < out_buff_len; i++) {
    data[i] = *(uint8_t *)(add + i);
  }
}
