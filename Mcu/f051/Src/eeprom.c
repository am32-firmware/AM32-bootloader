/*
 * bootloader.c
 *
 *  Created on: Mar. 25, 2020
 *      Author: Alka
 *
 */

#include "eeprom.h"

#include <string.h>

#define page_size 0x400 // 1 kb for f051

static const uint32_t FLASH_FKEY1 = 0x45670123;
static const uint32_t FLASH_FKEY2 = 0xCDEF89AB;

bool save_flash_nolib(const uint8_t* data, uint32_t length, uint32_t add)
{
  if ((add & 0x1) != 0 || (length & 0x1) != 0) {
    return false;
  }
  const uint32_t data_length = length / 2;

  // unlock flash
  while ((FLASH->SR & FLASH_SR_BSY) != 0) {
    /*  add time-out*/
  }
  if ((FLASH->CR & FLASH_CR_LOCK) != 0) {
    FLASH->KEYR = FLASH_FKEY1;
    FLASH->KEYR = FLASH_FKEY2;
  }

  // erase page if address even divisable by 1024
  if ((add % page_size) == 0) {
    FLASH->CR = FLASH_CR_PER;
    FLASH->AR = add;
    FLASH->CR |= FLASH_CR_STRT;
    while ((FLASH->SR & FLASH_SR_BSY) != 0) {
      /*  add time-out */
    }
    FLASH->SR = FLASH_SR_EOP;
    FLASH->CR = 0;
  }

  volatile uint32_t write_cnt = 0, index = 0;
  while (index < data_length) {
    uint16_t word16;
    memcpy(&word16, &data[index*2], sizeof(word16));
    FLASH->CR = FLASH_CR_PG; /* (1) */
    *(__IO uint16_t*)(add + write_cnt) = word16;
    while ((FLASH->SR & FLASH_SR_BSY) != 0) { /*  add time-out  */
    }
    FLASH->SR = FLASH_SR_EOP;
    FLASH->CR = 0;
    write_cnt += 2;
    index++;
  }
  SET_BIT(FLASH->CR, FLASH_CR_LOCK);

  // ensure data is correct
  return memcmp(data, (const void *)add, length) == 0;
}

void read_flash_bin(uint8_t* data, uint32_t add, int out_buff_len)
{
  memcpy(data, (const uint8_t *)add, out_buff_len);
}
