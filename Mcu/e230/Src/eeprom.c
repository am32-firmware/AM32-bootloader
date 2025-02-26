#include "eeprom.h"

#include <string.h>

#include "gd32e23x_fmc.h"

#define page_size 0x400 // 1 kb for e230

bool save_flash_nolib(const uint8_t* data, uint32_t length, uint32_t add)
{
  if ((add & 0x3) != 0 || (length & 0x3) != 0) {
    return false;
  }
  const uint32_t data_length = length / 4;

  // unlock flash

  fmc_unlock();

  // erase page if address even divisable by 1024
  if ((add % page_size) == 0) {
    fmc_page_erase(add);
  }

  volatile uint32_t index = 0;
  while (index < data_length) {
    uint32_t word;
    memcpy(&word, (void*)(data+(index*4)), sizeof(word));
    fmc_word_program(add + (index * 4), word);
    fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR);
    index++;
  }
  fmc_lock();

  // ensure data is correct
  return memcmp(data, (const void *)add, length) == 0;
}

void read_flash_bin(uint8_t* data, uint32_t add, int out_buff_len)
{
  memcpy(data, (void*)add, out_buff_len);
}
