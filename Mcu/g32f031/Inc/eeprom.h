#include "main.h"
#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#endif

#include <stdbool.h>

void read_flash_bin(uint8_t* data, uint32_t add, int out_buff_len);
bool save_flash_nolib(const uint8_t* data, uint32_t length, uint32_t add);