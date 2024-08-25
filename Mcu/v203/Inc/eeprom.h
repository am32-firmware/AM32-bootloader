#pragma once

#include "main.h"
#include <stdbool.h>

bool save_flash_nolib(const uint8_t* data, uint32_t length, uint32_t add);
void read_flash_bin(uint8_t* data, uint32_t add, int out_buff_len);
