#include "eeprom.h"

#include <string.h>

#include "at32f421_flash.h"

#define page_size 0x400 // 1 kb for f051

bool save_flash_nolib(const uint8_t* data, uint32_t length, uint32_t add)
{
    if ((add & 0x3) != 0 || (length & 0x3) != 0) {
	return false;
    }
    const uint32_t data_length = length / 4;

    // unlock flash
    flash_unlock();

    // erase page if address even divisable by 1024
    if ((add % page_size) == 0) {
        flash_sector_erase(add);
    }

    uint32_t index = 0;
    while (index < data_length) {
	uint32_t word;
	memcpy(&word, &data[index*4], sizeof(word));
	flash_word_program(add + (index * 4), word);
	flash_flag_clear(FLASH_PROGRAM_ERROR | FLASH_EPP_ERROR | FLASH_OPERATE_DONE);
        index++;
    }
    flash_lock();

    // ensure data is correct
    return memcmp(data, (const void *)add, length) == 0;
}

void read_flash_bin(uint8_t* data, uint32_t add, int out_buff_len)
{
    memcpy(data, (void*)add, out_buff_len);
}
