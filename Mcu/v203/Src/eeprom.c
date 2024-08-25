/*
 *   modified by TempersLee June 21,2024 for CH32V203
 * */

#include "eeprom.h"
#include <string.h>
#include <ch32v20x.h>

#define page_size 0x100 // 256 byte pages for v203

bool save_flash_nolib(const uint8_t* data, uint32_t length, uint32_t add)
{
    if ((add & 0x03) != 0 || (length & 0x03) != 0 || (length+(add&0xFF)) > page_size) {
	return false;
    }

    // wait for flash not busy
    while ((FLASH->STATR & FLASH_STATR_BSY) != 0) {}

    FLASH_Unlock();

    // erase page if on page boundary
    if ((add % page_size) == 0) {
        // fast erase is 256 bytes at a time, normal Flash_Erasepage
        // is 4k at a time
        FLASH_ErasePage_Fast(add);
    }

    const uint32_t data_length = length/4;
    uint32_t index = 0;
    while (index < data_length) {
        // flash one word at a time
        uint32_t word;
        memcpy((void*)&word, &data[index*4], sizeof(word));
        FLASH_ProgramWord(add+index*4, word);
        index++;
    }

    FLASH_Lock();

    // ensure data is correct
    return memcmp(data, (const void *)add, length) == 0;
}

void read_flash_bin(uint8_t* data, uint32_t add, int out_buff_len)
{
    memcpy(data, (const uint8_t *)add, out_buff_len);
}
