/*
 *   modified by TempersLee June 21,2024 for CH32V203
 * */

#include "eeprom.h"
#include <string.h>
#include <ch32v20x.h>

// #pragma GCC optimize("O1")

#define page_size 0x100 // 256 byte pages for v203

static void wait_not_busy(void)
{
    while ((FLASH->STATR & FLASH_STATR_BSY) != 0) {}
}

bool save_flash_nolib(const uint8_t* data, uint32_t length, uint32_t add)
{
    if ((add & 0x03) != 0 || (length & 0x03) != 0 || (length+(add&0xFF)) > page_size) {
        return false;
    }

    wait_not_busy();

    FLASH_Unlock_Fast();

    uint32_t flash_buffer[page_size/4];
    const uint8_t page_offset = add & 0xFFU;
    const uint32_t page_base = add & ~0xFFU;

    // get existing data
    memcpy(flash_buffer, (void*)page_base, page_size);

    // overwrite with new data
    memcpy(&flash_buffer[page_offset/4], data, length);

    // fast erase is 256 bytes at a time, normal Flash_Erasepage
    // is 4k at a time
    FLASH_ErasePage_Fast(page_base);

    wait_not_busy();

    FLASH_ProgramPage_Fast(page_base, flash_buffer);

    wait_not_busy();

    FLASH_Lock_Fast();

    // ensure data is correct
    return memcmp(data, (const void *)add, length) == 0;
}

void read_flash_bin(uint8_t* data, uint32_t add, int out_buff_len)
{
    memcpy(data, (const uint8_t *)add, out_buff_len);
}
