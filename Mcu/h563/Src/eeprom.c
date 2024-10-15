// 0x08fe000 (0x08000000 + 0x2000*127) // can't read
// 0x08fc000 (0x08000000 + 0x2000*126) // can't read
// 0x08fa000 (0x08000000 + 0x2000*125) // CAN read
#include "eeprom.h"

#include <string.h>

// !!!!!!!!!!!!!!!!!!!
// Per reference manual:
// The application software must not unlock an
// already unlocked register, otherwise this
// register remains locked until the next system reset.
// !!!!!!!!!!!!!!!!!!!!

// #define APP_START (uint32_t)0x08001000
// #define FLASH_STORAGE 0x08005000  // at the 31kb mark
// #define page_size 0x1800
#define EEPROM_PAGE_SIZE 0x2000 // 8kb for h563

uint32_t FLASH_FKEY1 = 0x45670123;
uint32_t FLASH_FKEY2 = 0xCDEF89AB;

bool memcmp16(uint16_t* d1, uint16_t* d2, int length) {
    for (int i = 0; i < length; i++) {
        if (d1[i] != d2[i]) {
            return false;
        }
    }
    return true;
}

bool save_flash_nolib(const uint8_t* data, uint32_t length, uint32_t add)
{
    // ensure address alignment to 16 bit boundaries
    // and length is divisible by 2
    if ((add & 0x1) != 0 || (length & 0x1) != 0) {
	    return false;
    }
    // copy submitted data to native data width array
    // ie we can only program 16 bit data to flash
    // TODO this copy is slow
    uint16_t data_to_FLASH[length / 2];
    // memset(data_to_FLASH, 0, length / 2);
    for (uint32_t i = 0; i < length / 2; i++) {
        data_to_FLASH[i] = data[i * 2 + 1] << 8 | data[i * 2]; // make 16 bit
    }

    // iterate by 16 bit words
    volatile uint32_t data_length = length / 2;

    while (flash_busy());

    // erase page if address even divisable by page size
    if ((add % EEPROM_PAGE_SIZE) == 0) {
        flash_erase_sector(126);
    }
    
    volatile uint32_t write_cnt = 0, index = 0;
    while (index < data_length) {
        flash_program_word(data_to_FLASH[index], (add + write_cnt));
        write_cnt += 2;
        index++;
    }
    // ensure data is correct
    return memcmp16(data_to_FLASH, (uint16_t*)add, length) == 0;
}

// see https://community.st.com/t5/stm32-mcus-products/stm32h5-high-cycle-data-read-more-than-16bit-at-once/td-p/584258
// "If the application reads an OTP data or flash high-cycle data not previously written,
// a double ECC error is reported and only a word full of set bits is returned"
void read_flash_bin(uint8_t* data, uint32_t add, int out_buff_len)
{
    for (int i = 0; i < out_buff_len; i++) {
        data[i] = *(uint8_t*)(add + i);
    }
}

void flash_erase_sector(uint8_t sector)
{
    // sector out of range
    // if (sector > FLASH_MAX_SECTORS - 1) {
    //     return;
    // }
    // sector must be aligned to page size
    // if (sector%EEPROM_PAGE_SIZE) {
    //     return;
    // }

    while (flash_busy());
    while (flash_dbne());


    flash_unlock();

    FLASH->NSCR &= ~FLASH_CR_SNB_Msk;
    FLASH->NSCR |= FLASH_CR_SER | (sector << FLASH_CR_SNB_Pos);
    // sector << FLASH_CR_SNB_Pos;
    FLASH->NSCR |= FLASH_CR_START;

    while (flash_busy());
    while (flash_dbne());

    // maybe not necessary
    // FLASH->NSCR &= ~FLASH_CR_SNB_Msk;

    FLASH->NSCR &= ~FLASH_CR_SER;
    flash_lock();
}

bool flash_busy()
{
    return FLASH->NSSR & FLASH_SR_BSY;
}

bool flash_dbne()
{
    return FLASH->NSSR & FLASH_SR_DBNE;
}

void flash_unlock()
{
    // unlock the flash
    if ((FLASH->NSCR & FLASH_CR_LOCK) != 0) {
        FLASH->NSKEYR = FLASH_FKEY1;
        FLASH->NSKEYR = FLASH_FKEY2;
    }
    while (FLASH->NSCR & FLASH_CR_LOCK)
    {};
}

void flash_lock()
{
    SET_BIT(FLASH->NSCR, FLASH_CR_LOCK);
}

void flash_program_word(uint16_t word, uint32_t address)
{
    while (flash_busy());
    while (flash_dbne());

    while (flash_wbne());

    flash_unlock();

    flash_enable_write();

    *(__IO uint16_t*)address = word;
    while (flash_busy());
    flash_disable_write();
    flash_lock();
}

void flash_enable_write()
{
    FLASH->NSCR |= FLASH_CR_PG;
}

void flash_disable_write()
{
    FLASH->NSCR &= ~FLASH_CR_PG;
}
bool flash_wbne()
{
    // return FLASH->NSSR & FLASH_SR_WBNE;
    return false;
}
