/*
 * bootloader.c
 *
 *  Created on: Mar. 25, 2020
 *      Author: Alka
 *
 */

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
#define page_size 0x2000 // 8kb for h563

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
    // TODO this copy is slow
    uint16_t data_to_FLASH[length / 2];
    memset(data_to_FLASH, 0, length / 2);
    for (uint16_t i = 0; i < length / 2; i++) {
        data_to_FLASH[i] = data[i * 2 + 1] << 8 | data[i * 2]; // make 16 bit
    }

    // iterate by 16 bit words
    const uint32_t data_length = length / 2;


    while (flash_busy());
    // erase page if address even divisable by page size
    if ((add % page_size) == 0) {
        flash_erase_sector(127);
    }
    while (flash_busy());

    // program flash
    volatile uint32_t write_cnt = 0, index = 0;
    while (index < data_length) {
        flash_program_word(data_to_FLASH[index], (add + write_cnt));
        write_cnt += 2;
        index++;
    }
    // ensure data is correct
    return memcmp16(data_to_FLASH, (uint16_t*)add, length) == 0;
}


void read_flash_bin(uint8_t* data, uint32_t add, int out_buff_len)
{
    // FLASH->EDATA1R_CUR |= 1 << 15;

    // uint32_t readData[10];
    int length = out_buff_len / 2;
    // uint16_t readData[length];
    uint16_t* readData = (uint16_t*)data;
    // volatile uint32_t read_data;
    for (int i = 0; i < length; i++) {
        uint32_t eeprom_add = add + i*2;
        readData[i] = *(uint16_t*)(eeprom_add);
        // readData[i] = *(uint32_t*)(add + i);
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