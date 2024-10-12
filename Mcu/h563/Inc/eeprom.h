/*
 * bootloader.h
 *
 *  Created on: Mar. 25, 2020
 *      Author: Alka
 */
#include "main.h"
#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#endif /* INC_BOOTLOADER_H_ */

#include <stdbool.h>

// void save_to_flash(uint8_t *data);
// void read_flash(uint8_t* data, uint32_t address);
// void save_to_flash_bin(uint8_t *data, int length, uint32_t add);
void read_flash_bin(uint8_t* data, uint32_t add, int out_buff_len);
bool save_flash_nolib(const uint8_t* data, uint32_t length, uint32_t add);
void flash_lock();
void flash_unlock();
bool flash_busy();
bool flash_dbne();
void flash_erase_sector(uint8_t sector);
void flash_program_word(uint16_t word, uint32_t address);
bool flash_wbne();
void flash_enable_write();
void flash_disable_write();