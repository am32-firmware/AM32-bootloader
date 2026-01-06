/*
 * bootloader.c
 *
 *  Created on: Mar. 25, 2020
 *      Author: Alka
 *
 */

#include "eeprom.h"

#include <string.h>

uint32_t status = 0;

//static const uint32_t FLASH_FKEY1 = 0x45670123;
//static const uint32_t FLASH_FKEY2 = 0xCDEF89AB;

bool save_flash_nolib(const uint8_t* data, uint32_t length, uint32_t add)
{
//	//Check if address and length is within 128 bytes boundary
//    if ((add % 8192) != 0) {
//    	return 1;
//    }
//
//    __disable_irq();
//
////    memset(&s_flashDriver, 0, sizeof(flash_config_t));
////
////    //Check if init went successful
////    status = FLASH_API->flash_init(&s_flashDriver);
////    if (status) {
////    	__asm volatile ("nop");
////    }
//
//    uint32_t pflashBlockBase  = 0U;
//    uint32_t pflashTotalSize  = 0U;
//    uint32_t pflashSectorSize = 0U;
//    uint32_t PflashPageSize   = 0U;
//
//    /* Get flash properties kFLASH_ApiEraseKey */
//    FLASH_API->flash_get_property(&s_flashDriver, kFLASH_PropertyPflashBlockBaseAddr, &pflashBlockBase);
//    FLASH_API->flash_get_property(&s_flashDriver, kFLASH_PropertyPflashSectorSize, &pflashSectorSize);
//    FLASH_API->flash_get_property(&s_flashDriver, kFLASH_PropertyPflashTotalSize, &pflashTotalSize);
//    FLASH_API->flash_get_property(&s_flashDriver, kFLASH_PropertyPflashPageSize, &PflashPageSize);
////
//////    uint32_t eeprom_address = s_flashDriver.PFlashBlockBase + (s_flashDriver.PFlashTotalSize - (1 * s_flashDriver.PFlashSectorSize));
////    uint32_t dest_addr = pflashBlockBase + (pflashTotalSize - pflashSectorSize);
//
//	//Update flash caches
//	modifyReg32(&SYSCON->LPCAC_CTRL, 0, SYSCON_LPCAC_CTRL_DIS_LPCAC(1));
//
//    //Erase last sector
//	status = FLASH_API->flash_erase_sector(&s_flashDriver, add, pflashSectorSize, kFLASH_ApiEraseKey);
//	if (status) {
//		__asm volatile ("nop");
//	}
//
//	//Verify sector erase
//	status = FLASH_API->flash_verify_erase_sector(&s_flashDriver, add, pflashSectorSize);
//	if (status) {
//		__asm volatile ("nop");
//	}
//
//	//Update flash caches
//	modifyReg32(&SYSCON->LPCAC_CTRL, 0, SYSCON_LPCAC_CTRL_DIS_LPCAC(1));
//
////	data[0] = 0x12;
////	data[1] = 0x34;
////	data[2] = 0xa5;
////	data[3] = 0x5a;
//
////	uint32_t num_flash_pages = length / PflashPageSize + ((length % PflashPageSize) > 0);
//
//	//Program data
//	status = FLASH_API->flash_program_page(&s_flashDriver, add, data, length);
//	if (status) {
//		__asm volatile ("nop");
//	}
//
////	uint8_t readout[400] = {0};
////	status = FLASH_API->flash_read(&s_flashDriver, add, readout, length);
////	if (status) {
////		__asm volatile ("nop");
////	}
//
//	//Verify programmed data
//	uint32_t failed_data_addr 	= 0;
//	uint32_t failed_data 		= 0;
//	status = FLASH_API->flash_verify_program(&s_flashDriver, add, length, data, &failed_data_addr, &failed_data);
//	if (status) {
//		__asm volatile ("nop");
//	}
//
//	//Update flash caches
//	modifyReg32(&SYSCON->LPCAC_CTRL, 0, SYSCON_LPCAC_CTRL_DIS_LPCAC(1));
////	SYSCON->LPCAC_CTRL |= SYSCON_LPCAC_CTRL_DIS_LPCAC(1U);
//
//	//Check if verify program found failed data
//	if (failed_data_addr != 0) {
//		__asm volatile ("nop");
//	}
//
//	__enable_irq();
//
//	return (failed_data | failed_data_addr);


	return 0;


//  if ((add & 0x1) != 0 || (length & 0x1) != 0) {
//    return false;
//  }
//  const uint32_t data_length = length / 2;
//
//  // unlock flash
//  while ((FLASH->SR & FLASH_SR_BSY) != 0) {
//    /*  add time-out*/
//  }
//  if ((FLASH->CR & FLASH_CR_LOCK) != 0) {
//    FLASH->KEYR = FLASH_FKEY1;
//    FLASH->KEYR = FLASH_FKEY2;
//  }
//
//  // erase page if address even divisable by 1024
//  if ((add % page_size) == 0) {
//    FLASH->CR = FLASH_CR_PER;
//    FLASH->AR = add;
//    FLASH->CR |= FLASH_CR_STRT;
//    while ((FLASH->SR & FLASH_SR_BSY) != 0) {
//      /*  add time-out */
//    }
//    FLASH->SR = FLASH_SR_EOP;
//    FLASH->CR = 0;
//  }
//
//  volatile uint32_t write_cnt = 0, index = 0;
//  while (index < data_length) {
//    uint16_t word16;
//    memcpy(&word16, &data[index*2], sizeof(word16));
//    FLASH->CR = FLASH_CR_PG; /* (1) */
//    *(__IO uint16_t*)(add + write_cnt) = word16;
//    while ((FLASH->SR & FLASH_SR_BSY) != 0) { /*  add time-out  */
//    }
//    FLASH->SR = FLASH_SR_EOP;
//    FLASH->CR = 0;
//    write_cnt += 2;
//    index++;
//  }
//  SET_BIT(FLASH->CR, FLASH_CR_LOCK);
//
//  // ensure data is correct
//  return memcmp(data, (const void *)add, length) == 0;
}

void read_flash_bin(uint8_t* data, uint32_t add, int out_buff_len)
{
//  memcpy(data, (const uint8_t *)add, out_buff_len);
}
