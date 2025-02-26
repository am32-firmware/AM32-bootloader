/********************************** (C) COPYRIGHT *******************************
 * File Name          : ch32v20x_it.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/12/29
 * Description        : Main Interrupt Service Routines.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "ch32v20x_it.h"

void NMI_Handler(void) __attribute__((interrupt));
void HardFault_Handler(void) __attribute__((interrupt));

void NMI_Handler(void)
{
  while (1) {
  }
}

void HardFault_Handler(void)
{
  while (1) {

  }
}

//for tenkhz
void SysTick_Handler(void)
{
  SysTick->SR = 0;
}
