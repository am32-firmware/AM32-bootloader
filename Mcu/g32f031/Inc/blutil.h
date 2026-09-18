/*
  MCU specific utility functions for the bootloader
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "main.h"

#define RAM_BASE 0x20000000
#define RAM_SIZE (8*1024)

#ifndef MCU_FLASH_START
#define MCU_FLASH_START 0x00000000
#endif

/*
  use 64k of flash
 */
#ifndef BOARD_FLASH_SIZE
#define BOARD_FLASH_SIZE 64
#endif

#define GPIO_PIN(n) (1U<<(n))

#define GPIO_PULL_NONE DDL_GPIO_PULL_NO
#define GPIO_PULL_UP   DDL_GPIO_PULL_UP
#define GPIO_PULL_DOWN DDL_GPIO_PULL_DOWN

#define GPIO_OUTPUT_PUSH_PULL DDL_GPIO_OUTPUT_PUSHPULL

// assume 64MHz crystal
uint32_t SystemCoreClock = 64000000U;

static inline void gpio_mode_set_input(uint32_t pin, uint32_t pull_up_down)
{
  DDL_GPIO_LockKey(input_port, DDL_GPIO_LOCK_DISABLE);
  DDL_GPIO_SetPinInputMode(input_port, pin, DDL_GPIO_INPUT_ENABLE);
  DDL_GPIO_SetPinMode(input_port, pin, DDL_GPIO_MODE_INPUT);
  DDL_GPIO_SetPinPull(input_port, pin, pull_up_down);
  DDL_GPIO_LockKey(input_port, DDL_GPIO_LOCK_ENABLE);
}

static inline void gpio_mode_set_output(uint32_t pin, uint32_t output_mode)
{
  DDL_GPIO_LockKey(input_port, DDL_GPIO_LOCK_DISABLE);
  DDL_GPIO_SetPinInputMode(input_port, pin, DDL_GPIO_INPUT_DISABLE);
  DDL_GPIO_SetPinMode(input_port, pin, DDL_GPIO_MODE_OUTPUT);
  DDL_GPIO_SetPinOutputType(input_port, pin, output_mode);
  DDL_GPIO_LockKey(input_port, DDL_GPIO_LOCK_ENABLE);
}

static inline void gpio_set(uint32_t pin)
{
  DDL_GPIO_SetOutputPin(input_port, pin);
}

static inline void gpio_clear(uint32_t pin)
{
  DDL_GPIO_ResetOutputPin(input_port, pin);
}

static inline bool gpio_read(uint32_t pin)
{
  return DDL_GPIO_IsInputPinSet(input_port, pin);
}

#define BL_TIMER BTMR0

/*
  initialise timer for 1us per tick
 */
static inline void bl_timer_init(void)
{
  DDL_BTMR_InitTypeDef BTMR_InitStruct = {0};

  DDL_RCC_Unlock();
  DDL_APB_GRP1_EnableClock(DDL_APB_GRP1_PERIPH_BTMR0);
  DDL_RCC_Lock();

  BTMR_InitStruct.Prescaler = 63;
  BTMR_InitStruct.CounterMode = DDL_BTMR_COUNTERMODE_UP;
  BTMR_InitStruct.Autoreload = 0xFFFF;
  DDL_BTMR_Init(BL_TIMER, &BTMR_InitStruct);

  DDL_BTMR_EnableCounter(BL_TIMER);
}

static inline void bl_timer_disable(void)
{
  DDL_BTMR_DeInit(BL_TIMER);
}

static inline uint16_t bl_timer_us(void)
{
  return DDL_BTMR_GetCounter(BL_TIMER);
}

static inline void bl_clock_config(void)
{
    DDL_RCC_Unlock();

    /* Set HSIEN */
    DDL_RCC_HSI_Enable();
    /* Wait for HSI READY */
    while (DDL_RCC_HSI_IsReady() != 1U)
    {
    }

    /* Configure FLASH latency */
    DDL_FLASH_SetLatency(DDL_FLASH_LATENCY3);

    /* Set HSI clock as system source clock, SYS_CLK = 64MHz */
    DDL_RCC_SetSysClkSource(DDL_RCC_SYS_CLKSOURCE_HSI);
    while (DDL_RCC_GetSysClkSource() != DDL_RCC_SYS_CLKSOURCE_HSI)
    {
    }

    /* Configure HSI Prescaler, SYS_CLK = HSI_CLK/1 */
    DDL_RCC_SetHSIPrescaler(DDL_RCC_HSI_DIV_1);

    /* Configure AHB Prescaler, AHB_CLK = SYS_CLK/1 */
    DDL_RCC_SetAHBPrescaler(DDL_RCC_AHB_DIV_1);

    /* Configure APB prescaler, APB_CLK = AHB_CLK/1 */
    DDL_RCC_SetAPBPrescaler(DDL_RCC_APB_DIV_1);

    /* Update SystemCoreClock variable */
    DDL_SetSystemCoreClock(SystemCoreClock);

    /* Set Systick to 1ms in using frequency set to SystemCoreClock */
    DDL_Init1msTick(SystemCoreClock);

    DDL_RCC_Lock();
}

static inline void bl_gpio_init(void)
{
  DDL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* G32F031 RCC registers are write-protected; must unlock before AHBCLKEN */
  DDL_RCC_Unlock();
  DDL_AHB_GRP1_EnableClock(DDL_AHB_GRP1_PERIPH_GPIOA);
  DDL_AHB_GRP1_EnableClock(DDL_AHB_GRP1_PERIPH_GPIOB);
  DDL_RCC_Lock();

  DDL_GPIO_LockKey(input_port, DDL_GPIO_LOCK_DISABLE);

  /* set as input with pull-up */
  GPIO_InitStruct.Pin = input_pin;
  GPIO_InitStruct.Mode = DDL_GPIO_MODE_INPUT;
  GPIO_InitStruct.InputEnable = DDL_GPIO_INPUT_ENABLE;
  GPIO_InitStruct.Pull = DDL_GPIO_PULL_UP;
  DDL_GPIO_Init(input_port, &GPIO_InitStruct);

  DDL_GPIO_LockKey(input_port, DDL_GPIO_LOCK_ENABLE);
}

/*
  return true if the MCU booted under a software reset
 */
static inline bool bl_was_software_reset(void)
{
  return (DDL_RCC_IsActiveFlag_SFTRST() != 0U);
}

/*
  no need for any action in SystemInit
 */
void SystemInit()
{
}

static inline void jump_to_application(void)
{
  __disable_irq();
  bl_timer_disable();
  const uint32_t app_address = MCU_FLASH_START + FIRMWARE_RELATIVE_START;
  const uint32_t *app_data = (const uint32_t *)app_address;
  const uint32_t stack_top = app_data[0];
  const uint32_t jump_address = app_data[1];

  SCB->VTOR = app_address;

  asm volatile(
    "mov sp, %0\n"
    "msr msp, %0\n"
    "bx %1\n"
    : : "r"(stack_top), "r"(jump_address) :);
}