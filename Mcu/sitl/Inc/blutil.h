/*
  MCU specific utility functions for the bootloader: SITL port, thin
  inlines over the sitl_bl runtime
 */
#pragma once

#include <sitl_bl.h>

// match the ARM targets the app header checks are performed against
#define RAM_BASE 0x20000000
#define RAM_SIZE 64 * 1024

#ifndef BOARD_FLASH_SIZE
#define BOARD_FLASH_SIZE 128
#endif

#define GPIO_PIN(n) (1U << (n))

#define GPIO_PULL_NONE 0
#define GPIO_PULL_UP 1
#define GPIO_PULL_DOWN 2

#define GPIO_OUTPUT_PUSH_PULL 0

static inline void gpio_mode_set_input(uint32_t pin, uint32_t pull_up_down)
{
    (void)pin;
    sitl_bl_pin_mode_input(pull_up_down);
}

static inline void gpio_mode_set_output(uint32_t pin, uint32_t output_mode)
{
    (void)pin;
    (void)output_mode;
    sitl_bl_pin_mode_output();
}

static inline void gpio_set(uint32_t pin)
{
    (void)pin;
    sitl_bl_pin_write(1);
}

static inline void gpio_clear(uint32_t pin)
{
    (void)pin;
    sitl_bl_pin_write(0);
}

static inline bool gpio_read(uint32_t pin)
{
    (void)pin;
    return sitl_bl_pin_read() != 0;
}

static inline void bl_timer_init(void)
{
}

static inline void bl_timer_disable(void)
{
}

static inline uint16_t bl_timer_us(void)
{
    return sitl_bl_timer_us();
}

static inline void bl_clock_config(void)
{
}

static inline void bl_gpio_init(void)
{
}

static inline bool bl_was_software_reset(void)
{
    return sitl_bl_was_software_reset();
}

void Error_Handler(void);

#ifdef FIRMWARE_RELATIVE_START
static inline void jump_to_application(void)
{
    sitl_bl_jump_app();
}
#endif
