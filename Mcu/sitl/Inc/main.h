/*
  main.h for the SITL bootloader build: host includes and the rename of
  the bootloader main(), which runs under the SITL harness process main
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <sitl_bl.h>

#define main am32_bl_main

// the host libc declares memmem; rename the bootloader's static copy
// (string.h is already included above, so its declaration keeps the
// real name and only the bootloader's definition and calls change)
#define memmem bl_memmem

// GPIO port tokens, only used as arguments to the blutil.h inlines
#define GPIOA 0
#define GPIOB 1
#define GPIOC 2

#define NVIC_SystemReset() sitl_bl_system_reset()

static inline void __disable_irq(void)
{
}
