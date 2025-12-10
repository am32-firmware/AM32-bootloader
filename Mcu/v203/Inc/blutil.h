/*
  MCU specific utility functions for the bootloader
 */
#pragma once

/*
  20k ram
 */
#define RAM_BASE 0x20000000
#define RAM_SIZE 20*1024

/*
  the first word of the app is not the stack address, disable the check in jump()
 */
#define DISABLE_APP_HEADER_CHECKS

/*
  use 64k flash for now
 */
#define BOARD_FLASH_SIZE 64

#define GPIO_PIN(n) (1U<<(n))

#define GPIO_PULL_NONE 0 // floating
#define GPIO_PULL_UP   1
#define GPIO_PULL_DOWN 2

#define GPIO_OUTPUT_PUSH_PULL (GPIO_Speed_2MHz)

// #pragma GCC optimize("O0")

#define NOINLINE __attribute((noinline))

static NOINLINE void gpio_mode_set_input(uint32_t pin, uint32_t pull_up_down)
{
  volatile uint32_t *cfgr;
  if (pin >= GPIO_PIN(8)) {
    pin >>= 8;
    cfgr = &input_port->CFGHR;
  } else {
    cfgr = &input_port->CFGLR;
  }
  const uint32_t mul = pin*pin*pin*pin;
  const uint32_t CFG = (*cfgr) & ~(0xf * mul);
  switch (pull_up_down) {
  case GPIO_PULL_NONE:
    *cfgr = CFG | (0x4*mul);
    break;
  case GPIO_PULL_UP:
    input_port->OUTDR |= pin;
    *cfgr = CFG | (0x8*mul);
    break;
  case GPIO_PULL_DOWN:
    input_port->OUTDR &= ~pin;
    *cfgr = CFG | (0x8*mul);
    break;
  }
}

static inline void gpio_mode_set_output(uint32_t pin, uint32_t output_mode)
{
  volatile uint32_t *cfgr;
  if (pin >= GPIO_PIN(8)) {
    pin >>= 8;
    cfgr = &input_port->CFGHR;
  } else {
    cfgr = &input_port->CFGLR;
  }
  const uint32_t mul = pin*pin*pin*pin;
  const uint32_t CFG = (*cfgr) & ~(0xf * mul);
  (*cfgr) = CFG | (output_mode*mul);
}

static inline void gpio_set(uint32_t pin)
{
  input_port->BSHR = pin;
}

static inline void gpio_clear(uint32_t pin)
{
  input_port->BCR = pin;
}

static inline bool gpio_read(uint32_t pin)
{
  return (input_port->INDR & pin) != 0;
}

#define BL_TIMER TIM2

/*
  initialise timer for 1us per tick
 */
static inline void bl_timer_init(void)
{
  RCC->APB1PCENR |= RCC_APB1Periph_TIM2;

  // direct register access to avoid pulling in TIM_TimeBaseInit (saves ~150 bytes)
  BL_TIMER->PSC = 143;         // prescaler for 1us at 144MHz
  BL_TIMER->ATRLR = 0xFFFF;    // period
  BL_TIMER->CTLR1 = 0;         // up counter, no clock division
  BL_TIMER->CNT = 0;           // reset counter
  BL_TIMER->SWEVGR = TIM_PSCReloadMode_Immediate;

  // enable preload and timer
  BL_TIMER->CTLR1 |= TIM_ARPE | TIM_CEN;
}

/*
  disable timer ready for app start
 */
static inline void bl_timer_disable(void)
{
  RCC->APB1PCENR &= ~RCC_APB1Periph_TIM2;
}

static inline uint16_t bl_timer_us(void)
{
  return BL_TIMER->CNT;
}

/*
  initialise clocks
 */
static inline void bl_clock_config(void)
{
  SystemInit();
}

static inline void bl_gpio_init(void)
{
  // enable GPIO clocks via direct register access (avoids RCC_APB2PeriphClockCmd)
  RCC->APB2PCENR |= RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB;

  // use our own gpio_mode_set_input instead of GPIO_Init (saves ~250 bytes)
  gpio_mode_set_input(input_pin, GPIO_PULL_NONE);
}

/*
  return true if the MCU booted under a software reset
 */
static inline bool bl_was_software_reset(void)
{
  return (RCC->RSTSCKR & RCC_SFTRSTF) != 0;
}

void Error_Handler()
{
  while (1) {}
}

/*
  jump from the bootloader to the application code
 */
static inline void jump_to_application(void)
{
  __disable_irq();
  bl_timer_disable();
  const uint32_t app_address = FIRMWARE_RELATIVE_START;
  const uint32_t stack_top = RAM_BASE + RAM_SIZE;

  // set the stack pointer and jump to the application
  asm volatile(
    "mv sp, %0 \n"
    "jr %1 \n"
    :                      // No output operands
    : "r"(stack_top), "r"(app_address)
    :                      // No clobbered registers
  );
}
