/*
  MCU specific utility functions for the bootloader
 */
/*
  based on https://github.com/AlkaMotors/AM32_Bootloader_F051/blob/main/Core/
 */
#pragma once

#include <stm32g4xx_ll_rcc.h>

/*
  64k ram
 */
#define RAM_BASE 0x20000000
#define RAM_SIZE 32*1024

/*
  we have up to 512k of flash, but only use 64k for now
 */
#ifndef BOARD_FLASH_SIZE
#define BOARD_FLASH_SIZE 64
#endif

#define GPIO_PIN(n) (1U<<(n))

#define GPIO_PULL_NONE LL_GPIO_PULL_NO
#define GPIO_PULL_UP   LL_GPIO_PULL_UP
#define GPIO_PULL_DOWN LL_GPIO_PULL_DOWN

#define GPIO_OUTPUT_PUSH_PULL LL_GPIO_OUTPUT_PUSHPULL

#ifdef PORT_LETTER
static inline void gpio_mode_set_input(uint32_t pin, uint32_t pull_up_down)
{
  LL_GPIO_SetPinMode(input_port, pin, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinPull(input_port, pin, pull_up_down);
}

static inline void gpio_mode_set_output(uint32_t pin, uint32_t output_mode)
{
  LL_GPIO_SetPinMode(input_port, pin, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinOutputType(input_port, pin, output_mode);
}

static inline void gpio_set(uint32_t pin)
{
  LL_GPIO_SetOutputPin(input_port, pin);
}

static inline void gpio_clear(uint32_t pin)
{
  LL_GPIO_ResetOutputPin(input_port, pin);
}

static inline bool gpio_read(uint32_t pin)
{
  return LL_GPIO_IsInputPinSet(input_port, pin);
}
#endif // PORT_LETTER

#define BL_TIMER TIM2

/*
  initialise timer for 1us per tick
 */
static inline void bl_timer_init(void)
{
  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

  TIM_InitStruct.Prescaler = (160-1); // HSI PLL clock is 160Mz, want 1MHz timer
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 0xFFFFFFFF;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(BL_TIMER, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(BL_TIMER);
  LL_TIM_SetClockSource(BL_TIMER, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_SetTriggerOutput(BL_TIMER, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(BL_TIMER);

  LL_TIM_SetCounterMode(BL_TIMER, LL_TIM_COUNTERMODE_UP);
  LL_TIM_EnableCounter(BL_TIMER);
}

/*
  disable timer ready for app start
 */
static inline void bl_timer_disable(void)
{
  LL_TIM_DeInit(BL_TIMER);
}

static inline uint16_t bl_timer_us(void)
{
  return LL_TIM_GetCounter(BL_TIMER);
}

/*
  initialise clocks
 */
static inline void bl_clock_config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
  while (LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_4) ;
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  while (LL_PWR_IsActiveFlag_VOS() != 0) ;

#ifdef USE_HSE
  // setup to use HSE
#if defined(USE_HSE_BYPASS) && (USE_HSE_BYPASS == 0)
  LL_RCC_HSE_DisableBypass(); // Use crystal mode
#else
  LL_RCC_HSE_EnableBypass(); // Default: Use external oscillator
#endif
  LL_RCC_HSE_Enable();
  while (LL_RCC_HSE_IsReady() != 1) ;
  #define PLL_SRC LL_RCC_PLLSOURCE_HSE
#if HSE_VALUE == 8000000
  #define PLLM_DIV LL_RCC_PLLM_DIV_1
#elif HSE_VALUE == 16000000
  #define PLLM_DIV LL_RCC_PLLM_DIV_2
#elif HSE_VALUE == 24000000
  #define PLLM_DIV LL_RCC_PLLM_DIV_3
#else
  #error "Unsupported HSE_VALUE"
#endif
#else // default to HSI (16MHz)
  LL_RCC_HSI_Enable();
  while (LL_RCC_HSI_IsReady() != 1) ;
  #define PLLM_DIV LL_RCC_PLLM_DIV_2
  #define PLL_SRC LL_RCC_PLLSOURCE_HSI
#endif

  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

  LL_RCC_PLL_ConfigDomain_SYS(PLL_SRC, PLLM_DIV, 40, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_EnableDomain_SYS();

#if DRONECAN_SUPPORT
  LL_RCC_SetFDCANClockSource(LL_RCC_FDCAN_CLKSOURCE_PLL);
  LL_RCC_PLL_EnableDomain_48M();
  MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLQ, LL_RCC_PLLQ_DIV_4);

  // also enable the backup domain registers in TAMP for communication
  // between bootloader and main firmware
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  PWR->CR1 |= PWR_CR1_DBP;
  LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_NONE);
  RCC->BDCR |= RCC_BDCR_RTCEN;
#endif

  LL_RCC_PLL_Enable();
  while (LL_RCC_PLL_IsReady() != 1) ;

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) ;
}

#ifdef PORT_LETTER
static inline void bl_gpio_init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);

  GPIO_InitStruct.Pin = input_pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;

  LL_GPIO_Init(input_port, &GPIO_InitStruct);
}

/*
  RGB LED support (active low)
 */
#ifdef USE_RGB_LED
static inline void bl_led_init(void)
{
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
  LL_GPIO_SetPinMode(GPIOC, GPIO_PIN(LED_RED_PIN), LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinOutputType(GPIOC, GPIO_PIN(LED_RED_PIN), LL_GPIO_OUTPUT_OPENDRAIN);
  LL_GPIO_SetPinMode(GPIOC, GPIO_PIN(LED_GREEN_PIN), LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinOutputType(GPIOC, GPIO_PIN(LED_GREEN_PIN), LL_GPIO_OUTPUT_OPENDRAIN);
  LL_GPIO_SetPinMode(GPIOC, GPIO_PIN(LED_BLUE_PIN), LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinOutputType(GPIOC, GPIO_PIN(LED_BLUE_PIN), LL_GPIO_OUTPUT_OPENDRAIN);
  // LEDs off (open drain high = hi-Z = off)
  LL_GPIO_SetOutputPin(GPIOC, GPIO_PIN(LED_RED_PIN) | GPIO_PIN(LED_GREEN_PIN) | GPIO_PIN(LED_BLUE_PIN));
}

static inline void bl_led_on(void)
{
  LL_GPIO_ResetOutputPin(GPIOC, GPIO_PIN(LED_RED_PIN) | GPIO_PIN(LED_GREEN_PIN) | GPIO_PIN(LED_BLUE_PIN));
}

static inline void bl_led_off(void)
{
  LL_GPIO_SetOutputPin(GPIOC, GPIO_PIN(LED_RED_PIN) | GPIO_PIN(LED_GREEN_PIN) | GPIO_PIN(LED_BLUE_PIN));
}

static inline void bl_led_red_on(void)
{
  LL_GPIO_ResetOutputPin(GPIOC, GPIO_PIN(LED_RED_PIN));
  LL_GPIO_SetOutputPin(GPIOC, GPIO_PIN(LED_GREEN_PIN) | GPIO_PIN(LED_BLUE_PIN));
}
#endif // USE_RGB_LED

/*
  return true if the MCU booted under a software reset
 */
static inline bool bl_was_software_reset(void)
{
  return (RCC->CSR & RCC_CSR_SFTRSTF) != 0;
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
  const uint32_t app_address = MCU_FLASH_START + FIRMWARE_RELATIVE_START;
  const uint32_t *app_data = (const uint32_t *)app_address;
  const uint32_t stack_top = app_data[0];
  const uint32_t JumpAddress = app_data[1];

  // setup vector table
  SCB->VTOR = app_address;

  // setup sp, msp and jump
  asm volatile(
    "mov sp, %0	\n"
    "msr msp, %0	\n"
    "bx	%1	\n"
    : : "r"(stack_top), "r"(JumpAddress) :);
}

/*
  nothing to do in SystemInit()
 */
void SystemInit()
{
}
#endif // PORT_LETTER

/*
  Debug UART support - USART2 TX on PB3 AF7, DMA1 CH1
  Lightweight output: no printf/vsnprintf to save flash
 */
#ifdef USE_DEBUG_UART

#include <stm32g4xx_ll_usart.h>
#include <stm32g4xx_ll_dma.h>
#include <stm32g4xx_ll_dmamux.h>

static inline void bl_debug_uart_init(void)
{
  // Enable clocks: GPIOB, USART2, DMA1
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMAMUX1);

  // PB3 as USART2_TX, AF7
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_3, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_3, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_3, LL_GPIO_AF_7);

  // USART2: 115200 8N1
  LL_USART_SetBaudRate(USART2, 160000000, LL_USART_PRESCALER_DIV1, LL_USART_OVERSAMPLING_16, 115200);
  LL_USART_SetDataWidth(USART2, LL_USART_DATAWIDTH_8B);
  LL_USART_SetStopBitsLength(USART2, LL_USART_STOPBITS_1);
  LL_USART_SetParity(USART2, LL_USART_PARITY_NONE);
  LL_USART_SetTransferDirection(USART2, LL_USART_DIRECTION_TX);
  LL_USART_EnableDMAReq_TX(USART2);
  LL_USART_Enable(USART2);

  // DMA1 Channel 1 for USART2_TX
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_1, LL_DMAMUX_REQ_USART2_TX);
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_LOW);
  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_NORMAL);
  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_BYTE);
  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_BYTE);
  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)&USART2->TDR);
}

// wait for previous DMA transfer to complete, then send new data
static inline void bl_debug_send(const char *data, uint32_t len)
{
  if (len == 0) return;
  // wait for any previous transfer
  while (LL_DMA_IsEnabledChannel(DMA1, LL_DMA_CHANNEL_1)) {
    if (LL_DMA_IsActiveFlag_TC1(DMA1)) {
      LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
      LL_DMA_ClearFlag_TC1(DMA1);
      break;
    }
  }
  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)data);
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, len);
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
}

static inline uint32_t bl_strlen(const char *s)
{
  uint32_t n = 0;
  while (s[n]) n++;
  return n;
}

// print a string (blocking wait for previous transfer)
static inline void bl_debug_print(const char *msg)
{
  bl_debug_send(msg, bl_strlen(msg));
}

// print a uint32 in decimal
static inline void bl_debug_print_u32(const char *label, uint32_t val)
{
  static char buf[32];
  uint32_t pos = 0;
  // copy label
  for (uint32_t i = 0; label[i] && pos < sizeof(buf)-12; i++) {
    buf[pos++] = label[i];
  }
  // convert value to decimal (reverse then swap)
  if (val == 0) {
    buf[pos++] = '0';
  } else {
    uint32_t start = pos;
    while (val > 0 && pos < sizeof(buf)-2) {
      buf[pos++] = '0' + (val % 10);
      val /= 10;
    }
    // reverse the digits
    for (uint32_t i = start, j = pos-1; i < j; i++, j--) {
      char tmp = buf[i]; buf[i] = buf[j]; buf[j] = tmp;
    }
  }
  buf[pos++] = '\r';
  buf[pos++] = '\n';
  bl_debug_send(buf, pos);
}

// print a uint32 in hex
static inline void bl_debug_print_hex(const char *label, uint32_t val)
{
  static char buf[32];
  uint32_t pos = 0;
  const char hex[] = "0123456789ABCDEF";
  for (uint32_t i = 0; label[i] && pos < sizeof(buf)-12; i++) {
    buf[pos++] = label[i];
  }
  buf[pos++] = '0';
  buf[pos++] = 'x';
  for (int i = 28; i >= 0; i -= 4) {
    buf[pos++] = hex[(val >> i) & 0xF];
  }
  buf[pos++] = '\r';
  buf[pos++] = '\n';
  bl_debug_send(buf, pos);
}

#endif // USE_DEBUG_UART
