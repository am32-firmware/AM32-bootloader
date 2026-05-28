/*
  MCU specific utility functions for the bootloader
 */
#pragma once

/*
  64k ram
 */
#define RAM_BASE 0x20000000
#define RAM_SIZE 64*1024

/*
  we have 256k of flash, but only use 64k for now
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

  TIM_InitStruct.Prescaler = 79;
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
  initialise clocks. Mirrors the oscillator choice in the main AM32 firmware's
  Mcu/l431/Src/peripherals.c so per-board #define blocks (USE_HSE/USE_LSE/USE_MSI)
  drive the bootloader and the application identically:

    USE_HSE       external high-speed crystal/oscillator. HSE_VALUE must be
                  8 / 16 / 24 MHz. USE_HSE_BYPASS=0 selects crystal, otherwise
                  external oscillator.
    USE_LSE       MSI clock disciplined by an external 32.768 kHz LSE; RTC
                  also runs from LSE. USE_LSE_BYPASS=1 selects an external
                  clock source instead of a crystal.
    USE_MSI       free-running MSI (no LSE discipline).
    (default)     HSI16, the no-external-oscillator path.

  All branches converge on an 80 MHz PLL output.
 */
static inline void bl_clock_config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
  while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_4) ;
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  while (LL_PWR_IsActiveFlag_VOS() != 0) ;

  // backup domain access is needed both to change the RTC clock source and
  // to use the BKP registers for the bootloader <-> firmware handoff.
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  LL_PWR_EnableBkUpAccess();

#ifdef USE_HSE
  /*
    External high-speed crystal / oscillator. USE_HSE_BYPASS=0 selects
    crystal mode; anything else selects bypass-from-external-oscillator.
   */
#if defined(USE_HSE_BYPASS) && (USE_HSE_BYPASS == 0)
  LL_RCC_HSE_DisableBypass();
#else
  LL_RCC_HSE_EnableBypass();
#endif
  LL_RCC_HSE_Enable();
  while (LL_RCC_HSE_IsReady() != 1) ;
#if HSE_VALUE == 24000000
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_3, 20, LL_RCC_PLLR_DIV_2);
#elif HSE_VALUE == 16000000
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_2, 20, LL_RCC_PLLR_DIV_2);
#elif HSE_VALUE == 8000000
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_1, 20, LL_RCC_PLLR_DIV_2);
#else
#error "Unsupported HSE_VALUE"
#endif

#elif defined(USE_LSE)
  /*
    LSE-disciplined MSI: a 32.768 kHz LSE drives the MSI's PLL-mode trim so
    the MSI is precise enough for clock-sensitive peripherals. RTC also runs
    from LSE. The PLL still sources from MSI (not from LSE directly).

    The backup-domain reset and full LSE re-configuration are only safe on
    cold boot. On warm boot (after the app has invoked NVIC_SystemReset for
    a DroneCAN firmware-update handoff) the LSE is already running AND the
    BKP registers carry the handoff magic + path that DroneCAN.c reads in
    DroneCAN_Startup. Resetting the backup domain wipes that state and
    breaks the OTA flow. Skip the reset/reconfigure path when LSE is
    already ready: the app and bootloader want the same LSE configuration,
    and we want to preserve whatever the app wrote to the BKP registers.
   */
  LL_RCC_MSI_Enable();
  while (LL_RCC_MSI_IsReady() != 1) ;
  LL_RCC_MSI_DisablePLLMode();
  if (LL_RCC_LSE_IsReady() != 1) {
    LL_RCC_ForceBackupDomainReset();
    LL_RCC_ReleaseBackupDomainReset();
    LL_RCC_LSE_SetDriveCapability(LL_RCC_LSEDRIVE_HIGH);
    #if defined(USE_LSE_BYPASS) && USE_LSE_BYPASS
      LL_RCC_LSE_EnableBypass();
    #else
      LL_RCC_LSE_DisableBypass();
    #endif
    LL_RCC_LSE_Enable();
    while (LL_RCC_LSE_IsReady() != 1) ;
    LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSE);
    LL_RCC_EnableRTC();
  }
  LL_RCC_MSI_EnablePLLMode();
  LL_RCC_MSI_EnableRangeSelection();
  LL_RCC_MSI_SetRange(LL_RCC_MSIRANGE_6);
  LL_RCC_MSI_SetCalibTrimming(0);
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_MSI, LL_RCC_PLLM_DIV_1, 40, LL_RCC_PLLR_DIV_2);

#elif defined(USE_MSI)
  /*
    Free-running MSI (no LSE discipline). Useful when neither LSE nor HSE
    is wired but the MSI accuracy is acceptable.
   */
  LL_RCC_MSI_Enable();
  while (LL_RCC_MSI_IsReady() != 1) ;
  LL_RCC_MSI_EnableRangeSelection();
  LL_RCC_MSI_SetRange(LL_RCC_MSIRANGE_6);
  LL_RCC_MSI_SetCalibTrimming(0);
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_MSI, LL_RCC_PLLM_DIV_1, 40, LL_RCC_PLLR_DIV_2);

#else
  /*
    Default: HSI16. The PLL produces 80 MHz from the internal 16 MHz HSI.
   */
  LL_RCC_HSI_Enable();
  while (LL_RCC_HSI_IsReady() != 1) ;
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_2, 20, LL_RCC_PLLR_DIV_2);
#endif

#ifndef USE_LSE
  /*
    Non-LSE paths still need the RTC running so the DroneCAN bootloader can
    hand off via the BKP registers. LSI is the natural source there. Cheap
    enough to enable unconditionally for non-CAN builds too.
   */
  LL_RCC_LSI_Enable();
  while (LL_RCC_LSI_IsReady() != 1) ;
  LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSI);
  LL_RCC_EnableRTC();
#endif

  LL_RCC_PLL_EnableDomain_SYS();
  LL_RCC_PLL_Enable();
  while (LL_RCC_PLL_IsReady() != 1) ;
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) ;

  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
}

#ifdef PORT_LETTER
static inline void bl_gpio_init(void)
{
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  LL_GPIO_ResetOutputPin(input_port, input_pin);
}

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

#ifdef FIRMWARE_RELATIVE_START
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
#endif // FIRMWARE_RELATIVE_START

void SystemInit(void)
{
#if defined(USER_VECT_TAB_ADDRESS)
  /* Configure the Vector Table location -------------------------------------*/
  SCB->VTOR = VECT_TAB_BASE_ADDRESS | VECT_TAB_OFFSET;
#endif

  /* FPU settings ------------------------------------------------------------*/
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
  SCB->CPACR |= ((3UL << 20U)|(3UL << 22U));  /* set CP10 and CP11 Full Access */
#endif

  /* Reset the RCC clock configuration to the default reset state ------------*/
  /* Set MSION bit */
  RCC->CR |= RCC_CR_HSION;

  /* Reset CFGR register */
  RCC->CFGR = 0x00000000U;

  /* Reset PLLCFGR register */
  RCC->PLLCFGR = 0x00001000U;

  /* Reset HSEBYP bit */
  RCC->CR &= 0xFFFBFFFFU;

  /* Disable all interrupts */
  RCC->CIER = 0x00000000U;

  // enable the backup domain
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  LL_PWR_EnableBkUpAccess();
}
#endif // PORT_LETTER
