/*
  MCU specific utility functions for the bootloader
 */
/*
  based on https://github.com/AlkaMotors/AM32_Bootloader_F051/blob/main/Core/
 */
#pragma once

/*
  8k ram
 */
#define RAM_BASE 0x20000000
#define RAM_SIZE 8*1024

/*
  use 32k of flash
 */
#define BOARD_FLASH_SIZE 1024

#define GPIO_PIN(n) (1U<<(n))

#define GPIO_PULL_NONE LL_GPIO_PULL_NO
#define GPIO_PULL_UP   LL_GPIO_PULL_UP
#define GPIO_PULL_DOWN LL_GPIO_PULL_DOWN

#define GPIO_OUTPUT_PUSH_PULL LL_GPIO_OUTPUT_PUSHPULL

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

#define BL_TIMER TIM2

/*
  initialise timer for 1us per tick
 */
static inline void bl_timer_init(void)
{
    /* Peripheral clock enable */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
    TIM2->PSC = 63;
    // TIM2->PSC = 31;
    TIM2->CR1 |= TIM_CR1_CEN;
    TIM2->EGR |= TIM_EGR_UG;

}

/*
  disable timer ready for app start
 */
static inline void bl_timer_disable(void)
{
    LL_TIM_DeInit(BL_TIMER);
}

static inline uint32_t bl_timer_us(void)
{
    return TIM2->CNT;
}

static inline void bl_timer_reset(void)
{
    TIM2->CNT = 0;
}

// configure the MPU so that the EEPROM
// does not throw a fault on access
static inline void bl_mpu_config(void)
{
    LL_MPU_Disable();
    LL_MPU_ConfigRegion(
        LL_MPU_REGION_NUMBER0,
        LL_MPU_REGION_ALL_RW,
        LL_MPU_ATTRIBUTES_NUMBER0,
        // 0x0900a800,
        // 0x0900c000
        EEPROM_BASE,
        EEPROM_BASE + EEPROM_PAGE_SIZE
    );
    LL_MPU_Enable(LL_MPU_CTRL_PRIVILEGED_DEFAULT);
}

static inline void bl_hsi_config(void)
{

}

static inline void bl_flash_enable_prefetch(void)
{
    // enable prefetch buffer
    FLASH->ACR |= FLASH_ACR_PRFTEN;
}


static inline void bl_clock_config(void)
{
    bl_flash_enable_prefetch(); 
    mpu_config();

    // // wait for any ongoing cache invalidation
    // while (ICACHE->CR & ICACHE_SR_BUSYF);
    // // enable icache miss monitor, hit monitor, and icache itself
    // ICACHE->CR |= ICACHE_CR_MISSMEN | ICACHE_CR_HITMEN | ICACHE_CR_EN;

    // LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);

    // LL_RCC_HSI_Enable();

    // /* Wait till HSI is ready */
    // while (LL_RCC_HSI_IsReady() != 1) ;
    // LL_RCC_HSI_SetCalibTrimming(16);
    // LL_RCC_HSI14_Enable();

    // /* Wait till HSI14 is ready */
    // while (LL_RCC_HSI14_IsReady() != 1) ;
    // LL_RCC_HSI14_SetCalibTrimming(16);
    // LL_RCC_LSI_Enable();

    // /* Wait till LSI is ready */
    // while (LL_RCC_LSI_IsReady() != 1) ;
    // LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_12);
    // LL_RCC_PLL_Enable();

    // /* Wait till PLL is ready */
    // while (LL_RCC_PLL_IsReady() != 1) ;
    // LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    // LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
    // LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

    // /* Wait till System clock is ready */
    // while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) ;
    // LL_Init1msTick(48000000);
    // LL_SetSystemCoreClock(48000000);
}

static inline void bl_gpio_init(void)
{
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);

    /**/
    GPIO_InitStruct.Pin = input_pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    LL_GPIO_Init(input_port, &GPIO_InitStruct);
}

/*
  return true if the MCU booted under a software reset
 */
static inline bool bl_was_software_reset(void)
{
    return (RCC->RSR & RCC_RSR_SFTRSTF) != 0;
}

/*
  no need for any action in SystemInit
 */
void SystemInit()
{
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

    // setup sp, msp and jump
    asm volatile(
        "mov sp, %0	\n"
        "msr msp, %0	\n"
        "bx	%1	\n"
	: : "r"(stack_top), "r"(JumpAddress) :);
}
