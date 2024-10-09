/*
  MCU specific utility functions for the bootloader
 */
#pragma once

/*
  16k ram
 */
#define RAM_BASE 0x20000000
#define RAM_SIZE 16*1024

/*
  use 32k of flash
 */
#define BOARD_FLASH_SIZE 32


#define GPIO_PIN(n) (1U<<(n))

#define GPIO_PULL_NONE GPIO_PUPD_NONE
#define GPIO_PULL_UP GPIO_PUPD_PULLUP
#define GPIO_PULL_DOWN GPIO_PUPD_PULLDOWN

#define GPIO_OUTPUT_PUSH_PULL GPIO_OTYPE_PP

static inline void gpio_mode_set_input(uint32_t pin, uint32_t pull_up_down)
{
    gpio_mode_set(input_port, GPIO_MODE_INPUT, pull_up_down, pin);
}

static inline void gpio_mode_set_output(uint32_t pin, uint32_t output_mode)
{
    gpio_mode_set(input_port, GPIO_MODE_OUTPUT, output_mode, pin);
}

static inline void gpio_set(uint32_t pin)
{
    gpio_bit_set(input_port, pin);
}

static inline void gpio_clear(uint32_t pin)
{
    gpio_bit_reset(input_port, pin);
}

static inline bool gpio_read(uint32_t pin)
{
    return (gpio_input_port_get(input_port) & pin) != 0;
}

#define BL_TIMER TIMER16

/*
  initialise timer for 1us per tick
 */
static inline void bl_timer_init(void)
{
    rcu_periph_clock_enable(RCU_TIMER16);
    TIMER_CAR(BL_TIMER) = 0xFFFF;
    TIMER_PSC(BL_TIMER) = 71;
    timer_auto_reload_shadow_enable(BL_TIMER);
    timer_enable(BL_TIMER);
}

/*
  disable timer ready for app start
 */
static inline void bl_timer_disable(void)
{
    timer_disable(BL_TIMER);
}

static inline uint16_t bl_timer_us(void)
{
    return timer_counter_read(BL_TIMER);
}

/*
  initialise clocks
 */
static inline void bl_clock_config(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
}

static inline void bl_gpio_init(void)
{
}

/*
  return true if the MCU booted under a software reset
 */
static inline bool bl_was_software_reset(void)
{
    return (RCU_RSTSCK & RCU_RSTSCK_SWRSTF) != 0;
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

void SysTick_Handler(void)
{
}

#define RCU_MODIFY(__delay)                 \
    do {                                    \
        volatile uint32_t i;                \
        if (0 != __delay) {                 \
            RCU_CFG0 |= RCU_AHB_CKSYS_DIV2; \
            for (i = 0; i < __delay; i++) { \
            }                               \
            RCU_CFG0 |= RCU_AHB_CKSYS_DIV4; \
            for (i = 0; i < __delay; i++) { \
            }                               \
        }                                   \
    } while (0)

static void system_clock_72m_irc8m(void)
{
    uint32_t timeout = 0U;
    uint32_t stab_flag = 0U;

    /* enable IRC8M */
    RCU_CTL0 |= RCU_CTL0_IRC8MEN;

    /* wait until IRC8M is stable or the startup time is longer than
     * IRC8M_STARTUP_TIMEOUT */
    do {
        timeout++;
        stab_flag = (RCU_CTL0 & RCU_CTL0_IRC8MSTB);
    } while ((0U == stab_flag) && (IRC8M_STARTUP_TIMEOUT != timeout));

    /* if fail */
    if (0U == (RCU_CTL0 & RCU_CTL0_IRC8MSTB)) {
        while (1) {
        }
    }

    FMC_WS = (FMC_WS & (~FMC_WS_WSCNT)) | WS_WSCNT_2;

    /* AHB = SYSCLK */
    RCU_CFG0 |= RCU_AHB_CKSYS_DIV1;
    /* APB2 = AHB */
    RCU_CFG0 |= RCU_APB2_CKAHB_DIV1;
    /* APB1 = AHB */
    RCU_CFG0 |= RCU_APB1_CKAHB_DIV1;
    /* PLL = (IRC8M/2) * 18 = 72 MHz */
    RCU_CFG0 &= ~(RCU_CFG0_PLLSEL | RCU_CFG0_PLLMF);
    RCU_CFG0 |= (RCU_PLLSRC_IRC8M_DIV2 | RCU_PLL_MUL18);

    /* enable PLL */
    RCU_CTL0 |= RCU_CTL0_PLLEN;

    /* wait until PLL is stable */
    while (0U == (RCU_CTL0 & RCU_CTL0_PLLSTB)) {
    }

    /* select PLL as system clock */
    RCU_CFG0 &= ~RCU_CFG0_SCS;
    RCU_CFG0 |= RCU_CKSYSSRC_PLL;

    /* wait until PLL is selected as system clock */
    while (0U == (RCU_CFG0 & RCU_SCSS_PLL)) {
    }
}

/*!
    \brief      configure the system clock
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void system_clock_config(void)
{
    system_clock_72m_irc8m();
}

/*!
    \brief      setup the microcontroller system, initialize the system
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SystemInit(void)
{
    /* enable IRC8M */
    RCU_CTL0 |= RCU_CTL0_IRC8MEN;
    while (0U == (RCU_CTL0 & RCU_CTL0_IRC8MSTB)) {
    }

    RCU_MODIFY(0x80);
    RCU_CFG0 &= ~RCU_CFG0_SCS;
    RCU_CTL0 &= ~(RCU_CTL0_HXTALEN | RCU_CTL0_CKMEN | RCU_CTL0_PLLEN | RCU_CTL0_HXTALBPS);
    /* reset RCU */
    RCU_CFG0 &= ~(RCU_CFG0_SCS | RCU_CFG0_AHBPSC | RCU_CFG0_APB1PSC | RCU_CFG0_APB2PSC | RCU_CFG0_ADCPSC | RCU_CFG0_CKOUTSEL | RCU_CFG0_CKOUTDIV | RCU_CFG0_PLLDV);
    RCU_CFG0 &= ~(RCU_CFG0_PLLSEL | RCU_CFG0_PLLMF | RCU_CFG0_PLLMF4 | RCU_CFG0_PLLDV);
    RCU_CFG1 &= ~(RCU_CFG1_PREDV);
    RCU_CFG2 &= ~(RCU_CFG2_USART0SEL | RCU_CFG2_ADCSEL);
    RCU_CFG2 &= ~RCU_CFG2_IRC28MDIV;
    RCU_CFG2 &= ~RCU_CFG2_ADCPSC2;
    RCU_CTL1 &= ~RCU_CTL1_IRC28MEN;
    RCU_INT = 0x00000000U;

    /* configure system clock */
    system_clock_config();
}
