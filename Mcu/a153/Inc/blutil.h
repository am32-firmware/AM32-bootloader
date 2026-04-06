/*
  MCU specific utility functions for the bootloader
 */
/*
  based on https://github.com/AlkaMotors/AM32_Bootloader_F051/blob/main/Core/
 */
#pragma once

#ifndef BLUTIL_H_
#define BLUTIL_H_

#include "main.h"

#define NXP

// Use 64k of flash
#define BOARD_FLASH_SIZE 64

#define GPIO_PULL_NONE 2
#define GPIO_PULL_UP   1
#define GPIO_PULL_DOWN 0

#define GPIO_OUTPUT_PUSH_PULL 3	//This variable is unused for MCXA153 but needed to build main.c

#define GPIO_PIN(n) (1U<<(n))

// Link STM GPIO and PORT to NXP ones
#define GPIOA PORT0
#define GPIOB PORT1
#define GPIOC PORT2
#define GPIOD PORT3

flash_config_t s_flashDriver;

static inline void gpio_mode_set_input(uint32_t pin, uint32_t pull_up_down)
{
	//Check if pull-up/down should be turned off
	if (pull_up_down == GPIO_PULL_NONE) {
		//Disable pull-up/down and set to GPIO
		modifyReg32(&input_port->PCR[pin],
				PORT_PCR_IBE_MASK | PORT_PCR_MUX_MASK | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK,
				PORT_PCR_IBE(1) | PORT_PCR_MUX(0) | PORT_PCR_PE(0) | PORT_PCR_PS(0));
	} else {
		//Enable on pull-up/down and set to GPIO
		modifyReg32(&input_port->PCR[pin],
				PORT_PCR_IBE_MASK | PORT_PCR_MUX_MASK | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK,
				PORT_PCR_IBE(1) | PORT_PCR_MUX(0) | PORT_PCR_PE(1) | PORT_PCR_PS(pull_up_down));
	}

	//Set to input
	modifyReg32(&input_GPIO->PDDR, (1 << pin), 0);
}

static inline void gpio_mode_set_output(uint32_t pin, uint32_t output_mode)
{
	//Disable pull-up/down and set to GPIO
	modifyReg32(&input_port->PCR[pin],
			PORT_PCR_IBE_MASK | PORT_PCR_MUX_MASK | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK,
			PORT_PCR_MUX(0) | PORT_PCR_PE(0) | PORT_PCR_PS(0));

	//Set to output
	modifyReg32(&input_GPIO->PDDR, 0, (1 << pin));
}

static inline void gpio_set(uint32_t pin)
{
	//Set output to HIGH
	input_GPIO->PSOR = (1 << pin);
}

static inline void gpio_clear(uint32_t pin)
{
	//Set output to LOW
	input_GPIO->PCOR = (1 << pin);
}

static inline bool gpio_read(uint32_t pin)
{
//  return LL_GPIO_IsInputPinSet(input_port, pin);
	return ((input_GPIO->PDIR & (1 << pin)) >> pin);
}

/*
 * @brief	Initializes CTIMER2 for use as an interval timer at 1MHz
 * 			1 tick = 1 us
 */
static inline void bl_timer_init(void)
{
	//Unlock clock configuration registers access
	modifyReg32(&SYSCON->CLKUNLOCK, SYSCON_CLKUNLOCK_UNLOCK(1), 0);

	//Enable peripheral clocks
	modifyReg32(&MRCC0->MRCC_GLB_CC0, MRCC_MRCC_GLB_CC0_CTIMER2_MASK, MRCC_MRCC_GLB_CC0_CTIMER2(1));

	//Release peripherals from reset
	modifyReg32(&MRCC0->MRCC_GLB_RST0, MRCC_MRCC_GLB_RST0_CTIMER2_MASK, MRCC_MRCC_GLB_RST0_CTIMER2(1));

	//Select functional clock for CTIMER2 to FRO_12M clock
	modifyReg32(&MRCC0->MRCC_CTIMER2_CLKSEL, MRCC_MRCC_CTIMER2_CLKSEL_MUX_MASK, MRCC_MRCC_CTIMER2_CLKSEL_MUX(0));

	//Enable CTIMER2 timer
	//And set CTIMER2 clock divider to /1
	modifyReg32(&MRCC0->MRCC_CTIMER2_CLKDIV,
			MRCC_MRCC_SYSTICK_CLKDIV_DIV_MASK | MRCC_MRCC_SYSTICK_CLKDIV_HALT(1),
			MRCC_MRCC_SYSTICK_CLKDIV_DIV(0));

	//Freeze clock configuration registers access
	modifyReg32(&SYSCON->CLKUNLOCK, 0, SYSCON_CLKUNLOCK_UNLOCK(1));

	//Set CTIMER2 prescaler to /12, so clock ticks at 1MHz
	CTIMER2->PR = 11;

	//Enable CTIMER2
	modifyReg32(&CTIMER2->TCR, CTIMER_TCR_CEN_MASK, CTIMER_TCR_CEN(1));
}

/*
  disable timer ready for app start
 */
static inline void bl_timer_disable(void)
{
	//Disable CTIMER2
	modifyReg32(&CTIMER2->TCR, CTIMER_TCR_CEN_MASK, CTIMER_TCR_CEN(0));
}

static inline uint16_t bl_timer_us(void)
{
	return (uint16_t)CTIMER2->TC;
}

/*
 * @brief	Initiate system. Enables read/write/execute access for flash.
 */
static inline void System_Init(void)
{
	SCB->CPACR |= ((3UL << 0*2) | (3UL << 1*2));    /* set CP0, CP1 Full Access in Secure mode (enable PowerQuad) */
	SCB->NSACR |= ((3UL << 0) | (3UL << 10));   /* enable CP0, CP1, CP10, CP11 Non-secure Access */

	extern void *__Vectors;
	SCB->VTOR = (uint32_t) &__Vectors;

	/* Enable the LPCAC */
	SYSCON->LPCAC_CTRL |= SYSCON_LPCAC_CTRL_LPCAC_MEM_REQ_MASK;
	SYSCON->LPCAC_CTRL &= ~SYSCON_LPCAC_CTRL_DIS_LPCAC_MASK;

	/* Enable flash RWX when FLASH_ACL in IFR0 is invalid */
	if ((*((volatile const uint32_t *)(0x1000000)) == 0xFFFFFFFFU) ||
		((*((volatile const uint32_t *)(0x1000000)) == 0x59630000U) &&
		 (*((volatile const uint32_t *)(0x1000040)) == 0xFFFFFFFFU) &&
		 (*((volatile const uint32_t *)(0x1000044)) == 0xFFFFFFFFU)))
	{
		/* Enable MBC register written with GLIKEY index15 */
		GLIKEY0->CTRL_0 = 0x00060000U;
		GLIKEY0->CTRL_0 = 0x0002000FU;
		GLIKEY0->CTRL_0 = 0x0001000FU;
		GLIKEY0->CTRL_1 = 0x00290000U;
		GLIKEY0->CTRL_0 = 0x0002000FU;
		GLIKEY0->CTRL_1 = 0x00280000U;
		GLIKEY0->CTRL_0 = 0x0000000FU;

		/* Enable RWX for GLBAC0 */
		MBC0->MBC_INDEX[0].MBC_MEMN_GLBAC[0] = 0x7700U;

		/* Use GLBAC0 for all flash block */
		for (uint8_t i = 0; i < 2U; i++)
		{
			MBC0->MBC_INDEX[0].MBC_DOM0_MEM0_BLK_CFG_W[i] = 0x00000000U;
		}

		/* Disable MBC register written */
		GLIKEY0->CTRL_0 = 0x0002000FU;
	}

	/* Route the PMC bandgap buffer signal to the ADC */
	SPC0->CORELDO_CFG |= (1U << 24U);

	uint32_t status = 0;
	memset(&s_flashDriver, 0, sizeof(flash_config_t));

	//Check if init went successful
	status = FLASH_API->flash_init(&s_flashDriver);
	if (status) {
		__asm volatile ("nop");
	}

	//Enable cache function
//	modifyReg32(&SYSCON->LPCAC_CTRL, SYSCON_LPCAC_CTRL_DIS_LPCAC(1), 0);

	//Disable cache function
	modifyReg32(&SYSCON->LPCAC_CTRL, 0, SYSCON_LPCAC_CTRL_DIS_LPCAC(1));
}

/*
 * @brief 	Configures the core voltage to 1.1V. Sets the Fast Internal Reference Clock (FIRC) to 192MHz.
 * 			Sets MUX to select FIRC as MAIN_CLK. Sets system clock divider to /2, so CPU_CLK and SYSTEM_CLK are 96MHz.
 * 			Also enables SIRC so 12MHz and 1MHz can be sourced to peripherals.
 */
static inline void SystemClock_Config(void)
{
	//Set VDD_CORE voltage level to 1.1V to set it in Standard Drive mode
	modifyReg32(&SPC0->ACTIVE_CFG, SPC_ACTIVE_CFG_CORELDO_VDD_LVL_MASK, SPC_ACTIVE_CFG_CORELDO_VDD_LVL(2));

	//Set VDD_CORE drive strength to normal (1.1V)
	modifyReg32(&SPC0->ACTIVE_CFG, SPC_ACTIVE_CFG_CORELDO_VDD_DS_MASK, SPC_ACTIVE_CFG_CORELDO_VDD_DS(1));

	//Wait for the SPC to finish its transition to 1.1V, i.e. SPC is not busy
	while ((SPC0->SC & SPC_SC_BUSY_MASK) >> SPC_SC_BUSY_SHIFT) {
		//Do nothing
		__asm volatile ("nop");
	}

	//Set flash memory to support higher voltage level and frequency
	//This sets the number of additional wait-states.
	//These need to be set according to the set FIRC frequency:
	//48MHz = 0
	//64MHz = 0
	//96MHz = 1
	//192MHz = 2
	//Set to 192MHz
	modifyReg32(&FMU0->FCTRL, FMU_FCTRL_RWSC_MASK, FMU_FCTRL_RWSC(2));

	//Set SRAM to support higher voltage levels
	modifyReg32(&SPC0->SRAMCTL, SPC_SRAMCTL_VSM_MASK, SPC_SRAMCTL_VSM(2));

	//Request SRAM voltage update
	modifyReg32(&SPC0->SRAMCTL, SPC_SRAMCTL_REQ_MASK, SPC_SRAMCTL_REQ(1));

	//Wait for the SRAM voltage change to complete
	while (!((SPC0->SRAMCTL & SPC_SRAMCTL_ACK_MASK) >> SPC_SRAMCTL_ACK_SHIFT)) {
		//Do nothing
		__asm volatile ("nop");
	}

	//Clear the SRAM voltage update request
	modifyReg32(&SPC0->SRAMCTL, SPC_SRAMCTL_REQ_MASK, 0);

	//Set System clock divider to 2 to make the CPU and SYSTEM clock 96MHz (this is the max clock) divider value = DIV + 1
	modifyReg32(&SYSCON->AHBCLKDIV, SYSCON_AHBCLKDIV_DIV_MASK, SYSCON_AHBCLKDIV_DIV(1));

	/* Config FIRC */
	//Set the Fast Internal Reference Clock (FIRC) to 192MHz
	modifyReg32(&SCG0->FIRCCFG, SCG_FIRCCFG_FREQ_SEL_MASK, SCG_FIRCCFG_FREQ_SEL(7));

	//Unlock FIRC control status register
	modifyReg32(&SCG0->FIRCCSR, SCG_FIRCCSR_LK_MASK, 0);

	//Enable FRO_HF clock to peripherals
	modifyReg32(&SCG0->FIRCCSR, SCG_FIRCCSR_FIRC_FCLK_PERIPH_EN_MASK, SCG_FIRCCSR_FIRC_FCLK_PERIPH_EN(1));

	//Enable FIRC 48MHz clock to peripherals
	modifyReg32(&SCG0->FIRCCSR, SCG_FIRCCSR_FIRC_SCLK_PERIPH_EN_MASK, SCG_FIRCCSR_FIRC_SCLK_PERIPH_EN(1));

	//Set that FIRC is disabled when in deep sleep mode
	modifyReg32(&SCG0->FIRCCSR, SCG_FIRCCSR_FIRCSTEN_MASK, SCG_FIRCCSR_FIRCSTEN(0));

	//Wait for the FIRC clock source to be valid
	while (!((SCG0->FIRCCSR & SCG_FIRCCSR_FIRCVLD_MASK) >> SCG_FIRCCSR_FIRCVLD_SHIFT)) {
		//Do nothing
		__asm volatile ("nop");
	}

	//Select FIRC as MAIN_CLK clock source
	modifyReg32(&SCG0->RCCR, SCG_RCCR_SCS_MASK, SCG_RCCR_SCS(3));

	//Wait for the MAIN_CLK clock source mux to be set correctly
	while (((SCG0->CSR & SCG_CSR_SCS_MASK) >> SCG_CSR_SCS_SHIFT) != 3) {
		//Do nothing
		__asm volatile ("nop");
	}

	//Enable FIRC clock source
	modifyReg32(&SCG0->FIRCCSR, SCG_FIRCCSR_FIRCEN_MASK, SCG_FIRCCSR_FIRCEN(1));

	//Lock FIRC control status register
	modifyReg32(&SCG0->FIRCCSR, 0, SCG_FIRCCSR_LK_MASK);

	/* Config SIRC */
	//Unlock SIRC control status register
	modifyReg32(&SCG0->SIRCCSR, SCG_SIRCCSR_LK_MASK, 0);

	//Set that SIRC is disabled in deep sleep mode
	modifyReg32(&SCG0->SIRCCSR, SCG_SIRCCSR_SIRCSTEN_MASK, 0);

	//Enable SIRC clock to peripherals
	modifyReg32(&SCG0->SIRCCSR, SCG_SIRCCSR_SIRC_CLK_PERIPH_EN_MASK, SCG_SIRCCSR_SIRC_CLK_PERIPH_EN(1));

	//Wait for SIRC clock source to be valid
	while (!((SCG0->SIRCCSR & SCG_SIRCCSR_SIRCVLD_MASK) >> SCG_SIRCCSR_SIRCVLD_SHIFT)) {
		//Do nothing
		__asm volatile ("nop");
	}

	//Lock SIRC control status register
	modifyReg32(&SCG0->SIRCCSR, 0, SCG_SIRCCSR_LK_MASK);
}

/*
  initialise clocks
 */
static inline void bl_clock_config(void)
{
	System_Init();
	SystemClock_Config();
	__enable_irq();
}

/*
 * @brief 	Enables all GPIO and PORT peripherals
 */
static inline void bl_gpio_init(void)
{
	//Unlock clock configuration registers access
	modifyReg32(&SYSCON->CLKUNLOCK, SYSCON_CLKUNLOCK_UNLOCK(1), 0);

	//Enable PORT peripheral clocks
	MRCC0->MRCC_GLB_CC0_SET = MRCC_MRCC_GLB_CC0_PORT0(1);
	MRCC0->MRCC_GLB_CC0_SET = MRCC_MRCC_GLB_CC0_PORT1(1);
	MRCC0->MRCC_GLB_CC0_SET = MRCC_MRCC_GLB_CC0_PORT2(1);
	MRCC0->MRCC_GLB_CC1_SET = MRCC_MRCC_GLB_CC1_PORT3(1);

	//Enable GPIO peripheral clocks
	MRCC0->MRCC_GLB_CC1_SET = MRCC_MRCC_GLB_CC1_GPIO0(1);
	MRCC0->MRCC_GLB_CC1_SET = MRCC_MRCC_GLB_CC1_GPIO1(1);
	MRCC0->MRCC_GLB_CC1_SET = MRCC_MRCC_GLB_CC1_GPIO2(1);
	MRCC0->MRCC_GLB_CC1_SET = MRCC_MRCC_GLB_CC1_GPIO3(1);

	//Enable INPUTMUX peripheral clock
	MRCC0->MRCC_GLB_CC0_SET = MRCC_MRCC_GLB_RST0_INPUTMUX0(1);

	//Release PORT peripherals from reset
	MRCC0->MRCC_GLB_RST0_SET = MRCC_MRCC_GLB_RST0_PORT0(1);
	MRCC0->MRCC_GLB_RST0_SET = MRCC_MRCC_GLB_RST0_PORT1(1);
	MRCC0->MRCC_GLB_RST0_SET = MRCC_MRCC_GLB_RST0_PORT2(1);
	MRCC0->MRCC_GLB_RST1_SET = MRCC_MRCC_GLB_RST1_PORT3(1);

	//Release GPIO peripherals from reset
	MRCC0->MRCC_GLB_RST1_SET = MRCC_MRCC_GLB_RST1_GPIO0(1);
	MRCC0->MRCC_GLB_RST1_SET = MRCC_MRCC_GLB_RST1_GPIO1(1);
	MRCC0->MRCC_GLB_RST1_SET = MRCC_MRCC_GLB_RST1_GPIO2(1);
	MRCC0->MRCC_GLB_RST1_SET = MRCC_MRCC_GLB_RST1_GPIO3(1);

	//Release INPUTMUX peripheral from reset
	MRCC0->MRCC_GLB_RST0_SET = MRCC_MRCC_GLB_RST0_INPUTMUX0(1);

	//Freeze clock configuration registers access
	modifyReg32(&SYSCON->CLKUNLOCK, 0, SYSCON_CLKUNLOCK_UNLOCK(1));
}

/*
  return true if the MCU booted under a software reset
 */
static inline bool bl_was_software_reset(void)
{
	return ((CMC->SRS & CMC_SRS_SW_MASK) >> CMC_SRS_SW_SHIFT);
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

#endif /* BLUTIL_H_ */
