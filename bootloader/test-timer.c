/*
  bootloader for AM32 ESC firmware

  based on https://github.com/AlkaMotors/AT32F421_AM32_Bootloader
 */
#include <main.h>

#include <version.h>

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>

//#pragma GCC optimize("O0")

#include <eeprom.h>

#ifndef MCU_FLASH_START
#define MCU_FLASH_START 0x08000000
#endif

#ifndef FIRMWARE_RELATIVE_START
#define FIRMWARE_RELATIVE_START 0x1000
#endif



#ifdef USE_PA2
#define input_pin        GPIO_PIN(2)
#define input_port       GPIOA
#define PIN_NUMBER       2
#define PORT_LETTER      0
#elif defined(USE_PB4)
#define input_pin         GPIO_PIN(4)
#define input_port        GPIOB
#define PIN_NUMBER        4
#define PORT_LETTER       1
#elif defined(USE_PA15)
#define input_pin         GPIO_PIN(15)
#define input_port        GPIOA
#define PIN_NUMBER        15
#define PORT_LETTER       0
#elif defined(USE_PA6)
#define input_pin         GPIO_PIN(6)
#define input_port        GPIOA
#define PIN_NUMBER        6
#define PORT_LETTER       0
#elif defined(USE_PA5)
#define input_pin         GPIO_PIN(5)
#define input_port        GPIOA
#define PIN_NUMBER        5
#define PORT_LETTER       0
#elif defined(USE_PA0)
#define input_pin         GPIO_PIN(0)
#define input_port        GPIOA
#define PIN_NUMBER        0
#define PORT_LETTER       0
#else
#error "Bootloader comms pin not defined"
#endif


#include <blutil.h>


/*
  currently only support 32, 64 or 128 k flash
 */
#if BOARD_FLASH_SIZE == 32
#define EEPROM_START_ADD (MCU_FLASH_START+0x7c00)
static uint8_t deviceInfo[9] = {0x34,0x37,0x31,0x00,0x1f,0x06,0x06,0x01,0x30};
#define ADDRESS_SHIFT 0

#elif BOARD_FLASH_SIZE == 64
#define EEPROM_START_ADD (MCU_FLASH_START+0xf800)
static uint8_t deviceInfo[9] = {0x34,0x37,0x31,0x64,0x35,0x06,0x06,0x01,0x30};
#define ADDRESS_SHIFT 0

#elif BOARD_FLASH_SIZE == 128
static uint8_t deviceInfo[9] = {0x34,0x37,0x31,0x64,0x2B,0x06,0x06,0x01,0x30};
#define EEPROM_START_ADD (MCU_FLASH_START+0x1f800)
#define ADDRESS_SHIFT 2 // addresses from the bl client are shifted 2 bits before being used

#elif BOARD_FLASH_SIZE == 1024
#define EEPROM_PAGE_SIZE (0x1800)
#define EEPROM_BASE (0x09000000)
#define EEPROM_PAGE (7)
// eeprom address is 0x900a800 with one EDATA high cycle (6kB) page enabled
#define EEPROM_START_ADD (EEPROM_BASE + EEPROM_PAGE*EEPROM_PAGE_SIZE)
// static uint8_t deviceInfo[9] = {0x34,0x37,0x31,0x64,0x35,0x06,0x06,0x01,0x30};
#define ADDRESS_SHIFT 0

#else
#error "unsupported BOARD_FLASH_SIZE"
#endif

void bl_timer_enable_clock()
{

	/* Peripheral clock enable */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
}

void bl_timer_configure_gpio()
{
	LL_GPIO_InitTypeDef gpio = {0};

	gpio.Pin = LL_GPIO_PIN_5;
	gpio.Mode = LL_GPIO_MODE_ALTERNATE; // put gpio under control of the timer
	gpio.Speed = LL_GPIO_SPEED_FREQ_HIGH; // 3/4 levels (VERY_HIGH) is higher
	gpio.OutputType = LL_GPIO_OUTPUT_PUSHPULL; // default
	gpio.Pull = LL_GPIO_PULL_UP;
	gpio.Alternate = LL_GPIO_AF_1;

	LL_GPIO_Init(GPIOA, &gpio);
}
void bl_timer_test_output()
{

	// modify capture compare mode register (CCMR)
	// set channel 1 mode to PWM1 (0b0110)
	TIM2->CCMR1 |= 0b0110 << 0;

	// modify capture compare enable register (CCER)
	// enable channel 1 capture compare
	TIM2->CCER |= TIM_CCER_CC1E;

	TIM2->ARR = 20000;
	TIM2->CCR1 = 10000; // preloaded
	TIM2->EGR |= TIM_EGR_UG; // load the preloaded registers
	TIM2->CR1 |= TIM_CR1_CEN; // enable the timer


}
// test timer output using pwm outpu configuration
// on gpio
// this can help with testing software serial, where
// flash prefetch + waitstate settings, and icache
// settings may matter
int main(void)
{
    bl_clock_config();
    // bl_timer_init();
	bl_timer_configure_gpio();
	bl_timer_enable_clock();
	bl_timer_test_output();

}
