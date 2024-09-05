/*
  bootloader for AM32 ESC firmware

  based on https://github.com/AlkaMotors/AT32F421_AM32_Bootloader
 */
#include <main.h>
#include <stdio.h>

#include <version.h>

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdio.h>

#pragma GCC optimize("O0")

#include <string.h>

#define GPIO_PORT_TYPE typeof(GPIOA)

static GPIO_PORT_TYPE input_port;
static uint32_t input_pin;

#define MCU_FLASH_START 0x08000000
#define FIRMWARE_RELATIVE_START 0x1000

#define BAUDRATE      2400
#define BITTIME          (1000000/BAUDRATE)
#define HALFBITTIME      (500000/BAUDRATE)

#include <blutil.h>

static void setTransmit()
{
    gpio_set(input_pin);
    gpio_mode_set_output(input_pin, GPIO_OUTPUT_PUSH_PULL);
}

static void delayMicroseconds(uint32_t micros)
{
    while (micros > 0) {
        uint32_t us = micros>10000?10000:micros;
        bl_timer_reset();
        while (bl_timer_us() < us) ;
        micros -= us;
    }
}

static void serialwriteChar(uint8_t data)
{
    // start bit is low
    gpio_clear(input_pin);
    delayMicroseconds(BITTIME);

    // send data bits
    uint8_t bits_written = 0;
    while (bits_written < 8) {
	if (data & 0x01) {
	    gpio_set(input_pin);
	} else {
	    // GPIO_BC(input_port) = input_pin;
	    gpio_clear(input_pin);
	}
	bits_written++;
	data = data >> 1;
	delayMicroseconds(BITTIME);
    }

    // send stop bit
    gpio_set(input_pin);
    delayMicroseconds(BITTIME);
}

static void sendString(const uint8_t *data, int len)
{
    for(int i = 0; i < len; i++){
	serialwriteChar(data[i]);
    }
}

#define ARRAY_SIZE(x) (sizeof(x)/sizeof(x[0]))

static bool is_debug(uint8_t portnum, uint32_t pin) {
    return portnum == 0 && (pin == 13 || pin == 14);
}

static struct {
    const char *pname;
    GPIO_PORT_TYPE port;
} ports[] = {
        { "PA", GPIOA },
        { "PB", GPIOB },
};

static void pin_scanner(void)
{
    for (uint8_t i=0; i< ARRAY_SIZE(ports); i++) {
        for (uint8_t p=0; p< 16; p++) {
            if (is_debug(i, p)) {
                continue;
            }

            for (uint8_t i2=0; i2< ARRAY_SIZE(ports); i2++) {
                for (uint8_t p2=0; p2< 16; p2++) {
                    if (is_debug(i2, p2)) {
                        continue;
                    }
                    if (i == i2 && p == p2) {
                        continue;
                    }
                    input_port = ports[i].port;
                    input_pin = GPIO_PIN(p);

                    /*
                      write the pin combination to the proposed signal pin
                     */
                    char str[12];
                    str[0] = '[';
                    strncpy(&str[1], ports[i].pname, sizeof(str)-1);
                    str[3] = '0' + (p / 10);
                    str[4] = '0' + (p % 10);
                    str[5] = ':';
                    strncpy(&str[6], ports[i2].pname, sizeof(str)-6);
                    str[8] = '0' + (p2 / 10);
                    str[9] = '0' + (p2 % 10);
                    str[10] = ']';
                    str[11] = 0;

                    setTransmit();
                    gpio_set(input_pin);
                    delayMicroseconds(1000);

                    sendString((const uint8_t *)str, strlen(str));

                    gpio_mode_set_input(input_pin, GPIO_PULL_NONE);

                    delayMicroseconds(200);

                    /*
                      oscillate the 2nd pin 10 times, 500us high, 1000us low
                     */
                    input_port = ports[i2].port;
                    input_pin = GPIO_PIN(p2);
                    setTransmit();
                    delayMicroseconds(200);
                    for (uint16_t c=0;c<5;c++) {
                        gpio_set(input_pin);
                        delayMicroseconds(500);
                        gpio_clear(input_pin);
                        delayMicroseconds(1000);
                    }
                    gpio_mode_set_input(input_pin, GPIO_PULL_NONE);
                }
            }
        }
    }
}

int main(void)
{
    bl_clock_config();
    bl_timer_init();

    // give time for debugger to attach
    delayMicroseconds(3000000);

    // setup all as float input
    for (uint8_t i=0; i< ARRAY_SIZE(ports); i++) {
        for (uint8_t p=0; p< 16; p++) {
            if (is_debug(i, p)) {
                continue;
            }
            input_port = ports[i].port;
            input_pin = GPIO_PIN(p);
            bl_gpio_init();
            gpio_mode_set_input(input_pin, GPIO_PULL_NONE);
        }
    }
    
    while (true) {
        pin_scanner();
    }
    return 0;
}
