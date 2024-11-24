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

//#pragma GCC optimize("O0")

#include <eeprom.h>

//#define USE_ADC_INPUT      // will go right to application and ignore eeprom
//#define UPDATE_EEPROM_ENABLE

// use this to check the clock config for the MCU (with a logic
// analyser on the input pin)
//#define BOOTLOADER_TEST_CLOCK

// use this to check the string output code. When enabled
// the string HELLO_WORLD is output every 10ms
//#define BOOTLOADER_TEST_STRING

// use this to check the backup domain registers work
//#define BOOTLOADER_TEST_BKUP

// when there is no app fw yet, disable jump()
//#define DISABLE_JUMP

// optionally enable stats on serial bit-banging
//#define SERIAL_STATS

/*
  enable checking for software reset for jump.
  generally on a software reset we do want to stay in the bootloader
  if the signal pin is floating, but disabling this can be useful for
  CAN testing
*/
#define CHECK_SOFTWARE_RESET 1

/*
  enable checking for eeprom configured before jump
  disabling this can be useful for CAN development
*/
#define CHECK_EEPROM_BEFORE_JUMP 1

/*
  should we update the bootloader version in eeprom?
 */
#define UPDATE_EEPROM_ENABLE 1

#include <string.h>

#ifndef MCU_FLASH_START
#define MCU_FLASH_START 0x08000000
#endif

#ifndef FIRMWARE_RELATIVE_START
#if DRONECAN_SUPPORT
#define FIRMWARE_RELATIVE_START 0x4000
#else
#define FIRMWARE_RELATIVE_START 0x1000
#endif
#endif

#ifndef EEPROM_MAX_SIZE
#define EEPROM_MAX_SIZE 1024 // must be multiple of 256
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
#elif defined(USE_PA0)
#define input_pin         GPIO_PIN(0)
#define input_port        GPIOA
#define PIN_NUMBER        0
#define PORT_LETTER       0
#else
#error "Bootloader comms pin not defined"
#endif

static uint16_t invalid_command;

#include <blutil.h>

#if DRONECAN_SUPPORT
#include "DroneCAN/DroneCAN.h"
#include "DroneCAN/sys_can.h"
#endif

#ifndef BOARD_FLASH_SIZE
#error "must define BOARD_FLASH_SIZE"
#endif

#define PIN_CODE (PORT_LETTER << 4 | PIN_NUMBER)

/*
  currently only support 32, 64 or 128 k flash
 */
#if BOARD_FLASH_SIZE == 32
#define EEPROM_START_ADD (MCU_FLASH_START+0x7c00)
#define FLASH_SIZE_CODE 0x1f
#define ADDRESS_SHIFT 0

#elif BOARD_FLASH_SIZE == 64
#define EEPROM_START_ADD (MCU_FLASH_START+0xf800)
#define FLASH_SIZE_CODE 0x35
#define ADDRESS_SHIFT 0

#elif BOARD_FLASH_SIZE == 128
#define EEPROM_START_ADD (MCU_FLASH_START+0x1f800)
#define FLASH_SIZE_CODE 0x2B
#define ADDRESS_SHIFT 2 // addresses from the bl client are shifted 2 bits before being used
#else
#error "unsupported BOARD_FLASH_SIZE"
#endif

/*
  the devinfo structure tells the configuration client our pin code,
  flash size and device type. It can also be used by the main firmware
  to confirm we have the right eeprom address and pin code. We have 2
  32bit magic values so the main firmware can confirm the bootloader
  supports this feature
 */
#define DEVINFO_MAGIC1 0x5925e3da
#define DEVINFO_MAGIC2 0x4eb863d9

static const struct {
    uint32_t magic1;
    uint32_t magic2;
    const uint8_t deviceInfo[9];
} devinfo __attribute__((section(".devinfo"))) = {
        .magic1 = DEVINFO_MAGIC1,
        .magic2 = DEVINFO_MAGIC2,
        .deviceInfo = {'4','7','1',PIN_CODE,FLASH_SIZE_CODE,0x06,0x06,0x01,0x30}
};

typedef void (*pFunction)(void);

#define APPLICATION_ADDRESS     (uint32_t)(MCU_FLASH_START + FIRMWARE_RELATIVE_START)


#define CMD_RUN             0x00
#define CMD_PROG_FLASH      0x01
#define CMD_ERASE_FLASH     0x02
#define CMD_READ_FLASH_SIL  0x03
#define CMD_VERIFY_FLASH    0x03
#define CMD_VERIFY_FLASH_ARM 0x04
#define CMD_READ_EEPROM     0x04
#define CMD_PROG_EEPROM     0x05
#define CMD_READ_SRAM       0x06
#define CMD_READ_FLASH_ATM  0x07
#define CMD_KEEP_ALIVE      0xFD
#define CMD_SET_ADDRESS     0xFF
#define CMD_SET_BUFFER      0xFE

static uint16_t low_pin_count;
static char receiveByte;
static int count;
static char messagereceived;
static uint16_t address_expected_increment;
static int cmd;
static char eeprom_req;
static int received;

static uint8_t rxBuffer[258];
static uint8_t payLoadBuffer[256];
static char rxbyte;
static uint32_t address;

typedef union __attribute__ ((packed)) {
    uint8_t bytes[2];
    uint16_t word;
} uint8_16_u;

static uint16_t len;
static uint16_t payload_buffer_size;
static char incoming_payload_no_command;

/* USER CODE BEGIN PFP */
static void sendString(const uint8_t data[], int len);
static void receiveBuffer();
static void serialwriteChar(uint8_t data);

#define BAUDRATE      19200
#define BITTIME          52 // 1000000/BAUDRATE
#define HALFBITTIME      26 // 500000/BAUDRATE

// used for timing bytes
static uint16_t us_start;

static void bl_timer_reset(void)
{
    us_start = bl_timer_us();
}

static uint16_t bl_timer_elapsed(void)
{
    return bl_timer_us() - us_start;
}

static void delayMicroseconds(uint16_t micros)
{
    bl_timer_reset();
    while (bl_timer_elapsed() < micros) {
    }
}

/*
  jump to the application firmware
 */
static void jump()
{
#ifndef DISABLE_JUMP
#if CHECK_EEPROM_BEFORE_JUMP
    uint8_t value = *(uint8_t*)(EEPROM_START_ADD);
    if (value != 0x01) {      // check first byte of eeprom to see if its programmed, if not do not jump
	invalid_command = 0;
	return;
    }
#endif
#ifndef DISABLE_APP_HEADER_CHECKS
    /*
      first word of the app is the stack pointer - make sure that it is in range
     */
    const uint32_t *app = (uint32_t*)(MCU_FLASH_START + FIRMWARE_RELATIVE_START);
    const uint32_t ram_start = 0x20000000;
    const uint32_t ram_limit_kb = 64;
    const uint32_t ram_end = ram_start+ram_limit_kb*1024;
    if (app[0] < ram_start || app[0] > ram_end) {
	invalid_command = 0;
	return;
    }
    /*
      2nd word is the entry point of the main app. Ensure that is in range
     */
    const uint32_t flash_limit_kb = 256;
    if (app[1] < APPLICATION_ADDRESS || app[1] > APPLICATION_ADDRESS+flash_limit_kb*1024) {
	// outside a 256k range, really unlikely to be a valid
	// application, don't jump
	invalid_command = 0;
	return;
    }
#endif // DISABLE_APP_HEADER_CHECKS

#if DRONECAN_SUPPORT
    if (!DroneCAN_boot_ok()) {
        invalid_command = 0;
        return;
    }

    sys_can_disable_IRQ();
#endif

    jump_to_application();
#endif
}

/*
  16 bit CRC
 */
uint16_t crc16(const uint8_t* pBuff, uint16_t length)
{
    uint16_t ret = 0;

    for (int i = 0; i < length; i++) {
        uint8_t xb = pBuff[i];
	for (uint8_t j = 0; j < 8; j++)
	{
            if (((xb & 0x01) ^ (ret & 0x0001)) != 0) {
                ret >>= 1;
                ret ^= 0xA001;
            } else {
                ret >>= 1;
            }
            xb = xb >> 1;
	}
    }
    return ret;
}

static bool checkCrc(uint8_t* pBuff, uint16_t length)
{
    const uint16_t crcin = pBuff[length] | (pBuff[length+1]<<8);
    const uint16_t crc2 = crc16(pBuff, length);

    return crcin == crc2;
}


static void setReceive()
{
    gpio_mode_set_input(input_pin, GPIO_PULL_UP);
    received = 0;
}

static void setTransmit()
{
    // set high before we set as output to guarantee idle high
    gpio_set(input_pin);
    gpio_mode_set_output(input_pin, GPIO_OUTPUT_PUSH_PULL);

    // delay a bit to let the sender get setup for receiving
    delayMicroseconds(BITTIME);
}


static void send_ACK()
{
    setTransmit();
    serialwriteChar(0x30);             // good ack!
    setReceive();
}

static void send_BAD_ACK()
{
    setTransmit();
    serialwriteChar(0xC1);                // bad command message.
    setReceive();
}

static void send_BAD_CRC_ACK()
{
    setTransmit();
    serialwriteChar(0xC2);                // bad command message.
    setReceive();
}

static void sendDeviceInfo()
{
    setTransmit();
    sendString(devinfo.deviceInfo,sizeof(devinfo.deviceInfo));
    setReceive();
}

static bool checkAddressWritable(uint32_t address)
{
    return address >= APPLICATION_ADDRESS;
}

static void decodeInput()
{
    if (incoming_payload_no_command) {
	len = payload_buffer_size;
	if (checkCrc(rxBuffer,len)) {
	    memset(payLoadBuffer, 0, sizeof(payLoadBuffer));             // reset buffer

	    for(int i = 0; i < len; i++){
		payLoadBuffer[i]= rxBuffer[i];
	    }
	    send_ACK();
	    incoming_payload_no_command = 0;
	    return;
	}else{
	    send_BAD_CRC_ACK();
	    return;
	}
    }

    cmd = rxBuffer[0];

    if (rxBuffer[16] == 0x7d) {
	if(rxBuffer[8] == 13 && rxBuffer[9] == 66) {
	    sendDeviceInfo();
	    rxBuffer[20]= 0;

	}
	return;
    }

    if (rxBuffer[20] == 0x7d) {
	if(rxBuffer[12] == 13 && rxBuffer[13] == 66) {
	    sendDeviceInfo();
	    rxBuffer[20]= 0;
	    return;
	}

    }
    if (rxBuffer[40] == 0x7d) {
	if (rxBuffer[32] == 13 && rxBuffer[33] == 66) {
	    sendDeviceInfo();
	    rxBuffer[20]= 0;
	    return;
	}
    }

    if (cmd == CMD_RUN) {
	// starts the main app
	if((rxBuffer[1] == 0) && (rxBuffer[2] == 0) && (rxBuffer[3] == 0)) {
	    invalid_command = 101;
	}
    }

    if (cmd == CMD_PROG_FLASH) {
	len = 2;
	if (!checkCrc((uint8_t*)rxBuffer, len)) {
	    send_BAD_CRC_ACK();

	    return;
	}

	if (!checkAddressWritable(address)) {
	    send_BAD_ACK();

	    return;
	}

	if (!save_flash_nolib((uint8_t*)payLoadBuffer, payload_buffer_size,address)) {
	    send_BAD_ACK();
	} else {
	    send_ACK();
	}

	return;
    }

    if (cmd == CMD_SET_ADDRESS) {
	// command set addressinput format is: CMD, 00 , High byte
	// address, Low byte address, crclb ,crchb
	len = 4;  // package without 2 byte crc
	if (!checkCrc((uint8_t*)rxBuffer, len)) {
	    send_BAD_CRC_ACK();

	    return;
	}


	// will send Ack 0x30 and read input after transfer out callback
	invalid_command = 0;
	address = MCU_FLASH_START + ((rxBuffer[2] << 8 | rxBuffer[3]) << ADDRESS_SHIFT);
	send_ACK();

	return;
    }

    if (cmd == CMD_SET_BUFFER) {
	// for writing buffer rx buffer 0 = command byte.  command set
	// address, input , format is CMD, 00 , 00 or 01 (if buffer is 256),
	// buffer_size,
	len = 4;  // package without 2 byte crc
	if (!checkCrc((uint8_t*)rxBuffer, len)) {
	    send_BAD_CRC_ACK();

	    return;
	}

        // no ack with command set buffer;
       	if(rxBuffer[2] == 0x01){
	    payload_buffer_size = 256;                          // if nothing in this buffer
       	}else{
	    payload_buffer_size = rxBuffer[3];
        }
	incoming_payload_no_command = 1;
	address_expected_increment = 256;
        setReceive();

        return;
    }

    if (cmd == CMD_KEEP_ALIVE) {
	len = 2;
	if (!checkCrc((uint8_t*)rxBuffer, len)) {
	    send_BAD_CRC_ACK();

	    return;
	}

	setTransmit();
	serialwriteChar(0xC1);                // bad command message.
	setReceive();

	return;
    }

    if (cmd == CMD_ERASE_FLASH) {
	len = 2;
	if (!checkCrc((uint8_t*)rxBuffer, len)) {
	    send_BAD_CRC_ACK();

	    return;
	}

	if (!checkAddressWritable(address)) {
	    send_BAD_ACK();

	    return;
	}

	send_ACK();
	return;
    }

    if (cmd == CMD_READ_EEPROM) {
	eeprom_req = 1;
    }

    if (cmd == CMD_READ_FLASH_SIL) {
	// for sending contents of flash memory at the memory location set in
	// bootloader.c need to still set memory with data from set mem
	// command
	len = 2;
	if (!checkCrc((uint8_t*)rxBuffer, len)) {
	    send_BAD_CRC_ACK();

	    return;
	}

	count++;
	uint16_t out_buffer_size = rxBuffer[1];//
	if(out_buffer_size == 0){
	    out_buffer_size = 256;
	}
	address_expected_increment = 128;

	setTransmit();
	uint8_t read_data[out_buffer_size + 3];        // make buffer 3 larger to fit CRC and ACK
	memset(read_data, 0, sizeof(read_data));
        //    read_flash((uint8_t*)read_data , address);                 // make sure read_flash reads two less than buffer.
	read_flash_bin((uint8_t*)read_data , address, out_buffer_size);

        const uint16_t crc = crc16(read_data,out_buffer_size);
        read_data[out_buffer_size] = crc&0xFF;
        read_data[out_buffer_size + 1] = crc>>8;
        read_data[out_buffer_size + 2] = 0x30;
        sendString(read_data, out_buffer_size+3);

	setReceive();

	return;
    }

    setTransmit();

    serialwriteChar(0xC1);                // bad command message.
    invalid_command++;
    setReceive();
}

#ifdef SERIAL_STATS
// stats for debugging serial protocol
struct {
    uint32_t no_idle;
    uint32_t no_start;
    uint32_t bad_start;
    uint32_t bad_stop;
    uint32_t good;
} stats;
#endif

/*
  read one byte from the input pin, 19200, not inverted, one stop bit

  return false if we can't get a byte, or the byte has bad framing
 */
static bool serialreadChar()
{
    rxbyte=0;
    bl_timer_reset();

    // UART is idle high, wait for it to be in the idle state
    while (!gpio_read(input_pin)) { // wait for rx to go high
        if (bl_timer_elapsed() > 20000U) {
	    /*
	      if we don't get a command for 20ms then assume we should
	      be trying to boot the main firmware, invalid_command 101
	      triggers the jump immediately
	     */
	    invalid_command = 101;
#ifdef SERIAL_STATS
	    stats.no_idle++;
#endif
	    return false;
	}
    }

    // now we need to wait for the start bit leading edge, which is low
    bl_timer_reset();
    while (gpio_read(input_pin)) {
        if (bl_timer_elapsed() > 5*BITTIME) {
#if DRONECAN_SUPPORT
            if (DroneCAN_update()) {
                jump();
            }
#endif
            if (messagereceived) {
                // we've been waiting too long, don't allow for long gaps
                // between bytes
#ifdef SERIAL_STATS
                stats.no_start++;
#endif
                return false;
            }
        }
    }

    // wait to get the center of bit time. We want to sample at the
    // middle of each bit
    delayMicroseconds(HALFBITTIME);
    if (gpio_read(input_pin)) {
	// bad framing, we should be half-way through the start bit
	// which should still be low
#ifdef SERIAL_STATS
	stats.bad_start++;
#endif
	return false;
    }

    /*
      now sample the 8 data bits
     */
    int bits_to_read = 0;
    while (bits_to_read < 8) {
	delayMicroseconds(BITTIME);
	rxbyte = rxbyte | gpio_read(input_pin) << bits_to_read;
	bits_to_read++;
    }

    // wait till middle of stop bit, so we can check that too
    delayMicroseconds(BITTIME);
    if (!gpio_read(input_pin)) {
	// bad framing, stop bit should be high
#ifdef SERIAL_STATS
	stats.bad_stop++;
#endif
	return false;
    }

    // we got a good byte
    messagereceived = 1;
    receiveByte = rxbyte;
#ifdef SERIAL_STATS
    stats.good++;
#endif
    return true;
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

    /*
      note that we skip the delay by BITTIME for the full stop bit and
      do it in sendString() instead to ensure when sending an ACK
      immediately followed by a setReceive() on a slow MCU that we
      start on the receive as soon as possible.
    */
}


static void sendString(const uint8_t *data, int len)
{
    for(int i = 0; i < len; i++){
        serialwriteChar(data[i]);
        // for multi-byte writes we add the stop bit delay
        delayMicroseconds(BITTIME);
    }
}

static void receiveBuffer()
{
    count = 0;
    messagereceived = 0;
    memset(rxBuffer, 0, sizeof(rxBuffer));

    setReceive();

    for(uint32_t i = 0; i < sizeof(rxBuffer); i++){
	if (!serialreadChar()) {
	    break;
	}

	if(incoming_payload_no_command) {
	    if(count == payload_buffer_size+2){
		break;
	    }
	    rxBuffer[i] = rxbyte;
	    count++;
	} else {
            if(bl_timer_elapsed() > 250){
	
		count = 0;

		break;
	    } else {
		rxBuffer[i] = rxbyte;
		if(i == 257){
		    invalid_command+=20;       // needs one hundred to trigger a jump but will be reset on next set address commmand

		}
	    }
	}
    }
    if (messagereceived) {
	decodeInput();
    }
}

#ifdef UPDATE_EEPROM_ENABLE
static void update_EEPROM()
{
  if (!bl_was_software_reset()) {
    // we only update the bootloader version on a software reset to reduce the chances
    // of a brownout or spark causing a eeprom write that corrupts the settings
    return;
  }
  const uint8_t *eeprom = (const uint8_t *)EEPROM_START_ADD;
  if (BOOTLOADER_VERSION != eeprom[2]) {
    if (eeprom[2] == 0xFF || eeprom[2] == 0x00) {
      return;
    }

    // update only the bootloader version, preserve all other bytes up to EEPROM_MAX_SIZE
    uint8_t data[EEPROM_MAX_SIZE];
    memcpy(data, eeprom, EEPROM_MAX_SIZE);
    data[2] = BOOTLOADER_VERSION;

    // flash in 256 byte chunks as save_flash_nolib may not support larger chunks
    uint32_t remaining = EEPROM_MAX_SIZE;
    uint32_t addr = EEPROM_START_ADD;
    const uint8_t *p = &data[0];

    while (remaining > 0) {
      const uint32_t chunk = 256;
      save_flash_nolib(p, chunk, addr);
      p += chunk;
      addr += chunk;
      remaining -= chunk;
    }
  }
}
#endif // UPDATE_EEPROM_ENABLE

static void checkForSignal()
{
    gpio_mode_set_input(input_pin, GPIO_PULL_DOWN);
	
    delayMicroseconds(500);

    for(int i = 0 ; i < 500; i ++){
	if(!gpio_read(input_pin)){
	    low_pin_count++;
        }

	delayMicroseconds(10);
    }
    if (low_pin_count > 450) {
#if CHECK_SOFTWARE_RESET
        if (!bl_was_software_reset()) {
	    jump();
        }
#else
        jump();
#endif
    }

    gpio_mode_set_input(input_pin, GPIO_PULL_UP);
	
    delayMicroseconds(500);

    for (int i = 0 ; i < 500; i++) {
	if( !(gpio_read(input_pin))){
	    low_pin_count++;
	}else{

	}
	delayMicroseconds(10);
    }
    if (low_pin_count == 0) {
	return;           // all high while pin is pulled low, bootloader signal
    }

    low_pin_count = 0;

    gpio_mode_set_input(input_pin, GPIO_PULL_NONE);

    delayMicroseconds(500);

    for (int i = 0 ; i < 500; i ++) {
	if( !(gpio_read(input_pin))){
	    low_pin_count++;
	}

	delayMicroseconds(10);
    }

    if (low_pin_count > 0) {
	jump();
    }
}

#ifdef BOOTLOADER_TEST_CLOCK
/*
  this should provide a 2ms low followed by a 1ms high if the clock is correct
 */
static void test_clock(void)
{
    setTransmit();
    while (1) {
	gpio_clear(input_pin);
        bl_timer_reset();
        while (bl_timer_elapsed() < 2000) ;
	gpio_set(input_pin);
        bl_timer_reset();
        while (bl_timer_elapsed() < 1000) ;
    }
}
#endif // BOOTLOADER_TEST_CLOCK

#ifdef BOOTLOADER_TEST_STRING
/*
  this should send HELLO_WORLD continuously
 */
static void test_string(void)
{
    setTransmit();
    while (1) {
        delayMicroseconds(10000);
        sendString((uint8_t*)"HELLO_WORLD",11);
    }
}
#endif // BOOTLOADER_TEST_STRING


#ifdef BOOTLOADER_TEST_BKUP
/*
  test operation of backup domain registers
 */
volatile struct {
    uint32_t value;
    uint32_t fail;
} bkup;

static void test_rtc_backup(void)
{
    const uint8_t idx = 1;
    while (true) {
        bkup.value++;
        set_rtc_backup_register(idx, bkup.value);
        const uint32_t bkup_value2 = get_rtc_backup_register(idx);
        if (bkup_value2 != bkup.value) {
            bkup.fail++;
        }
        delayMicroseconds(1000);
    }
}
#endif

int main(void)
{
    bl_clock_config();
    bl_timer_init();
    bl_gpio_init();

#ifdef BOOTLOADER_TEST_CLOCK
    test_clock();
#endif
#ifdef BOOTLOADER_TEST_STRING
    test_string();
#endif
#ifdef BOOTLOADER_TEST_BKUP
    test_rtc_backup();
#endif

    checkForSignal();

    gpio_mode_set_input(input_pin, GPIO_PULL_NONE);
		
#ifdef USE_ADC_INPUT  // go right to application
    jump();
#endif

#ifdef UPDATE_EEPROM_ENABLE
     update_EEPROM();
#endif

    while (1) {
	  receiveBuffer();
	  if (invalid_command > 100) {
	      jump();
          }
#if DRONECAN_SUPPORT
          if (DroneCAN_update()) {
              jump();
          }
#endif
    }
}
