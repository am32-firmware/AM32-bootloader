/*
  sitl_bl.h - runtime API for the AM32 bootloader SITL port

  The bootloader runs as a single threaded native process. Simulated
  time only advances when the bootloader reads the timer or the input
  pin, making the 19200 bit-bang timing fully deterministic; a tick
  hook run every ~50us of simulated time polls the UDP sockets and
  paces the simulation against the wall clock.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

// command line configuration
typedef struct {
    const char* eeprom_path;
    const char* flash_path; // default <eeprom>.blflash
    int input_port;
    const char* can_uri;
    float speedup; // 0 = free run
    bool bind_any;
    const char* uid;
    int initial_line; // line state at boot: -1 floating, 0 low, 1 high
    int reset_cause; // SITL_RESET_*
    bool verbose;
    char** app_argv; // exec vector after --, NULL terminated (or NULL)
} sitl_bl_config_t;

extern sitl_bl_config_t sitl_bl_cfg;

enum {
    SITL_RESET_POWER = 0,
    SITL_RESET_SOFTWARE = 1,
    SITL_RESET_WATCHDOG = 2,
};

// time
void sitl_bl_time_init(void);
uint64_t sitl_bl_time_ns(void);
void sitl_bl_advance(uint32_t ns); // advance sim time, may run the tick
uint16_t sitl_bl_timer_us(void); // 1MHz 16 bit timer, advances time

// pin model (the single signal wire)
void sitl_bl_pin_init(void);
void sitl_bl_pin_poll(void); // drain input UDP socket
uint8_t sitl_bl_pin_read(void);
void sitl_bl_pin_write(uint8_t level);
void sitl_bl_pin_mode_input(uint32_t pull); // flushes pending tx as a reply
void sitl_bl_pin_mode_output(void);

// flash/eeprom backing store mapped at 0x08000000
void sitl_bl_flash_init(void);

// process control
void sitl_bl_jump_app(void); // exec the application (or exit 42)
void sitl_bl_system_reset(void); // re-exec self with --reset-cause software
bool sitl_bl_was_software_reset(void);

// CAN poll hook (weak no-op without DRONECAN_SUPPORT)
void sitl_bl_can_poll(void);

// helpers
int sitl_bl_udp_socket(void);
uint32_t sitl_bl_bkup_read(uint8_t idx);
void sitl_bl_bkup_write(uint8_t idx, uint32_t value);
