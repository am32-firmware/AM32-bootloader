/*
  per-board bootloader configuration.

  Each board is a "#ifdef <BOARDNAME> ... #endif" block, named and formatted as
  a subset of the matching board block in the main AM32 firmware repo
  (../AM32/Inc/targets.h). The build system (make/parse_targets.py) scans this
  file and creates one bootloader target per board:

      AM32_<MCU>_BOOTLOADER_<TARGET_TAG>_CAN

  e.g. the ARK_G431_CAN block below generates AM32_G431_BOOTLOADER_ARKG4_CAN.
  Building that target passes -D<BOARDNAME> (here -DARK_G431_CAN), which
  activates this board's block. targets.h is force-included into every
  translation unit, so with no board define every block expands to nothing and
  generic builds are unaffected.

  Required fields per board:
    FILE_NAME    - quoted board name; the MCU is the token matching a known MCU
                   family (G431, L431, ...) and a trailing _CAN marks a CAN build
    TARGET_TAG   - short tag placed in the build-target name (the "pin" slot)
    USE_P<port><n> - the bit-banged comms pin (e.g. USE_PB4)

  Optional fields (see the main firmware repo for the full set):
    CAN_RX_PORT/CAN_RX_PIN, CAN_TX_PORT/CAN_TX_PIN - FDCAN1 pins (default PA11/PA12)
    CAN_TERM_PIN / CAN_TERM_POLARITY               - CAN termination pin, driven
                                                     from EEPROM byte 183
                                                     (eepromBuffer.can.term_enable)
                                                     when the bootloader starts
                                                     DroneCAN. Encode the pin with
                                                     GPIO_PORT_PIN(portnum, pinnum)
                                                     where portnum is 0=A,1=B,2=C.
    USE_RGB_LED + RED_PORT/RED_PIN, GREEN_PORT/GREEN_PIN, BLUE_PORT/BLUE_PIN
    USE_HSE / HSE_VALUE / USE_HSE_BYPASS           - external high-speed
                                                     oscillator (see the
                                                     per-MCU Mcu/<mcu>/Inc/
                                                     blutil.h bl_clock_config()
                                                     for which MCUs honour it)
    USE_LSE / USE_LSE_BYPASS                       - external low-speed
                                                     32.768 kHz crystal (L431)
    USE_MSI                                        - free-running MSI clock
                                                     (L431, no LSE/HSE)
    RAM_LIMIT_KB                                   - max app stack-pointer RAM (default 64)

  BOARD_FLASH_SIZE=128, DRONECAN_SUPPORT=1 and AM32_MCU come automatically from
  the generated _CAN build, so they need not be repeated here.

  To disable a board without deleting it, comment out its FILE_NAME line with //
  or add DISABLE_BUILD to it (matching the main firmware repo convention).
*/

/*
  encode a (port, pin) pair as a single 16-bit value, used by CAN_TERM_PIN.
  Matches the same-named macro in the main firmware's Inc/targets.h so per-board
  blocks here can be byte-identical to their main-firmware counterparts.
 */
#define GPIO_PORT_PIN(portnum, pinnum) ((portnum)<<8|(pinnum))

#ifdef ARK_G431_CAN
#define FILE_NAME "ARK_G431_CAN"   // parser: MCU=G431, CAN build
#define TARGET_TAG ARKG4           // -> AM32_G431_BOOTLOADER_ARKG4_CAN
#define USE_PB4                    // bit-banged comms pin

// FDCAN1 pins: RX PA11, TX PB9 (AF9)
#define CAN_RX_PORT GPIOA
#define CAN_RX_PIN  LL_GPIO_PIN_11
#define CAN_TX_PORT GPIOB
#define CAN_TX_PIN  LL_GPIO_PIN_9

// RGB LED on PC6 (red), PC7 (green), PC8 (blue), active low
#define USE_RGB_LED
#define RED_PORT   GPIOC
#define RED_PIN    LL_GPIO_PIN_6
#define GREEN_PORT GPIOC
#define GREEN_PIN  LL_GPIO_PIN_7
#define BLUE_PORT  GPIOC
#define BLUE_PIN   LL_GPIO_PIN_8

// CAN termination pin on PC12, active high
#define CAN_TERM_PIN GPIO_PORT_PIN(2, 12) // PC12
#define CAN_TERM_POLARITY 1

// 8MHz external oscillator (bypass mode)
#define USE_HSE
#undef  HSE_VALUE
#define HSE_VALUE 8000000
#define USE_HSE_BYPASS 0

// G491 has 112KB RAM; allow app stack pointer above the 64KB default
#define RAM_LIMIT_KB 112
#endif

#ifdef SEQURE_G431_CAN
#define FILE_NAME "SEQURE_G431_CAN" // parser: MCU=G431, CAN build
#define TARGET_TAG SEQUREG4         // -> AM32_G431_BOOTLOADER_SEQUREG4_CAN
#define USE_PA2                     // from HARDWARE_GROUP_G4_D (INPUT_PIN PA2)

// FDCAN1 pins default to PA11/PA12; SEQURE uses the defaults, so no CAN_*
// overrides are needed here.

// 8MHz external HSE crystal (USE_HSE_BYPASS 0 = crystal, not oscillator)
#define USE_HSE
#undef  HSE_VALUE
#define HSE_VALUE 8000000
#define USE_HSE_BYPASS 0
#endif

#ifdef TBS_12S_L431_CAN
#define FILE_NAME "TBS_12S_L431_CAN" // parser: MCU=L431, CAN build
#define TARGET_TAG TBS12SL4          // -> AM32_L431_BOOTLOADER_TBS12SL4_CAN
#define USE_PA2                      // from HARDWARE_GROUP_L4_C (INPUT_PIN PA2)

// bxCAN pins are fixed at PA11/PA12 in the L431 bootloader CAN driver.

// CAN termination pin on PB3, active high (see commit applying CAN_TERM)
#define CAN_TERM_PIN GPIO_PORT_PIN(1, 3) // PB3
#define CAN_TERM_POLARITY 1

// LSE-disciplined MSI clock (32.768 kHz crystal, not bypass)
#define USE_LSE
#define USE_LSE_BYPASS 0
#endif

#ifdef TBS_16S_L431_CAN
#define FILE_NAME "TBS_16S_L431_CAN" // parser: MCU=L431, CAN build
#define TARGET_TAG TBS16SL4          // -> AM32_L431_BOOTLOADER_TBS16SL4_CAN
#define USE_PA2                      // from HARDWARE_GROUP_L4_A (INPUT_PIN PA2)

// bxCAN pins are fixed at PA11/PA12 in the L431 bootloader CAN driver.

// CAN termination pin on PB3, active high
#define CAN_TERM_PIN GPIO_PORT_PIN(1, 3) // PB3
#define CAN_TERM_POLARITY 1

// LSE-disciplined MSI clock (external 32.768 kHz clock, bypass mode)
#define USE_LSE
#define USE_LSE_BYPASS 1
#endif

#ifdef TBS_12S_F415_CAN
#define FILE_NAME "TBS_12S_F415_CAN" // parser: MCU=F415, CAN build
#define TARGET_TAG TBS12SF4          // -> AM32_F415_BOOTLOADER_TBS12SF4_CAN
#define USE_PB4                      // from HARDWARE_GROUP_AT_D (INPUT_PIN PB4)

// AT32 CAN pins are fixed at PA11/PA12 (CAN1_GMUX_0000) in the F415 driver.

// CAN termination pin on PB3, active high
#define CAN_TERM_PIN GPIO_PORT_PIN(1, 3) // PB3
#define CAN_TERM_POLARITY 1
#endif

#ifdef TBS_F415_CAN
#define FILE_NAME "TBS_F415_CAN" // parser: MCU=F415, CAN build
#define TARGET_TAG TBSF4         // -> AM32_F415_BOOTLOADER_TBSF4_CAN
#define USE_PA2                  // from HARDWARE_GROUP_AT_H (INPUT_PIN PA2)

// AT32 CAN pins are fixed at PA11/PA12 (CAN1_GMUX_0000) in the F415 driver.
// Main firmware block has no CAN_TERM_PIN, so we don't set one here either.
#endif

#ifdef VIMDRONES_L431_CAN
#define FILE_NAME "VIMDRONES_L431_CAN" // parser: MCU=L431, CAN build
#define TARGET_TAG VIML4               // -> AM32_L431_BOOTLOADER_VIML4_CAN
#define USE_PA2                        // from HARDWARE_GROUP_L4_B (INPUT_PIN PA2)

// bxCAN pins are fixed at PA11/PA12 in the L431 bootloader CAN driver.
// Main firmware block has no oscillator or CAN_TERM_PIN settings; the
// bootloader defaults to HSI16 (the main firmware does too via the
// fall-through path in Mcu/l431/Src/peripherals.c).
#endif

#ifdef VIMDRONES_NANO_L431_CAN
#define FILE_NAME "VIMDRONES_NANO_L431_CAN" // parser: MCU=L431, CAN build
#define TARGET_TAG VIMNANO                  // -> AM32_L431_BOOTLOADER_VIMNANO_CAN
#define USE_PA2                             // from HARDWARE_GROUP_L4_B (INPUT_PIN PA2)

// bxCAN pins are fixed at PA11/PA12 in the L431 bootloader CAN driver.

// 24 MHz external HSE clock (main firmware doesn't set USE_HSE_BYPASS, so
// the bootloader default applies = bypass mode = external clock source).
#define USE_HSE
#undef  HSE_VALUE
#define HSE_VALUE 24000000
#endif

#ifdef VIMDRONES_S50_L431_CAN
#define FILE_NAME "VIMDRONES_S50_L431_CAN" // parser: MCU=L431, CAN build
#define TARGET_TAG VIMS50                  // -> AM32_L431_BOOTLOADER_VIMS50_CAN
#define USE_PA2                            // from HARDWARE_GROUP_L4_B (INPUT_PIN PA2)

// bxCAN pins are fixed at PA11/PA12 in the L431 bootloader CAN driver.

// 24 MHz external HSE clock (same as VIMDRONES_NANO, see above).
#define USE_HSE
#undef  HSE_VALUE
#define HSE_VALUE 24000000
#endif
