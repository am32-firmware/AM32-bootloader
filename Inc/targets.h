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
    USE_RGB_LED + RED_PORT/RED_PIN, GREEN_PORT/GREEN_PIN, BLUE_PORT/BLUE_PIN
    USE_HSE / HSE_VALUE / USE_HSE_BYPASS           - external oscillator
    RAM_LIMIT_KB                                   - max app stack-pointer RAM (default 64)

  BOARD_FLASH_SIZE=128, DRONECAN_SUPPORT=1 and AM32_MCU come automatically from
  the generated _CAN build, so they need not be repeated here.

  To disable a board without deleting it, comment out its FILE_NAME line with //
  or add DISABLE_BUILD to it (matching the main firmware repo convention).
*/

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

// 8MHz external oscillator (bypass mode)
#define USE_HSE
#undef  HSE_VALUE
#define HSE_VALUE 8000000
#define USE_HSE_BYPASS 0

// G491 has 112KB RAM; allow app stack pointer above the 64KB default
#define RAM_LIMIT_KB 112
#endif
