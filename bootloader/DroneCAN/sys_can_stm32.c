/*
  sys_can.c - MCU specific CAN code for STM32 bxCAN

  This driver is based on the ArduPilot bxCAN driver
  https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_HAL_ChibiOS/CanIface.cpp
 */

#if DRONECAN_SUPPORT && defined(MCU_L431)

#include "sys_can.h"
#include <eeprom.h>
#include <blutil.h>
#include <string.h>

typedef struct {
  volatile uint32_t TIR;
  volatile uint32_t TDTR;
  volatile uint32_t TDLR;
  volatile uint32_t TDHR;
} TxMailboxType;

typedef struct {
  volatile uint32_t RIR;
  volatile uint32_t RDTR;
  volatile uint32_t RDLR;
  volatile uint32_t RDHR;
} RxMailboxType;

typedef struct {
  volatile uint32_t FR1;
  volatile uint32_t FR2;
} FilterRegisterType;

typedef struct {
  volatile uint32_t  MCR;                 /*!< CAN master control register,         Address offset: 0x00          */
  volatile uint32_t  MSR;                 /*!< CAN master status register,          Address offset: 0x04          */
  volatile uint32_t  TSR;                 /*!< CAN transmit status register,        Address offset: 0x08          */
  volatile uint32_t  RF0R;                /*!< CAN receive FIFO 0 register,         Address offset: 0x0C          */
  volatile uint32_t  RF1R;                /*!< CAN receive FIFO 1 register,         Address offset: 0x10          */
  volatile uint32_t  IER;                 /*!< CAN interrupt enable register,       Address offset: 0x14          */
  volatile uint32_t  ESR;                 /*!< CAN error status register,           Address offset: 0x18          */
  volatile uint32_t  BTR;                 /*!< CAN bit timing register,             Address offset: 0x1C          */
  uint32_t           RESERVED0[88];       /*!< Reserved, 0x020 - 0x17F                                            */
  TxMailboxType      TxMailbox[3];        /*!< CAN Tx MailBox,                      Address offset: 0x180 - 0x1AC */
  RxMailboxType      RxMailbox[2];        /*!< CAN FIFO MailBox,                    Address offset: 0x1B0 - 0x1CC */
  uint32_t           RESERVED1[12];       /*!< Reserved, 0x1D0 - 0x1FF                                            */
  volatile uint32_t  FMR;                 /*!< CAN filter master register,          Address offset: 0x200         */
  volatile uint32_t  FM1R;                /*!< CAN filter mode register,            Address offset: 0x204         */
  uint32_t           RESERVED2;           /*!< Reserved, 0x208                                                    */
  volatile uint32_t  FS1R;                /*!< CAN filter scale register,           Address offset: 0x20C         */
  uint32_t           RESERVED3;           /*!< Reserved, 0x210                                                    */
  volatile uint32_t  FFA1R;               /*!< CAN filter FIFO assignment register, Address offset: 0x214         */
  uint32_t           RESERVED4;           /*!< Reserved, 0x218                                                    */
  volatile uint32_t  FA1R;                /*!< CAN filter activation register,      Address offset: 0x21C         */
  uint32_t           RESERVED5[8];        /*!< Reserved, 0x220-0x23F                                              */
  FilterRegisterType FilterRegister[28];  /*!< CAN Filter Register,                 Address offset: 0x240-0x31C   */
} CanType;

#define BXCAN ((volatile CanType*)0x40006400U)

/* CAN master control register */
static uint32_t MCR_INRQ =            (1U << 0); /* Bit 0: Initialization Request */
static uint32_t MCR_SLEEP =           (1U << 1); /* Bit 1: Sleep Mode Request */
static uint32_t MCR_AWUM =            (1U << 5); /* Bit 5: Automatic Wakeup Mode */
static uint32_t MCR_ABOM =            (1U << 6); /* Bit 6: Automatic Bus-Off Management */

/* CAN master status register */
static uint32_t MSR_INAK =            (1U << 0); /* Bit 0: Initialization Acknowledge */

/* CAN transmit status register */
#define TSR_RQCP0            (1U << 0) /* Bit 0: Request Completed Mailbox 0 */
#define TSR_TXOK0            (1U << 1) /* Bit 1 : Transmission OK of Mailbox 0 */
#define TSR_ALST0            (1U << 2) /* Bit 2 : Arbitration Lost for Mailbox 0 */
#define TSR_TERR0            (1U << 3) /* Bit 3 : Transmission Error of Mailbox 0 */
#define TSR_ABRQ0            (1U << 7) /* Bit 7 : Abort Request for Mailbox 0 */
#define TSR_RQCP1            (1U << 8) /* Bit 8 : Request Completed Mailbox 1 */
#define TSR_TXOK1            (1U << 9) /* Bit 9 : Transmission OK of Mailbox 1 */
#define TSR_ALST1            (1U << 10)/* Bit 10 : Arbitration Lost for Mailbox 1 */
#define TSR_TERR1            (1U << 11)/* Bit 11 : Transmission Error of Mailbox 1 */
#define TSR_ABRQ1            (1U << 15)/* Bit 15 : Abort Request for Mailbox 1 */
#define TSR_RQCP2            (1U << 16)/* Bit 16 : Request Completed Mailbox 2 */
#define TSR_TXOK2            (1U << 17)/* Bit 17 : Transmission OK of Mailbox 2 */
#define TSR_ALST2            (1U << 18)/* Bit 18: Arbitration Lost for Mailbox 2 */
#define TSR_TERR2            (1U << 19)/* Bit 19: Transmission Error of Mailbox 2 */
#define TSR_ABRQ2            (1U << 23)/* Bit 23: Abort Request for Mailbox 2 */
#define TSR_CODE_SHIFT       (24U)     /* Bits 25-24: Mailbox Code */
#define TSR_CODE_MASK        (3U << 24)
#define TSR_TME0             (1U << 26)/* Bit 26: Transmit Mailbox 0 Empty */
#define TSR_TME1             (1U << 27)/* Bit 27: Transmit Mailbox 1 Empty */
#define TSR_TME2             (1U << 28)/* Bit 28: Transmit Mailbox 2 Empty */
#define TSR_LOW0             (1U << 29)/* Bit 29: Lowest Priority Flag for Mailbox 0 */
#define TSR_LOW1             (1U << 30)/* Bit 30: Lowest Priority Flag for Mailbox 1 */
#define TSR_LOW2             (1U << 31)/* Bit 31: Lowest Priority Flag for Mailbox 2 */

/* CAN receive FIFO 0/1 registers */
static uint32_t RFR_FMP_MASK =        (3U << 0);
static uint32_t RFR_FULL =            (1U << 3); /* Bit 3: FIFO 0 Full */
static uint32_t RFR_FOVR =            (1U << 4); /* Bit 4: FIFO 0 Overrun */
static uint32_t RFR_RFOM =            (1U << 5); /* Bit 5: Release FIFO 0 Output Mailbox */

/* CAN interrupt enable register */

static uint32_t IER_TMEIE =           (1U << 0); /* Bit 0: Transmit Mailbox Empty Interrupt Enable */
static uint32_t IER_FMPIE0 =          (1U << 1); /* Bit 1: FIFO Message Pending Interrupt Enable */
static uint32_t IER_FMPIE1 =          (1U << 4); /* Bit 4: FIFO Message Pending Interrupt Enable */

/* CAN error status register */
#define ESR_LEC_SHIFT 4U    /* Bits 6-4: Last Error Code */
static uint32_t ESR_LEC_MASK =        (7U << ESR_LEC_SHIFT);

/* TX mailbox identifier register */
static uint32_t TIR_TXRQ =            (1U << 0); /* Bit 0: Transmit Mailbox Request */
static uint32_t TIR_RTR =             (1U << 1); /* Bit 1: Remote Transmission Request */
static uint32_t TIR_IDE =             (1U << 2); /* Bit 2: Identifier Extension */

/* Rx FIFO mailbox identifier register */

static uint32_t RIR_RTR =             (1U << 1); /* Bit 1: Remote Transmission Request */
static uint32_t RIR_IDE =             (1U << 2); /* Bit 2: Identifier Extension */
#define RIR_EXID_SHIFT 3U      /* Bit 3-31: Extended Identifier */
#define RIR_STID_SHIFT 21U     /* Bits 21-31: Standard Identifier */

/* CAN filter master register */
static uint32_t FMR_FINIT =           (1U << 0); /* Bit 0:  Filter Init Mode */

#define NumTxMailboxes 3
#define MaskExtID 0x1FFFFFFFU
#define MaskStdID 0x7FFU
#define NumFilters 14

/*
  send a CAN frame
 */
static bool can_send(const CanardCANFrame *frame)
{
  /*
   * Seeking for an empty slot
   */
  uint8_t txmailbox = 0xFF;
  if ((BXCAN->TSR & TSR_TME0) == TSR_TME0) {
    txmailbox = 0;
  } else if ((BXCAN->TSR & TSR_TME1) == TSR_TME1) {
    txmailbox = 1;
  } else if ((BXCAN->TSR & TSR_TME2) == TSR_TME2) {
    txmailbox = 2;
  } else {
    return false;       // No slot free
  }

  /*
   * Setting up the mailbox
   */
  volatile TxMailboxType *mb = &BXCAN->TxMailbox[txmailbox];
  if (frame->id & CANARD_CAN_FRAME_EFF) {
    // 29 bit extended addressing
    mb->TIR = ((frame->id & MaskExtID) << 3) | TIR_IDE;
  } else {
    // 11 bit addressing
    mb->TIR = ((frame->id & MaskStdID) << 21);
  }

  if (frame->id & CANARD_CAN_FRAME_RTR) {
    mb->TIR |= TIR_RTR;
  }

  mb->TDTR = frame->data_len;

  const uint32_t *d32 = (const uint32_t *)&frame->data[0];
  mb->TDHR = d32[1];
  mb->TDLR = d32[0];

  mb->TIR |= TIR_TXRQ;  // Go.

  return true;
}

static void handleTxMailboxInterrupt(uint8_t mailbox_index, bool txok)
{
  DroneCAN_processTxQueue();
}

static void pollErrorFlagsFromISR()
{
  const uint8_t lec = (uint8_t)((BXCAN->ESR & ESR_LEC_MASK) >> ESR_LEC_SHIFT);
  if (lec != 0) {
    BXCAN->ESR = 0;
  }
}

static void handleTxInterrupt(void)
{
  // TXOK == false means that there was a hardware failure
  if (BXCAN->TSR & TSR_RQCP0) {
    const bool txok = BXCAN->TSR & TSR_TXOK0;
    BXCAN->TSR = TSR_RQCP0;
    handleTxMailboxInterrupt(0, txok);
  }
  if (BXCAN->TSR & TSR_RQCP1) {
    const bool txok = BXCAN->TSR & TSR_TXOK1;
    BXCAN->TSR = TSR_RQCP1;
    handleTxMailboxInterrupt(1, txok);
  }
  if (BXCAN->TSR & TSR_RQCP2) {
    const bool txok = BXCAN->TSR & TSR_TXOK2;
    BXCAN->TSR = TSR_RQCP2;
    handleTxMailboxInterrupt(2, txok);
  }

  pollErrorFlagsFromISR();
}

volatile struct {
  uint32_t rx_overflow;
} can_stats;

static void handleRxInterrupt(uint8_t fifo_index)
{
  volatile uint32_t* const rfr_reg = (fifo_index == 0) ? &BXCAN->RF0R : &BXCAN->RF1R;
  if ((*rfr_reg & RFR_FMP_MASK) == 0) {
    return;
  }

  /*
   * Register overflow as a hardware error
   */
  if ((*rfr_reg & RFR_FOVR) != 0) {
    can_stats.rx_overflow++;
  }

  /*
   * Read the frame contents
   */
  CanardCANFrame frame = {};
  const volatile RxMailboxType *rf = &BXCAN->RxMailbox[fifo_index];

  if ((rf->RIR & RIR_IDE) == 0) {
    frame.id = MaskStdID & (rf->RIR >> 21);
  } else {
    frame.id = MaskExtID & (rf->RIR >> 3);
    frame.id |= CANARD_CAN_FRAME_EFF;
  }

  if ((rf->RIR & RIR_RTR) != 0) {
    frame.id |= CANARD_CAN_FRAME_RTR;
  }

  frame.data_len = rf->RDTR & 15;

  uint32_t *d32 = (uint32_t *)&frame.data[0];
  d32[0] = rf->RDLR;
  d32[1] = rf->RDHR;

  *rfr_reg = RFR_RFOM | RFR_FOVR | RFR_FULL;  // Release FIFO entry we just read

  DroneCAN_handleFrame(&frame);

  pollErrorFlagsFromISR();
}

/*
  get a 16 byte unique ID for this node
*/
void sys_can_getUniqueID(uint8_t id[16])
{
  const uint8_t *uidbase = (const uint8_t *)UID_BASE;
  memcpy(id, uidbase, 12);

  // put CPU ID in last 4 bytes, handy for knowing the exact MCU we are on
  const uint32_t cpuid = SCB->CPUID;
  memcpy(&id[12], &cpuid, 4);
}

/*
  interrupt handlers for CAN1
*/
void CAN1_RX0_IRQHandler(void)
{
  handleRxInterrupt(0);
}

void CAN1_RX1_IRQHandler(void)
{
  handleRxInterrupt(1);
}

void CAN1_TX_IRQHandler(void)
{
  handleTxInterrupt();
}

/*
  try to transmit a frame.
  return 1 for success, 0 for no space, -ve for failure
 */
int16_t sys_can_transmit(const CanardCANFrame* txf)
{
  return can_send(txf) ? 1 : 0;
}

/*
  check for an incoming frame
  return 1 on new frame, 0 for no frame, -ve for erro
 */
int16_t sys_can_receive(CanardCANFrame *rx_frame)
{
  // not used on STM32
  return -1;
}

/*
  disable CAN IRQs
 */
void sys_can_disable_IRQ(void)
{
  NVIC_DisableIRQ(CAN1_TX_IRQn);
  NVIC_DisableIRQ(CAN1_RX1_IRQn);
  NVIC_DisableIRQ(CAN1_RX0_IRQn);
}

/*
  enable CAN IRQs
 */
void sys_can_enable_IRQ(void)
{
  NVIC_EnableIRQ(CAN1_RX0_IRQn);
  NVIC_EnableIRQ(CAN1_RX1_IRQn);
  NVIC_EnableIRQ(CAN1_TX_IRQn);
}

/*
  init code should be small, not fast
 */
#pragma GCC optimize("Os")

static void waitMsrINakBitStateChange(bool target_state)
{
  while (true) {
    const bool state = (BXCAN->MSR & MSR_INAK) != 0;
    if (state == target_state) {
      return;
    }
  }
}

static void can_init(void)
{
#if defined(RCC_APB1ENR1_CAN1EN)
  RCC->APB1ENR1 |=  RCC_APB1ENR1_CAN1EN;
  RCC->APB1RSTR1 |=  RCC_APB1RSTR1_CAN1RST;
  RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_CAN1RST;
#else
  RCC->APB1ENR  |=  RCC_APB1ENR_CAN1EN;
  RCC->APB1RSTR |=  RCC_APB1RSTR_CAN1RST;
  RCC->APB1RSTR &= ~RCC_APB1RSTR_CAN1RST;
#endif

  /*
   * We need to silence the controller in the first order, otherwise it may interfere with the following operations.
   */
  BXCAN->MCR &= ~MCR_SLEEP; // Exit sleep mode
  BXCAN->MCR |= MCR_INRQ;   // Request init

  BXCAN->IER = 0;                  // Disable interrupts while initialization is in progress

  waitMsrINakBitStateChange(true);

  /*
   * Hardware initialization (the hardware has already confirmed initialization mode, see above)
   */
  BXCAN->MCR = MCR_ABOM | MCR_AWUM | MCR_INRQ;  // RM page 648

  // timings assuming 80MHz clock
  const uint8_t sjw = 0;
  const uint8_t bs1 = 7;
  const uint8_t bs2 = 0;
  const uint8_t prescaler = 7;

  BXCAN->BTR =
    ((sjw & 3U)  << 24) |
    ((bs1 & 15U) << 16) |
    ((bs2 & 7U)  << 20) |
    (prescaler & 1023U);

  BXCAN->IER = IER_TMEIE |   // TX mailbox empty
               IER_FMPIE0 |  // RX FIFO 0 is not empty
               IER_FMPIE1;   // RX FIFO 1 is not empty

  BXCAN->MCR &= ~MCR_INRQ;   // Leave init mode

  waitMsrINakBitStateChange(false);

  /*
   * Default filter configuration
   */
  BXCAN->FMR |= FMR_FINIT;

  BXCAN->FMR &= 0xFFFFC0F1;
  BXCAN->FMR |= NumFilters << 8;  // Slave (CAN2) gets half of the filters

  BXCAN->FFA1R = 0;                           // All assigned to FIFO0 by default
  BXCAN->FM1R = 0;                            // Indentifier Mask mode

  BXCAN->FS1R = 0x1fff;
  BXCAN->FilterRegister[0].FR1 = 0;
  BXCAN->FilterRegister[0].FR2 = 0;
  BXCAN->FA1R = 1;

  BXCAN->FMR &= ~FMR_FINIT;
}

/*
  initialise CAN hardware
 */
void sys_can_init(void)
{
  /*
    setup CAN RX and TX pins
   */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_CAN1);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);

  // assume PA11/PA12 for now
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = LL_GPIO_PIN_11 | LL_GPIO_PIN_12;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.Alternate = 9; // AF9==CAN
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  can_init();

  /*
    enable interrupt for CAN receive and transmit
  */
  NVIC_SetPriority(CAN1_RX0_IRQn, 5);
  NVIC_SetPriority(CAN1_RX1_IRQn, 5);
  NVIC_SetPriority(CAN1_TX_IRQn, 5);
}

uint32_t get_rtc_backup_register(uint8_t idx)
{
  const volatile uint32_t *bkp = &RTC->BKP0R;
  return bkp[idx];
}

void set_rtc_backup_register(uint8_t idx, uint32_t value)
{
  volatile uint32_t *bkp = &RTC->BKP0R;
  bkp[idx] = value;
}

#endif // DRONECAN_SUPPORT && defined(MCU_L431)

