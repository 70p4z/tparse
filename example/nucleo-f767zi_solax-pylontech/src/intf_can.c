#include "main.h"


#ifdef CAN3
#define CAN_INTF_COUNT 2
#else
#define CAN_INTF_COUNT 1
#endif

uint32_t can_get_idx(CAN_TypeDef* _CAN) {
#ifdef CAN3
  if ( _CAN == CAN3) {
    return 1;
  }
#endif // CAN3
  return 0;
}

struct {
  size_t size;
  struct {
    uint32_t id;
    size_t id_bitlen;
    uint8_t data[8];
    size_t data_len;
  } entries[CAN_FIFO_RX_ENTRY_COUNT];
} G_can_fifo_rx[CAN_INTF_COUNT];


size_t can_tx(CAN_TypeDef* _CAN, uint32_t id, size_t id_bitlen, uint8_t *frame, size_t frame_len) {
  uint8_t retry_count=3;
  // no free slot?
again:
  if (!(_CAN->TSR&(CAN_TSR_TME0|CAN_TSR_TME1|CAN_TSR_TME2))) {
    // try to recover any previous transmission error
    if (_CAN->ESR&0xFF) {
      _CAN->TSR |= CAN_TSR_ABRQ0|CAN_TSR_ABRQ1|CAN_TSR_ABRQ2;
      // until at least one mailbox is empty
      while((_CAN->TSR&(CAN_TSR_ABRQ0|CAN_TSR_ABRQ1|CAN_TSR_ABRQ2)) 
            == (CAN_TSR_ABRQ0|CAN_TSR_ABRQ1|CAN_TSR_ABRQ2));
      if (retry_count-->0) {
        goto again;
      }
      return -5;
    }
    if (!(_CAN->TSR&(CAN_TSR_TME0|CAN_TSR_TME1|CAN_TSR_TME2))) { 
      return -1;
    }
  }
  uint8_t slot = (_CAN->TSR & CAN_TSR_CODE) >>CAN_TSR_CODE_Pos;
  if (slot == 3) {
    return -2;
  }

  switch(id_bitlen) {
    case 11:
      _CAN->sTxMailBox[slot].TIR = (id << 21);
      break;
    case 29:
      _CAN->sTxMailBox[slot].TIR = (id << 3) | CAN_TI0R_IDE;
      break;
    default:
      return -3;
  }
  if (frame_len > 8) {
    return -4;
  }
  _CAN->sTxMailBox[slot].TDTR = frame_len;
  // copy data
  uint32_t d[2] = {0};
  uint32_t i=0;
  while (i<frame_len) {
    d[i/4] |= (frame[i]&0xFF)<<((i%4)*8);
    i++;
  }
  _CAN->sTxMailBox[slot].TDLR = d[0];
  _CAN->sTxMailBox[slot].TDHR = d[1];

  // enable transmission of that slot
  _CAN->sTxMailBox[slot].TIR |= CAN_TI0R_TXRQ;
  return frame_len;
}

static size_t can_rx(CAN_TypeDef* _CAN, uint32_t * id, size_t * id_bitlen, uint8_t* frame, size_t frame_max_len) {
  uint32_t slot;
  if (_CAN->RF0R&(CAN_RF0R_FMP0_Msk)) {
    slot = 0;
  }
  else if (_CAN->RF1R&(CAN_RF1R_FMP1_Msk)) {
    slot = 1;
  }
  else {
    if (id_bitlen) {
      *id_bitlen = 0;
    }
    return 0;
  }

  if (id_bitlen) {
    *id_bitlen = _CAN->sFIFOMailBox[slot].RIR & CAN_RI0R_IDE?29:11;
    if (id) {
      *id = _CAN->sFIFOMailBox[slot].RIR >> ( 32 - *id_bitlen);
    }
  }
  frame_max_len = MIN(frame_max_len, _CAN->sFIFOMailBox[slot].RDTR & CAN_RDT0R_DLC_Msk);

  if (frame) {
    uint32_t d[2] = {0};
    uint32_t i=0;
    d[0] = _CAN->sFIFOMailBox[slot].RDLR;
    d[1] = _CAN->sFIFOMailBox[slot].RDHR;
    while (i<frame_max_len) {
      frame[i] = d[i/4] >> ((i%4)*8);
      i++;
    }
  }
  else {
    frame_max_len = 0;
  }

  // consume frame
  switch(slot) {
    case 0:
      _CAN->RF0R |= CAN_RF0R_RFOM0;
      break;
    case 1:
      _CAN->RF1R |= CAN_RF1R_RFOM1;
      break;
  }
  return frame_max_len; 
}

void CAN_IRQHandler_init(void) {
  //__disable_irq();
  memset(G_can_fifo_rx, 0, sizeof (G_can_fifo_rx));
  //__enable_irq();
}

void CANx_RXy_IRQHandler(CAN_TypeDef* _CAN) {
  uint32_t can_idx = can_get_idx(_CAN);
  if (G_can_fifo_rx[can_idx].size >= CAN_FIFO_RX_ENTRY_COUNT) {
    // consume received frame, no FIFO space to store it
    can_rx(_CAN, NULL, NULL, NULL, 0);
    // discard
    return;
  }
  // consume in 0, append at the end
  size_t entry = G_can_fifo_rx[can_idx].size++;
  G_can_fifo_rx[can_idx].entries[entry].data_len =
    can_rx(_CAN,
         &G_can_fifo_rx[can_idx].entries[entry].id,
         &G_can_fifo_rx[can_idx].entries[entry].id_bitlen,
         G_can_fifo_rx[can_idx].entries[entry].data,
         8);
}

void CAN1_RX0_IRQHandler(void) {
  CANx_RXy_IRQHandler(CAN1);
}

void CAN1_RX1_IRQHandler(void) {
  CANx_RXy_IRQHandler(CAN1);
}

void CAN3_RX0_IRQHandler(void) {
  CANx_RXy_IRQHandler(CAN3);
}

void CAN3_RX1_IRQHandler(void) {
  CANx_RXy_IRQHandler(CAN3);
}

size_t can_fifo_avail(CAN_TypeDef* _CAN) {
  uint32_t can_idx = can_get_idx(_CAN);
  if (G_can_fifo_rx[can_idx].size == 0) {
    return 0;
  }
  return G_can_fifo_rx[can_idx].entries[0].data_len;
}

size_t can_fifo_rx(CAN_TypeDef* _CAN, uint32_t * id, size_t * id_bitlen, uint8_t* frame, size_t frame_max_len) {
  uint32_t can_idx = can_get_idx(_CAN);
  // no entry readable in the fifo
  if (G_can_fifo_rx[can_idx].size == 0) {
    *id_bitlen = 0;
    return 0;
  }
  // consume in 0, append at the end
  *id = G_can_fifo_rx[can_idx].entries[0].id;
  *id_bitlen = G_can_fifo_rx[can_idx].entries[0].id_bitlen;
  frame_max_len = MIN(frame_max_len, G_can_fifo_rx[can_idx].entries[0].data_len);
  memmove(frame, G_can_fifo_rx[can_idx].entries[0].data, frame_max_len);
  __disable_irq();
  // consume fifo entry
  G_can_fifo_rx[can_idx].size--;
  memmove(&G_can_fifo_rx[can_idx].entries[0], &G_can_fifo_rx[can_idx].entries[1], sizeof(G_can_fifo_rx[can_idx].entries[0])*G_can_fifo_rx[can_idx].size);
  __enable_irq();
  // return consumed entry len
  return frame_max_len;
}

static void Configure_CAN_generic(CAN_TypeDef* _CAN, uint32_t frequency, uint32_t auto_retransmit) {

  CAN_IRQHandler_init();

  // Wipe CAN peripheral, by u32
  uint32_t* p = (uint32_t*)_CAN;
  size_t l = sizeof(CAN_TypeDef)/4;
  while (l--) {
    *p++ = 0;
  }

  _CAN->MCR |= CAN_MCR_INRQ;
  while(! (_CAN->MSR & CAN_MSR_INAK));

#define CFG_BTR(sjw, ts2, ts1, presc) (((sjw - 1)<<24) | ((ts2 - 1)<<20) | ((ts1 - 1)<<16) | (presc - 1))
#if CFG_BTR(2, 3, 4, 4) != 0x01230003
#error Invalid macro
#endif // CFG_BTR
  switch(frequency) {
    default:
    case 500000:
      //_CAN->BTR = CFG_BTR(2, 3, 4, 4); // 0x01230003 // some disconnect after 30000sec of activity
      //_CAN->BTR = CFG_BTR(1, 7, 8, 2);
      _CAN->BTR = CFG_BTR(1, 3, 12, 2);
      break;
    case 1000000:
      //_CAN->BTR = 0x001c0004;
      break;
    case 100000:
      //_CAN->BTR = 0x001c0031;
      break;
  }

  // leave sleep mode upon bus activity
  _CAN->MCR |= CAN_MCR_AWUM;

  if (auto_retransmit) {
    // retransmit after colliding frame has ended
    _CAN->MCR &= ~CAN_MCR_NART;
  }
  else {
    _CAN->MCR |= CAN_MCR_NART; 
  }

  // fifo lock after reception to avoid overrun
  _CAN->MCR |= CAN_MCR_RFLM;

  // init any mask to receive everything from the bus
  _CAN->FMR |= CAN_FMR_FINIT;

  // 16 bits matchall
  _CAN->FA1R &= ~CAN_FA1R_FACT0;
  _CAN->FM1R &= ~CAN_FM1R_FBM0;
  _CAN->FS1R &= ~CAN_FS1R_FSC0;
  _CAN->FFA1R &= ~CAN_FFA1R_FFA0;
  _CAN->sFilterRegister[0].FR1 = 0;
  _CAN->sFilterRegister[0].FR2 = 0;
  _CAN->FA1R |= CAN_FA1R_FACT0;

  // 32 bits matchall
  _CAN->FA1R &= ~CAN_FA1R_FACT1;
  _CAN->FM1R &= ~CAN_FM1R_FBM1;
  _CAN->FS1R |= CAN_FS1R_FSC1;
  _CAN->FFA1R &= ~CAN_FFA1R_FFA1;
  _CAN->sFilterRegister[1].FR1 = 0;
  _CAN->sFilterRegister[1].FR2 = 0; 
  _CAN->FA1R |= CAN_FA1R_FACT1;


  // interrupt on fifo entry
  _CAN->IER |= CAN_IER_FMPIE0 | CAN_IER_FMPIE1;

  _CAN->FMR &= ~CAN_FMR_FINIT;

  // enter normal mode
  _CAN->MCR &= ~CAN_MCR_INRQ;
} 

void Configure_CAN1(uint32_t frequency) {
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_CAN1);

  Configure_CAN_generic(CAN1, frequency, 0);

  // CAN RX
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_8, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(GPIOB, LL_GPIO_PIN_8, LL_GPIO_AF_9);
  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_8, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_8, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_8, LL_GPIO_PULL_NO);
  // CAN TX 
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_9, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(GPIOB, LL_GPIO_PIN_9, LL_GPIO_AF_9);
  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_9, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_9, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_9, LL_GPIO_PULL_NO);

  NVIC_EnableIRQ(CAN1_RX0_IRQn);
  NVIC_EnableIRQ(CAN1_RX1_IRQn);
}

void Configure_CAN3(uint32_t frequency) {
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_CAN3);

  Configure_CAN_generic(CAN3, frequency, 0);

  // CAN RX
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_8, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_8, LL_GPIO_AF_11);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_8, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_8, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_8, LL_GPIO_PULL_NO);
  // CAN TX 
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_4, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_4, LL_GPIO_AF_11);
  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_4, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_4, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_4, LL_GPIO_PULL_NO);

  NVIC_EnableIRQ(CAN3_RX0_IRQn);
  NVIC_EnableIRQ(CAN3_RX1_IRQn);
}
