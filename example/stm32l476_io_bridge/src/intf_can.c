#include "main.h"

size_t can_fifo_rx_size;
struct {
  uint32_t id;
  size_t id_bitlen;
  uint8_t data[8];
  size_t data_len;
} can_fifo_rx_entries[CAN_FIFO_RX_ENTRY_COUNT];


size_t can_tx(uint32_t id, size_t id_bitlen, uint8_t *frame, size_t frame_len) {
  // no free slot?
  if (!(CAN->TSR&(CAN_TSR_TME0|CAN_TSR_TME1|CAN_TSR_TME2))) {
    // try to recover any previous transmission error
    if (CAN->ESR&0xFF) {
      CAN->TSR |= CAN_TSR_ABRQ0|CAN_TSR_ABRQ1|CAN_TSR_ABRQ2;
      while(CAN->TSR&(CAN_TSR_ABRQ0|CAN_TSR_ABRQ1|CAN_TSR_ABRQ2));
    }
    if (!(CAN->TSR&(CAN_TSR_TME0|CAN_TSR_TME1|CAN_TSR_TME2))) { 
      return -1;
    }
  }
  uint8_t slot = (CAN->TSR & CAN_TSR_CODE) >>CAN_TSR_CODE_Pos;
  if (slot == 3) {
    return -2;
  }

  switch(id_bitlen) {
    case 11:
      CAN->sTxMailBox[slot].TIR = (id << 21);
      break;
    case 29:
      CAN->sTxMailBox[slot].TIR = (id << 3) | CAN_TI0R_IDE;
      break;
    default:
      return -3;
  }
  if (frame_len > 8) {
    return -4;
  }
  CAN->sTxMailBox[slot].TDTR = frame_len;
  // copy data
  uint32_t d[2] = {0};
  uint32_t i=0;
  while (i<frame_len) {
    d[i/4] |= (frame[i]&0xFF)<<((i%4)*8);
    i++;
  }
  CAN->sTxMailBox[slot].TDLR = d[0];
  CAN->sTxMailBox[slot].TDHR = d[1];

  // enable transmission of that slot
  CAN->sTxMailBox[slot].TIR |= CAN_TI0R_TXRQ;
  return frame_len;
}

// static size_t can_avail(void) {
//   if (CAN->RF0R&(CAN_RF0R_FMP0_Msk)) {
//     return CAN->sFIFOMailBox[0].RDTR & CAN_RDT0R_DLC_Msk;
//   }
//   if (CAN->RF1R&(CAN_RF1R_FMP1_Msk)) {
//     return CAN->sFIFOMailBox[1].RDTR & CAN_RDT1R_DLC_Msk;
//   }
//   return 0;
// }

static size_t can_rx(uint32_t * id, size_t * id_bitlen, uint8_t* frame, size_t frame_max_len) {
  uint32_t slot;
  if (CAN->RF0R&(CAN_RF0R_FMP0_Msk)) {
    slot = 0;
  }
  else if (CAN->RF1R&(CAN_RF1R_FMP1_Msk)) {
    slot = 1;
  }
  else {
    if (id_bitlen) {
      *id_bitlen = 0;
    }
    return 0;
  }

  if (id_bitlen) {
    *id_bitlen = CAN->sFIFOMailBox[slot].RIR & CAN_RI0R_IDE?29:11;
    if (id) {
      *id = CAN->sFIFOMailBox[slot].RIR >> ( 32 - *id_bitlen);
    }
  }
  frame_max_len = MIN(frame_max_len, CAN->sFIFOMailBox[slot].RDTR & CAN_RDT0R_DLC_Msk);

  if (frame) {
    uint32_t d[2] = {0};
    uint32_t i=0;
    d[0] = CAN->sFIFOMailBox[slot].RDLR;
    d[1] = CAN->sFIFOMailBox[slot].RDHR;
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
      CAN->RF0R |= CAN_RF0R_RFOM0;
      break;
    case 1:
      CAN->RF1R |= CAN_RF1R_RFOM1;
      break;
  }
  return frame_max_len; 
}

void CAN_IRQHandler_init(void) {
  can_fifo_rx_size = 0;
  memset(can_fifo_rx_entries, 0, sizeof (can_fifo_rx_entries));
}

void CAN1_RX0_IRQHandler(void) {
  if (can_fifo_rx_size >= CAN_FIFO_RX_ENTRY_COUNT) {
    // consume received frame, no FIFO space to store it
    can_rx(NULL, NULL, NULL, 0);
    // discard
    return;
  }
  // consume in 0, append at the end
  size_t entry = can_fifo_rx_size++;
  can_fifo_rx_entries[entry].data_len =
    can_rx(&can_fifo_rx_entries[entry].id,
         &can_fifo_rx_entries[entry].id_bitlen,
         can_fifo_rx_entries[entry].data,
         8);
}

void CAN1_RX1_IRQHandler(void) {
  CAN1_RX0_IRQHandler();
}

size_t can_fifo_avail(void) {
  if (can_fifo_rx_size == 0) {
    return 0;
  }
  return can_fifo_rx_entries[0].data_len;
}

size_t can_fifo_rx(uint32_t * id, size_t * id_bitlen, uint8_t* frame, size_t frame_max_len) {
  // no entry readable in the fifo
  if (can_fifo_rx_size == 0) {
    *id_bitlen = 0;
    return 0;
  }
  // consume in 0, append at the end
  *id = can_fifo_rx_entries[0].id;
  *id_bitlen = can_fifo_rx_entries[0].id_bitlen;
  frame_max_len = MIN(frame_max_len, can_fifo_rx_entries[0].data_len);
  memmove(frame, can_fifo_rx_entries[0].data, frame_max_len);
  __disable_irq();
  // consume fifo entry
  can_fifo_rx_size--;
  memmove(&can_fifo_rx_entries[0], &can_fifo_rx_entries[1], sizeof(can_fifo_rx_entries[0])*can_fifo_rx_size);
  __enable_irq();
  // return consumed entry len
  return frame_max_len;
}

void Configure_CAN(uint32_t frequency, uint32_t auto_retransmit) {

  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_CAN1);

  // Wipe CAN peripheral, by u32
  uint32_t* p = (uint32_t*)CAN;
  size_t l = sizeof(CAN_TypeDef)/4;
  while (l--) {
    *p++ = 0;
  }

  CAN->MCR |= CAN_MCR_INRQ;
  while(! (CAN->MSR & CAN_MSR_INAK));

  switch(frequency) {
    default:
    case 500000:
      CAN->BTR = 0x001c0009;
      break;
    case 1000000:
      CAN->BTR = 0x001c0004;
      break;
    case 100000:
      CAN->BTR = 0x001c0031;
      break;
  }

  // leave sleep mode upon bus activity
  CAN->MCR |= CAN_MCR_AWUM;

  if (auto_retransmit) {
    // retransmit after colliding frame has ended
    CAN->MCR &= ~CAN_MCR_NART;
  }
  else {
    CAN->MCR |= CAN_MCR_NART; 
  }

  // fifo lock after reception to avoid overrun
  CAN->MCR |= CAN_MCR_RFLM;

  // init any mask to receive everything from the bus
  CAN->FMR |= CAN_FMR_FINIT;

  // 16 bits matchall
  CAN->FA1R &= ~CAN_FA1R_FACT0;
  CAN->FM1R &= ~CAN_FM1R_FBM0;
  CAN->FS1R &= ~CAN_FS1R_FSC0;
  CAN->FFA1R &= ~CAN_FFA1R_FFA0;
  CAN->sFilterRegister[0].FR1 = 0;
  CAN->sFilterRegister[0].FR2 = 0;
  CAN->FA1R |= CAN_FA1R_FACT0;

  // 32 bits matchall
  CAN->FA1R &= ~CAN_FA1R_FACT1;
  CAN->FM1R &= ~CAN_FM1R_FBM1;
  CAN->FS1R |= CAN_FS1R_FSC1;
  CAN->FFA1R &= ~CAN_FFA1R_FFA1;
  CAN->sFilterRegister[1].FR1 = 0;
  CAN->sFilterRegister[1].FR2 = 0; 
  CAN->FA1R |= CAN_FA1R_FACT1;

  CAN_IRQHandler_init();

  // interrupt on fifo entry
  CAN->IER |= CAN_IER_FMPIE0 | CAN_IER_FMPIE1;

  CAN->FMR &= ~CAN_FMR_FINIT;

  // CAN RX
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_11, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_11, LL_GPIO_AF_9);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_11, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_11, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_11, LL_GPIO_PULL_NO);
  // CAN TX 
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_12, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_12, LL_GPIO_AF_9);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_12, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_12, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_12, LL_GPIO_PULL_NO);

  NVIC_EnableIRQ(CAN1_RX0_IRQn);
  NVIC_EnableIRQ(CAN1_RX1_IRQn);

  // enter normal mode
  CAN->MCR &= ~CAN_MCR_INRQ;
} 
