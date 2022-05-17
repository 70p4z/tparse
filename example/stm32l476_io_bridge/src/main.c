/*******************************************************************************
*   TPARSE Demonstration tool: iobridge
*   (c) 2022 Olivier TOMAZ
*
*  Licensed under the Apache License, Version 2.0 (the "License");
*  you may not use this file except in compliance with the License.
*  You may obtain a copy of the License at
*
*      http://www.apache.org/licenses/LICENSE-2.0
*
*  Unless required by applicable law or agreed to in writing, software
*  distributed under the License is distributed on an "AS IS" BASIS,
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*  See the License for the specific language governing permissions and
*  limitations under the License.
********************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#ifndef MAX
#define MAX(x,y) ((x)>(y)?(x):(y))
#endif // MAX

#ifndef __bswap_32
/* Swap bytes in 32 bit value.  */
#define __bswap_32(x) \
     ((((x) & 0xff000000) >> 24) | (((x) & 0x00ff0000) >>  8) |                      \
      (((x) & 0x0000ff00) <<  8) | (((x) & 0x000000ff) << 24))
#endif // __bswap_32

#define NO_TIMEOUT 0
#define TIMEOUT_1S 1000
#define CAN_FIFO_RX_ENTRY_COUNT 256

/* Private functions ---------------------------------------------------------*/

char uart_buffer[32+512];
uint8_t tmp[300];
char iso_buffer[MAX(256+2, 5+255)]; // max T=0 command/responselength
size_t iso_offset_read;

const struct {
  uint8_t port;
  uint8_t pin;
} gpio_reserved[] = {
  {0, 2},  // STLINK USART TX (D1)
  {0, 3},  // STLINK USART RX (D0)
  {0, 13}, // STLINK SWD
  {0, 14}, // STLINK SWD
  //(0, 5),  // ISO SWP         (D13) =1 at boot, mutable by UART
  {0, 6},  // I2C INT         (D12)
  {0, 8},  // ISO CLK         (D7)
  {0, 9},  // ISO IO          (D8)
  {1, 6},  // ISO GND         (D10) (SE POWER)
  {2, 7},  // ISO RST         (D9)
  {1, 8},  // I2C SCL         (D15)
  {1, 9},  // I2C SDA         (D14)
  //(1, 10),  // ISO SWP        (D6) =0 at boot, mutable by UART
  {0, 11}, // CAN RX          (CN10-14)
  {0, 12}, // CAN TX          (CN10-12)
};

#include "tparse.h"
#include "stddef.h"
#include "iso7816.h"

void Configure_CAN(uint32_t frequency, uint32_t auto_retransmit);

uint32_t tparse_al_time(void) {
  return uwTick; /* todo bind the systick count here */;
}

void uart_reply_mem(uint8_t* ptr, size_t len) {
  while (len--) {
    while (!LL_USART_IsActiveFlag_TXE(USART2)){}
    LL_USART_TransmitData8(USART2, *ptr++);
  }
}

void uart_reply(char* string) {
  uart_reply_mem((uint8_t*)string, strlen(string));
}

uint8_t n2h(uint8_t c) {
  if (c<10) {
    return c+'0';
  }
  else {
    return c+'a'-10;
  }
}

void uart_reply_hex(uint8_t* buf, size_t len) {
  char c[2];
  c[1] = 0;
  while (len--) {
    c[0] = n2h(*buf>>4);
    uart_reply(c);
    c[0] = n2h(*buf&0xF);
    uart_reply(c);
    buf++;
  }
}

void i2c_stop(void) {
  LL_I2C_GenerateStopCondition(I2C1);
  LL_I2C_Disable(I2C1);
  volatile uint32_t i = 0x100 ; while (i--);
  LL_I2C_Enable(I2C1);
}

uint32_t i2c_strobe(uint32_t addr) {
  // cleanup previous transaction flags
  I2C1->ICR = 0xFFFFFFFF;
  LL_I2C_HandleTransfer(I2C1, addr, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
  uint32_t limit = uwTick + 10;
  while (I2C1->CR2 & I2C_CR2_START) {
    if (uwTick - limit < 0x80000000UL) {
      // timeout presenting address
      i2c_stop();
      return 0;
    }
  }
  if (I2C1->ISR & (I2C_ISR_STOPF|I2C_ISR_NACKF|I2C_ISR_BERR|I2C_ISR_ARLO)) {
    // invalid address
    i2c_stop();
    return 0;
  }
  i2c_stop();
  return 1;
}

size_t i2c_read(uint8_t addr, uint8_t* buf, size_t maxlen) {
  size_t len = maxlen;
  // cleanup previous transaction flags
  I2C1->ICR = 0xFFFFFFFF;
  LL_I2C_HandleTransfer(I2C1, addr, LL_I2C_ADDRSLAVE_7BIT, len, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ);
  uint32_t limit = uwTick + TIMEOUT_1S;
  while (I2C1->CR2 & I2C_CR2_START) {
    if (uwTick - limit < 0x80000000UL) {
      // timeout presenting address
      i2c_stop();
      return -1;
    }
  }
  if (I2C1->ISR & (I2C_ISR_STOPF|I2C_ISR_NACKF|I2C_ISR_BERR|I2C_ISR_ARLO)) {
    // invalid address
    i2c_stop();
    return -2;
  }
  while (len--) {
    while (!LL_I2C_IsActiveFlag_RXNE(I2C1)) {
      if (I2C1->ISR & (I2C_ISR_STOPF|I2C_ISR_NACKF|I2C_ISR_BERR|I2C_ISR_ARLO)) {
        i2c_stop();
        return maxlen - len - 1;
      }
    }
    *buf++ = LL_I2C_ReceiveData8(I2C1);
  }
  return maxlen;
}

size_t i2c_write(uint8_t addr, uint8_t* buf, size_t len) {
  size_t l = len;
  // cleanup previous transaction flags
  I2C1->ICR = 0xFFFFFFFF;
  LL_I2C_HandleTransfer(I2C1, addr, LL_I2C_ADDRSLAVE_7BIT, len, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
  uint32_t limit = uwTick + TIMEOUT_1S;
  while (I2C1->CR2 & I2C_CR2_START) {
    if (uwTick - limit < 0x80000000UL) {
      // timeout presenting address
      i2c_stop();
      return -1;
    }
  }
  if (I2C1->ISR & (I2C_ISR_STOPF|I2C_ISR_NACKF|I2C_ISR_BERR|I2C_ISR_ARLO)) {
    // invalid address
    i2c_stop();
    return -2;
  }
  while (len--) {
    while (!LL_I2C_IsActiveFlag_TXE(I2C1)) {
      if (I2C1->ISR & (I2C_ISR_STOPF|I2C_ISR_NACKF|I2C_ISR_BERR|I2C_ISR_ARLO)) {
        i2c_stop();
        return l - len - 1;
      }
    }
    LL_I2C_TransmitData8(I2C1, *buf++);
  }
  return l;
}

#ifdef I2C_FLAG_EXTI
uint32_t i2c_i_flag;
void EXTI9_5_IRQHandler(void) {
  NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
  if (LL_EXTI_ReadFlag_0_31(LL_EXTI_LINE_6)) {
    NVIC_DisableIRQ(EXTI9_5_IRQn);
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_6);
    i2c_i_flag = 1;
    gpio_set(2, 0, 1); // OTO DEBUG PC0
  }
}

uint32_t i2c_consume_int(void) {
  // don't use EXTI->PRx register to avoid race condition
  // between read and clear and a external set
  uint32_t flag = i2c_i_flag;
  if (! flag && !gpio_get(0, 6)) {
    // it sounds like we missed the EXTI !!
    flag = 1;
  }
  i2c_i_flag = 0;
  NVIC_EnableIRQ(EXTI9_5_IRQn);
  return flag;
}
#endif // I2C_FLAG_EXTI

void gpio_set(uint32_t port, uint32_t pin, uint32_t value) {
  // denied?
  for (int i = 0; i< sizeof(gpio_reserved)/ sizeof(gpio_reserved[0]); i++) {
    if (gpio_reserved[i].port == port && gpio_reserved[i].pin == pin) {
      return;
    }
  }
  // configure GPIO as output
  GPIO_TypeDef* GPIO = (GPIO_TypeDef*)((uintptr_t)GPIOA_BASE + 0x400*port);
  uint32_t PIN = 1<<pin;
  LL_GPIO_SetPinMode(GPIO, PIN, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinSpeed(GPIO, PIN, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinPull(GPIO, PIN, LL_GPIO_PULL_NO);
  // set state
  GPIO->BSRR = PIN<<(value?0:16);
}

uint32_t gpio_get(uint32_t port, uint32_t pin) {
  // no reserved pin for reading
  GPIO_TypeDef* GPIO = (GPIO_TypeDef*)((uintptr_t)GPIOA_BASE + 0x400*port);
  return GPIO->IDR & (1<<pin);
}

void iso_delay_ms(uint32_t ms) {
  LL_mDelay(ms);
}

void iso_rst(uint32_t level) {
  GPIOC->BSRR = LL_GPIO_PIN_7<<(level?0:16);
}

void iso_gnd(uint32_t level) {
  GPIOB->BSRR = LL_GPIO_PIN_6<<(level?0:16);
}

void iso_usart_ETU(uint32_t etu) {
  LL_USART_Disable(USART1);
  // set baudrate
  // multiply by prescaler to take into account real clock on the clk line
  USART1->BRR = ((etu<<1 /*etu=8 <-> 0x10*/) * ((USART1->GTPR&0xFF)<<1))&(~(1<<3));
  LL_USART_Enable(USART1);
}

void iso_usart_flush(void) {
  while (LL_USART_IsActiveFlag_BUSY(USART1)){}
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_5);
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_5, sizeof(iso_buffer));
  iso_offset_read = 0;
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_5);
}

size_t iso_usart_available(void) {
  return (2 * sizeof(iso_buffer) - DMA1_Channel5->CNDTR - iso_offset_read) % sizeof(iso_buffer);
}

// NOTE: we're not supposed to be consuming data while receiving, it is a purely half duplex protocol
size_t iso_usart_recv(uint8_t* buffer, size_t length, uint32_t timeout) {
  size_t l, len;
  uint32_t start = uwTick + timeout;
  // consider the packet always fit the DMA buffer, don't stream
  while(iso_usart_available()<length) {
    if (timeout != NO_TIMEOUT && (uwTick - start) < 0x80000000UL) {
      return 0;
    }
  }

  // copy data with two memmove
  len = length;
  while(length) {
    l = MIN(length, sizeof(iso_buffer)-iso_offset_read);
    if (buffer) {
      memmove(buffer, iso_buffer+iso_offset_read, l);
      buffer += l;
    }
    length -= l;
    iso_offset_read += l;
    if (iso_offset_read>=sizeof(iso_buffer)) {
      iso_offset_read %= sizeof(iso_buffer);
    }
  }
  return len;
}

void iso_usart_send(const uint8_t* buffer, size_t length) {
  if (length) {
    size_t l;
    // wait idle as we're consuming what we're sending afterwards (thanks ST wrong HD)
    // we don't want some funky overlay.
    while (LL_USART_IsActiveFlag_BUSY(USART1)){}
    // split transfer in chunks
    while(length) {
      l = MIN(sizeof(iso_buffer), length);
      while(l--) {
        USART1->ICR = 0xFFFFFFFF;
        LL_USART_TransmitData8(USART1, *buffer++);
        while (!LL_USART_IsActiveFlag_TC(USART1)){}
      }
      // wait end of transfer
      while ((USART1->ISR & (USART_ISR_TXE|USART_ISR_BUSY) ) != (USART_ISR_TXE) );

      // shall be done quickly to avoid the real reply to be concat to our sent data
      l = MIN(iso_usart_available(), MIN(sizeof(iso_buffer), length));

      iso_usart_recv(NULL, l, NO_TIMEOUT /*has been tested as available*/);
      length -= MIN(sizeof(iso_buffer), length);
    }
  }
}

#ifdef CAN
size_t can_tx(uint32_t id, size_t id_bitlen, uint8_t *frame, size_t frame_len) {
  // no free slot?
  if (!(CAN->TSR&(CAN_TSR_TME0|CAN_TSR_TME1|CAN_TSR_TME2))) {
    return -1;
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

size_t can_avail(void) {
  if (CAN->RF0R&(CAN_RF0R_FMP0_Msk)) {
    return CAN->sFIFOMailBox[0].RDTR & CAN_RDT0R_DLC_Msk;
  }
  if (CAN->RF1R&(CAN_RF1R_FMP1_Msk)) {
    return CAN->sFIFOMailBox[1].RDTR & CAN_RDT1R_DLC_Msk;
  }
  return 0;
}

size_t can_rx(uint32_t * id, size_t * id_bitlen, uint8_t* frame, size_t frame_max_len) {
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

size_t can_fifo_rx_size;
struct {
  uint32_t id;
  size_t id_bitlen;
  uint8_t data[8];
  size_t data_len;
} can_fifo_rx_entries[CAN_FIFO_RX_ENTRY_COUNT];

void CAN_IRQHandler_init(void) {
  can_fifo_rx_size = 0;
  memset(can_fifo_rx_entries, 0, sizeof (can_fifo_rx_entries));
}

void CAN1_RX0_IRQHandler(void) {
  if (can_fifo_rx_size >= CAN_FIFO_RX_ENTRY_COUNT) {
    // consume received frame, no FIFO space to store it
    can_rx(NULL, NULL, NULL, 0);
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

#endif // CAN

void interp(void) {

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  tparse_ctx_t tp;
  tparse_init(&tp, uart_buffer, sizeof(uart_buffer), " \n");

  // set one PIN in each level at startup, to allow for specific reset condition
  // (device factory reset for example). Those pins are switchable but have a default
  // and known default value
  gpio_set(0, 5, 1);
  gpio_set(1, 10, 0);

  while (1)
  {
    uint32_t cmd;
    uint32_t addr;
    uint32_t len;
    uint32_t port;
    uint32_t pin;
    uint32_t val;
    size_t ts;

    tparse_finger(&tp, sizeof(uart_buffer) - DMA1_Channel6->CNDTR);

    static const char * const cmds[] = {
        "i2cr", "i2cw", "t0", "i2ci",
        "gpo", "gpi", "atr", "off",
        "info", "on", "i2ciwait", "i2cscan",
        "reset",
        "ctx", "crx", "cavail", "ccfg",
    };
    len = tparse_has_line(&tp);
    if (len) {
      ts = sizeof(tmp);
      cmd = tparse_token_in(&tp, (char**)cmds, sizeof(cmds)/sizeof(cmds[0]), (char*)tmp, &ts);
      switch (cmd) {
      default:
      case __COUNTER__:
        uart_reply("ERROR: unsupported: ");
        ts = MIN(ts, 32);
        uart_reply_mem(tmp, ts);
        uart_reply("\n");
        break;
      case __COUNTER__:
        // I2C read
        addr = tparse_token_u32(&tp);
        if (addr > 0x7F || addr == -1) {
          uart_reply("ERROR: invalid address\n");
          break;
        }
        len = tparse_token_u32(&tp);
        if (len == -1 ||len == 0) {
          uart_reply("ERROR: invalid length\n");
          break;
        }
        len = MIN(len, sizeof(tmp));
        val = i2c_read(addr, tmp, len);
        if (len != val) {
          uart_reply("ERROR: not all bytes read: ");
          uart_reply_hex((uint8_t*)&val, 4);
          uart_reply("\n");
          break;
        }
        uart_reply("OK:");
        uart_reply_hex(tmp, len);
        uart_reply("\n");
        break;
      case __COUNTER__:
        // I2C write
        addr = tparse_token_u32(&tp);
        if (addr > 0x7F || addr == -1) {
          uart_reply("ERROR: invalid address\n");
          break;
        }
        len = tparse_token_hex(&tp, tmp, sizeof(tmp));
        if (len == 0) {
          uart_reply("ERROR: invalid data\n");
          break;
        }
        val = i2c_write(addr, tmp, len);
        if (len != val) {
          uart_reply("ERROR: not all bytes written: ");
          uart_reply_hex((uint8_t*)&val, 4);
          uart_reply("\n");
          break;
        }
        uart_reply("OK:\n");
        break;
      case __COUNTER__:
        // T=0 APDU
        len = tparse_token_hex(&tp, tmp, sizeof(tmp));
        if (len > 255+5 || len < 5) {
          uart_reply("ERROR: invalid T=0 apdu\n");
          break;
        }
        if (len > 5 && len - 5 != tmp[4]) {
          uart_reply("ERROR: malformed apdu: ");
          uart_reply_hex(tmp, len);
          uart_reply("\n");
          break;
        }
        len = iso_apdu_t0(tmp, len);
        uart_reply("OK:");
        uart_reply_hex(tmp, len);
        uart_reply("\n");
        break;
      case __COUNTER__:
        // I2C interrupt read
        uart_reply("OK:");
#ifdef I2C_FLAG_EXTI
        uart_reply_hex((uint8_t*)(i2c_consume_int()?&"\x01":&"\x00"), 1);
#else // I2C_FLAG_EXTI
        uart_reply_hex((uint8_t*)(!gpio_get(0, 6)?&"\x01":&"\x00"), 1);
#endif // I2C_FLAG_EXTI
        uart_reply("\n");
        break;
      case __COUNTER__:
        // GPO
        port = tparse_token_u32(&tp);
        if (port > 7) {
          uart_reply("ERROR: invalid port\n");
          break;
        }
        pin = tparse_token_u32(&tp);
        if (pin > 15) {
          uart_reply("ERROR: invalid pin\n");
          break;
        }
        val = tparse_token_u32(&tp);
        if (val == -1) {
          uart_reply("ERROR: invalid value\n");
          break;
        }
        gpio_set(port, pin, val);
        uart_reply("OK:\n");
        break;
      case __COUNTER__:
        // GPI
        port = tparse_token_u32(&tp);
        if (port > 7) {
          uart_reply("ERROR: invalid port\n");
          break;
        }
        pin = tparse_token_u32(&tp);
        if (pin > 15) {
          uart_reply("ERROR: invalid pin\n");
          break;
        }
        val = gpio_get(port, pin);
        uart_reply("OK:");
        val = val?1:0;
        uart_reply_hex((uint8_t*)&val, 1);
        uart_reply("\n");
        break;
      case __COUNTER__:
        // ATR
        len = iso_powercycle(tmp, sizeof(tmp));
        if (len < 2) {
          iso_power_down();
          uart_reply("ERROR: no card detected\n");
          break;
        }
        uart_reply("OK:");
        uart_reply_hex(tmp, len);
        uart_reply("\n");
        break;
      case __COUNTER__:
        // OFF
        iso_power_down();
        uart_reply("OK:\n");
        break;
      case __COUNTER__:
        // info
        uart_reply("INFO:");
        len = tparse_token_hex(&tp, tmp, sizeof(tmp));
        if (len) {
          uart_reply_hex(tmp, len);
          uart_reply(",");
        }
        uart_reply("VERSION=0.1\n");
        break;
      case __COUNTER__:
        // ON
        iso_gnd(0);
        uart_reply("OK:\n");
        break;
      case __COUNTER__:
        // i2ciwait
        len = uwTick + 30*TIMEOUT_1S;
        // until timeout
        for(;;) {
          if ((uwTick - len) < 0x80000000UL) {
            uart_reply("TIMEOUT:\n");
            break;
          }
#ifdef I2C_FLAG_EXTI
          if (i2c_consume_int()) {
#else // I2C_FLAG_EXTI
          if (!gpio_get(0, 6)) {
#endif // I2C_FLAG_EXTI
            uart_reply("OK:\n");
            break;
          }
        }
        break;
      case __COUNTER__:
        // I2C scan
        len = 0;
        uart_reply("OK:");
        while (len<256) {
          if (i2c_strobe(len)) {
            uart_reply("0x");
            uart_reply_hex((uint8_t*)&len, 1);
            uart_reply(",");
          }
          len+=2; // skip the READ address each time
        }
        uart_reply("\n");
        break;
      case __COUNTER__:
        // reset
        uart_reply("OK:\n");
        LL_mDelay(10);
        NVIC_SystemReset();
        break;
      case __COUNTER__:
        // can tx
        val = tparse_token_u32(&tp); // id
        // Standard/Extended
        if (1 != tparse_token(&tp, tmp, 1)) {
        error_can_style:
          uart_reply("ERROR: invalid Standard/Extended indication\n");
          break;
        } 
        switch(tmp[0]) {
          case 'E':
          case 'e':
            ts = 29;
            break;
          case 'S':
          case 's':
            ts = 11;
            break;
          default:
            goto error_can_style;
        }
        len = tparse_token_hex(&tp, tmp, sizeof(tmp));
        if (len > 8) {
          uart_reply("ERROR: invalid frame len (max 8 bytes)\n");
          break;
        }
        ts = can_tx(val, ts, tmp, len);
        if (len != ts) {
          uart_reply("ERROR: CAN transmit failed,");
          uart_reply_hex(&ts,4);
          uart_reply("\n");
          break;
        }
        uart_reply("OK:\n");
        break;
      case __COUNTER__:
        // can rx
        len = can_fifo_rx(&addr, &ts, tmp, sizeof(tmp));
        if (ts == 0) {
          uart_reply("ERROR: no frame available\n");
          break;
        }
        uart_reply("OK:");
        addr = __bswap_32(addr); // change endianess (LE cpu => BE uart)
        uart_reply_hex(&addr, 4);
        switch(ts) {
          case 11:
            uart_reply(",s,");
            break;
          case 29:
            uart_reply(",e,");
            break;
          default:
            uart_reply(",U,");
            break;
        }
        uart_reply_hex(tmp, len);
        uart_reply("\n");
        break;
      case __COUNTER__:
        // can available
        len = can_fifo_avail();
        uart_reply("OK:");
        uart_reply_hex(&len, 1);
        uart_reply("\n");
        break;
      case __COUNTER__:
        // can config
        val = tparse_token_u32(&tp);
        if (val > SystemCoreClock/4) {
          uart_reply("ERROR: invalid frequency parameter\n");
          break;
        }
        Configure_CAN(val, 0);
        uart_reply("OK:\n");
        break;
      }
      // discard any remnant of the processed line
      tparse_discard_line(&tp);
    }
  }
}

void Configure_USART(void)
{
  /* DMA1 used for USART2 Transmission and Reception
   */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  LL_DMA_ConfigTransfer(DMA1, LL_DMA_CHANNEL_6,
            LL_DMA_DIRECTION_PERIPH_TO_MEMORY |
            LL_DMA_PRIORITY_HIGH              |
            LL_DMA_MODE_CIRCULAR              |
            LL_DMA_PERIPH_NOINCREMENT         |
            LL_DMA_MEMORY_INCREMENT           |
            LL_DMA_PDATAALIGN_BYTE            |
            LL_DMA_MDATAALIGN_BYTE);
  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_6,
             LL_USART_DMA_GetRegAddr(USART2, LL_USART_DMA_REG_DATA_RECEIVE),
             (uint32_t)uart_buffer,
             LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_6));
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_6, sizeof(uart_buffer));
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_6, LL_DMA_REQUEST_2);

  /* Enable DMA Channel Rx */
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_6);

  /* Configure Tx Pin as : Alternate function, High Speed, Push pull, Pull up */
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_2, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_2, LL_GPIO_AF_7);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_2, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_2, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_2, LL_GPIO_PULL_UP);

  /* Configure Rx Pin as : Alternate function, High Speed, Push pull, Pull up */
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_3, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_3, LL_GPIO_AF_7);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_3, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_3, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_3, LL_GPIO_PULL_NO);

  /* (2) Enable USART2 peripheral clock and clock source ****************/
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

  /* Set clock source */
  LL_RCC_SetUSARTClockSource(LL_RCC_USART2_CLKSOURCE_PCLK1);

  /* (3) Configure USART2 functional parameters ********************************/

  /* Disable USART prior modifying configuration registers */
  /* Note: Commented as corresponding to Reset value */
  // LL_USART_Disable(USART2);

  /* TX/RX direction */
  LL_USART_SetTransferDirection(USART2, LL_USART_DIRECTION_TX_RX);

  /* 8 data bit, 1 start bit, 1 stop bit, no parity */
  LL_USART_ConfigCharacter(USART2, LL_USART_DATAWIDTH_8B, LL_USART_PARITY_NONE, LL_USART_STOPBITS_1);

  /* No Hardware Flow control */
  /* Reset value is LL_USART_HWCONTROL_NONE */
  // LL_USART_SetHWFlowCtrl(USART2, LL_USART_HWCONTROL_NONE);

  /* Oversampling by 16 */
  /* Reset value is LL_USART_OVERSAMPLING_16 */
  // LL_USART_SetOverSampling(USART2, LL_USART_OVERSAMPLING_16);

  /* Set Baudrate to 115200 using APB frequency set to 80000000 Hz */
  /* Frequency available for USART peripheral can also be calculated through LL RCC macro */
  /* Ex :
      Periphclk = LL_RCC_GetUSARTClockFreq(Instance); or LL_RCC_GetUARTClockFreq(Instance); depending on USART/UART instance

      In this example, Peripheral Clock is expected to be equal to 80000000 Hz => equal to SystemCoreClock
  */
  LL_USART_SetBaudRate(USART2, SystemCoreClock, LL_USART_OVERSAMPLING_16, 921600);

  /* (4) Enable USART2 **********************************************************/
  LL_USART_Enable(USART2);

  /* Polling USART initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(USART2))) || (!(LL_USART_IsActiveFlag_REACK(USART2))))
  {
  }

  /* Enable DMA RX Interrupt */
  LL_USART_EnableDMAReq_RX(USART2);
}

void Configure_I2C(void) {
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

  LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_SYSCLK);

  /* Disable prior modifying configuration registers */
  LL_I2C_Disable(I2C1);

  /* Timing register value is computed with the STM32CubeMX Tool,
    * Standard Mode @100kHz with I2CCLK = 16 MHz,
    * rise time = 50ns, fall time = 10ns
    * Timing Value = (uint32_t)0x0020098E
    */
  #define I2C_TIMING                 __LL_I2C_CONVERT_TIMINGS(0x0, 0xF, 0x0, 0x2B, 0x86)

  /* Configure the SDA setup, hold time and the SCL high, low period */
  /* (uint32_t)0x0020098E = I2C_TIMING*/
  LL_I2C_SetTiming(I2C1, I2C_TIMING);

  /* INT PA6 (D12)*/
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_6, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_6, LL_GPIO_PULL_UP);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
#ifdef I2C_FLAG_EXTI
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_EXTI_LINE_6);
  LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_6);
  LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_6);
  NVIC_EnableIRQ(EXTI9_5_IRQn);
#endif // I2C_FLAG_EXTI

  /* SDA PB9 (D14) */
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_9, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(GPIOB, LL_GPIO_PIN_9, LL_GPIO_AF_4);
  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_9, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_9, LL_GPIO_OUTPUT_OPENDRAIN);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_9, LL_GPIO_PULL_UP);

  /* SCL PB8 (D15) */
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_8, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(GPIOB, LL_GPIO_PIN_8, LL_GPIO_AF_4);
  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_8, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_8, LL_GPIO_OUTPUT_OPENDRAIN);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_8, LL_GPIO_PULL_UP);

  LL_I2C_Enable(I2C1);
}

void Configure_ISO(void)
{
  /* DMA1 used for USART2 Transmission and Reception
   */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  LL_DMA_ConfigTransfer(DMA1, LL_DMA_CHANNEL_5,
            LL_DMA_DIRECTION_PERIPH_TO_MEMORY |
            LL_DMA_PRIORITY_HIGH              |
            LL_DMA_MODE_CIRCULAR              |
            LL_DMA_PERIPH_NOINCREMENT         |
            LL_DMA_MEMORY_INCREMENT           |
            LL_DMA_PDATAALIGN_BYTE            |
            LL_DMA_MDATAALIGN_BYTE);
  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_5,
             LL_USART_DMA_GetRegAddr(USART1, LL_USART_DMA_REG_DATA_RECEIVE),
             (uint32_t)iso_buffer,
             LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_5));
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_5, sizeof(iso_buffer));
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_5, LL_DMA_REQUEST_2);

  /* Enable DMA Channel Rx */
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_5);

  LL_USART_Disable(USART1);

  /* Configure Rx/Tx Pin as : Alternate function, High Speed, Push pull, Pull up */
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_8, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_8, LL_GPIO_AF_7);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_8, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_8, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_8, LL_GPIO_PULL_UP);

  /* Configure Clk Pin as : Alternate function, High Speed, Push pull, Pull up */
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_9, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_9, LL_GPIO_AF_7);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_9, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_9, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_9, LL_GPIO_PULL_UP);

  // ISO GND
  LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_6);
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_6, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_6, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_6, LL_GPIO_OUTPUT_PUSHPULL);

  // ISO RST
  LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_7);
  LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_7, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinSpeed(GPIOC, LL_GPIO_PIN_7, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(GPIOC, LL_GPIO_PIN_7, LL_GPIO_OUTPUT_PUSHPULL);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
  LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_SYSCLK);

  LL_USART_SetTransferDirection(USART1, LL_USART_DIRECTION_TX_RX);
  LL_USART_ConfigCharacter(USART1, LL_USART_DATAWIDTH_9B, LL_USART_PARITY_EVEN, LL_USART_STOPBITS_1_5);
  LL_USART_ConfigClock(USART1, LL_USART_PHASE_2EDGE, LL_USART_POLARITY_HIGH, LL_USART_LASTCLKPULSE_OUTPUT);
  LL_USART_SetOverSampling(USART1, LL_USART_OVERSAMPLING_8);
  LL_USART_SetRxTimeout(USART1, 0); // done in iso_recv
  LL_USART_EnableOneBitSamp(USART1);
  LL_USART_EnableSmartcard(USART1);
#define SC_CLOCK 5000000
#define PRESC (SystemCoreClock/2/(SC_CLOCK))
  LL_USART_SetSmartcardPrescaler(USART1, PRESC);
  SET_BIT(USART1->CR2, USART_CR2_CLKEN); //LL_USART_ConfigSyncMode(USART1); // thanks ST, it's clearing the SCEN, which is required to be set prior to setting CLKEN.

  USART1->ICR = 0xFFFFFFFF;
  iso_offset_read = 0;

  LL_USART_Enable(USART1);

  /* Polling USART initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(USART1))) || (!(LL_USART_IsActiveFlag_REACK(USART1))))
  {
  }
  /* Enable DMA RX Interrupt */
  LL_USART_EnableDMAReq_RX(USART1);
}

#ifdef CAN
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
#endif // CAN

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follows :
  *            System Clock source            = PLL (MSI)
  *            SYSCLK(Hz)                     = 80000000
  *            HCLK(Hz)                       = 80000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 4000000
  *            PLL_M                          = 1
  *            PLL_N                          = 40
  *            PLL_R                          = 2
  *            Flash Latency(WS)              = 4
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  /* MSI configuration and activation */
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
  LL_RCC_MSI_Enable();
  while(LL_RCC_MSI_IsReady() != 1)
  {
  };

  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_MSI, LL_RCC_PLLM_DIV_1, 40, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_Enable();
  LL_RCC_PLL_EnableDomain_SYS();
  while(LL_RCC_PLL_IsReady() != 1)
  {
  };

  /* Sysclk activation on the main PLL */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  };

  /* Set APB1 & APB2 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(80000000);

  /* Set systick to 1ms in using frequency set to 80MHz */
  /* This frequency can be calculated through LL RCC macro */
  /* ex: __LL_RCC_CALC_PLLCLK_FREQ(__LL_RCC_CALC_MSI_FREQ(LL_RCC_MSIRANGESEL_RUN, LL_RCC_MSIRANGE_6),
                                  LL_RCC_PLLM_DIV_1, 40, LL_RCC_PLLR_DIV_2)*/
  LL_Init1msTick(SystemCoreClock);
  SysTick_Config(SystemCoreClock/1000);
}

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* Configure the system clock to 80 MHz */
  SystemClock_Config();

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);

  /* Configure USARTx (USART IP configuration and related GPIO initialization) */
  Configure_USART();

  Configure_I2C();

  Configure_ISO();

#ifdef CAN
  Configure_CAN(500000, 0);
#endif // CAN

  /* Infinite loop */
  while (1)
  {
    interp();
  }
}
