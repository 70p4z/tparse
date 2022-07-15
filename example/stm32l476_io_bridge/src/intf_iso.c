#include "main.h"
#include "iso7816.h"

char iso_buffer[MAX(256+2, 5+255)]; // max T=0 command/responselength
size_t iso_offset_read;

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

void Configure_USART1_ISO_CLK(uint32_t smartcard_clock)
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

  /* Configure Clk Pin as : Alternate function, High Speed, Push pull, Pull up */
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_8, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_8, LL_GPIO_AF_7);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_8, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_8, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_8, LL_GPIO_PULL_UP);

  /* Configure Rx/Tx Pin as : Alternate function, High Speed, Push pull, Pull up */
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
  // default clock when none provided
  if (smartcard_clock == 0) {
    smartcard_clock = 8000000;
  }
  LL_USART_SetSmartcardPrescaler(USART1, (SystemCoreClock/2/(smartcard_clock)));
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

void Configure_USART1_ISO(void) {
  // use default clock
  Configure_USART1_ISO_CLK(0);
}
