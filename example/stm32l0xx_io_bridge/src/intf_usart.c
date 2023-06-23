#include "main.h"

char uart_usbvcp_buffer[32+512*2];

USART_TypeDef* usart_intf;

void uart_select_intf(USART_TypeDef* usart) {
  usart_intf = usart;
}

void uart_send_mem(const void* _ptr, size_t len) {
  const uint8_t* ptr = (const uint8_t*) _ptr;
  while (LL_DMA_IsEnabledChannel(DMA1, LL_DMA_CHANNEL_7) && LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_7));
  LL_USART_DisableDMAReq_TX(USART2);
  while (len--) {
    while (!LL_USART_IsActiveFlag_TXE(usart_intf)){}
    LL_USART_TransmitData8(usart_intf, *ptr++);
  }
}

void uart_send(const char* string) {
  uart_send_mem((uint8_t*)string, strlen(string));
}

uint8_t n2h(uint8_t c) {
  if (c<10) {
    return c+'0';
  }
  else {
    return c+'a'-10;
  }
}

void uart_send_hex(const void* _buf, size_t len) {
  const uint8_t* buf = (const uint8_t*)_buf;
  char c[2];
  c[1] = 0;
  while (len--) {
    c[0] = n2h(*buf>>4);
    uart_send(c);
    c[0] = n2h(*buf&0xF);
    uart_send(c);
    buf++;
  }
}

void uart_send_dma(const void* buf, size_t len) {
  // wait until previous DMA transfer ended
  while (LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_7));
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_7);
  LL_USART_EnableDMAReq_TX(USART2);
  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_7,
             (uint32_t)buf,
             LL_USART_DMA_GetRegAddr(USART2, LL_USART_DMA_REG_DATA_TRANSMIT),
             LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_7));
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_7, len);
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_7);
}

void Configure_USART2_USBVCP(uint32_t baudrate)
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
             (uint32_t)uart_usbvcp_buffer,
             LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_6));
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_6, sizeof(uart_usbvcp_buffer));
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_6, LL_DMA_REQUEST_4);
  /* Enable DMA Channel Rx */
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_6);

  LL_DMA_ConfigTransfer(DMA1, LL_DMA_CHANNEL_7,
            LL_DMA_DIRECTION_MEMORY_TO_PERIPH |
            LL_DMA_PRIORITY_HIGH              |
            LL_DMA_MODE_NORMAL                |
            LL_DMA_PERIPH_NOINCREMENT         |
            LL_DMA_MEMORY_INCREMENT           |
            LL_DMA_PDATAALIGN_BYTE            |
            LL_DMA_MDATAALIGN_BYTE);
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_7, LL_DMA_REQUEST_4);

  /* Configure Tx Pin as : Alternate function, High Speed, Push pull, Pull up */
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_2, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_2, LL_GPIO_AF_4);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_2, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_2, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_2, LL_GPIO_PULL_UP);

  /* Configure Rx Pin as : Alternate function, High Speed, Push pull, Pull up */
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_3, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_3, LL_GPIO_AF_4);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_3, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_3, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_3, LL_GPIO_PULL_UP);

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


  /* Set Baudrate to 921600 using APB frequency set to 32000000 Hz */
  /* Frequency available for USART peripheral can also be calculated through LL RCC macro */
  /* Ex :
      Periphclk = LL_RCC_GetUSARTClockFreq(Instance); or LL_RCC_GetUARTClockFreq(Instance); depending on USART/UART instance

      In this example, Peripheral Clock is expected to be equal to 32000000 Hz => equal to SystemCoreClock
  */

  /* Oversampling by 16 */
  /* Reset value is LL_USART_OVERSAMPLING_16 */
  // LL_USART_SetOverSampling(USART2, LL_USART_OVERSAMPLING_16);
  LL_USART_SetBaudRate(USART2, SystemCoreClock, LL_USART_OVERSAMPLING_16, baudrate);

  /* (4) Enable USART2 **********************************************************/
  LL_USART_Enable(USART2);

  /* Polling USART initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(USART2))) || (!(LL_USART_IsActiveFlag_REACK(USART2))))
  {
  }

  /* Enable DMA RX Interrupt */
  LL_USART_EnableDMAReq_RX(USART2);
  LL_USART_DisableDMAReq_TX(USART2);
}

