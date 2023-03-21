#include "main.h"

#define USART_USBVCP USART3
#define USART_USBVCP_DMA_PERIPH DMA1
#define USART_USBVCP_DMA_CHAN_TX LL_DMA_STREAM_3
#define USART_USBVCP_DMA_REQ_TX LL_DMA_CHANNEL_4
#define USART_USBVCP_DMA_CHAN_RX LL_DMA_STREAM_1
#define USART_USBVCP_DMA_REQ_RX LL_DMA_CHANNEL_4

char uart_usbvcp_buffer[32+512*2];
char uart3_buffer[32+512*2];
char uart4_buffer[1024];

USART_TypeDef* usart_intf;

void uart_select_intf(USART_TypeDef* usart) {
  usart_intf = usart;
}

void uart_send_mem(const void* _ptr, size_t len) {
  const uint8_t* ptr = (const uint8_t*) _ptr;
  while (LL_DMA_IsEnabledStream(USART_USBVCP_DMA_PERIPH, USART_USBVCP_DMA_CHAN_TX) 
    && LL_DMA_GetDataLength(USART_USBVCP_DMA_PERIPH, USART_USBVCP_DMA_CHAN_TX));
  LL_USART_DisableDMAReq_TX(USART_USBVCP);
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
  while (LL_DMA_GetDataLength(USART_USBVCP_DMA_PERIPH, USART_USBVCP_DMA_CHAN_TX));
  LL_DMA_DisableStream(USART_USBVCP_DMA_PERIPH, USART_USBVCP_DMA_CHAN_TX);
  LL_USART_EnableDMAReq_TX(USART_USBVCP);
  LL_DMA_ConfigAddresses(USART_USBVCP_DMA_PERIPH, USART_USBVCP_DMA_CHAN_TX,
             (uint32_t)buf,
             LL_USART_DMA_GetRegAddr(USART_USBVCP, LL_USART_DMA_REG_DATA_TRANSMIT),
             LL_DMA_GetDataTransferDirection(USART_USBVCP_DMA_PERIPH, USART_USBVCP_DMA_CHAN_TX));
  LL_DMA_SetDataLength(USART_USBVCP_DMA_PERIPH, USART_USBVCP_DMA_CHAN_TX, len);
  LL_DMA_EnableStream(USART_USBVCP_DMA_PERIPH, USART_USBVCP_DMA_CHAN_TX);
}

void Configure_USBVCP(uint32_t baudrate)
{
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  /* RX: DMA1 Stream1 Channel 4
     TX: DMA1 Stream3 Channel 4
   */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);


  LL_DMA_ConfigTransfer(USART_USBVCP_DMA_PERIPH, USART_USBVCP_DMA_CHAN_RX,
            LL_DMA_DIRECTION_PERIPH_TO_MEMORY |
            LL_DMA_PRIORITY_HIGH              |
            LL_DMA_MODE_CIRCULAR              |
            LL_DMA_PERIPH_NOINCREMENT         |
            LL_DMA_MEMORY_INCREMENT           |
            LL_DMA_PDATAALIGN_BYTE            |
            LL_DMA_MDATAALIGN_BYTE);
  LL_DMA_ConfigAddresses(USART_USBVCP_DMA_PERIPH, USART_USBVCP_DMA_CHAN_RX,
             LL_USART_DMA_GetRegAddr(USART_USBVCP, LL_USART_DMA_REG_DATA_RECEIVE),
             (uint32_t)uart_usbvcp_buffer,
             LL_DMA_GetDataTransferDirection(USART_USBVCP_DMA_PERIPH, USART_USBVCP_DMA_CHAN_RX));
  LL_DMA_SetDataLength(USART_USBVCP_DMA_PERIPH, USART_USBVCP_DMA_CHAN_RX, sizeof(uart_usbvcp_buffer));
  LL_DMA_SetChannelSelection(USART_USBVCP_DMA_PERIPH, USART_USBVCP_DMA_CHAN_RX, USART_USBVCP_DMA_REQ_RX);
  /* Enable DMA Channel Rx */
  LL_DMA_EnableStream(USART_USBVCP_DMA_PERIPH, USART_USBVCP_DMA_CHAN_RX);

  LL_DMA_ConfigTransfer(USART_USBVCP_DMA_PERIPH, USART_USBVCP_DMA_CHAN_TX,
            LL_DMA_DIRECTION_MEMORY_TO_PERIPH |
            LL_DMA_PRIORITY_HIGH              |
            LL_DMA_MODE_NORMAL                |
            LL_DMA_PERIPH_NOINCREMENT         |
            LL_DMA_MEMORY_INCREMENT           |
            LL_DMA_PDATAALIGN_BYTE            |
            LL_DMA_MDATAALIGN_BYTE);
  LL_DMA_SetChannelSelection(USART_USBVCP_DMA_PERIPH, USART_USBVCP_DMA_CHAN_TX, USART_USBVCP_DMA_REQ_TX);

  /* Configure Tx Pin as : Alternate function, High Speed, Push pull, Pull up */
  LL_GPIO_SetPinMode(GPIOD, LL_GPIO_PIN_8, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(GPIOD, LL_GPIO_PIN_8, LL_GPIO_AF_7);
  LL_GPIO_SetPinSpeed(GPIOD, LL_GPIO_PIN_8, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(GPIOD, LL_GPIO_PIN_8, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(GPIOD, LL_GPIO_PIN_8, LL_GPIO_PULL_UP);

  /* Configure Rx Pin as : Alternate function, High Speed, Push pull, Pull up */
  LL_GPIO_SetPinMode(GPIOD, LL_GPIO_PIN_9, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(GPIOD, LL_GPIO_PIN_9, LL_GPIO_AF_7);
  LL_GPIO_SetPinSpeed(GPIOD, LL_GPIO_PIN_9, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(GPIOD, LL_GPIO_PIN_9, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(GPIOD, LL_GPIO_PIN_9, LL_GPIO_PULL_UP);

  /* Set clock source */
  LL_RCC_SetUSARTClockSource(LL_RCC_USART3_CLKSOURCE_PCLK1);

  /* (3) Configure USART functional parameters ********************************/

  /* Disable USART prior modifying configuration registers */
  /* Note: Commented as corresponding to Reset value */
  // LL_USART_Disable(USART_USBVCP);

  /* TX/RX direction */
  LL_USART_SetTransferDirection(USART_USBVCP, LL_USART_DIRECTION_TX_RX);

  /* 8 data bit, 1 start bit, 1 stop bit, no parity */
  LL_USART_ConfigCharacter(USART_USBVCP, LL_USART_DATAWIDTH_8B, LL_USART_PARITY_NONE, LL_USART_STOPBITS_1);

  /* No Hardware Flow control */
  /* Reset value is LL_USART_HWCONTROL_NONE */
  // LL_USART_SetHWFlowCtrl(USART_USBVCP, LL_USART_HWCONTROL_NONE);

  /* Oversampling by 16 */
  /* Reset value is LL_USART_OVERSAMPLING_16 */
  // LL_USART_SetOverSampling(USART_USBVCP, LL_USART_OVERSAMPLING_16);

  /* Set Baudrate to 115200 using APB frequency set to 80000000 Hz */
  /* Frequency available for USART peripheral can also be calculated through LL RCC macro */
  /* Ex :
      Periphclk = LL_RCC_GetUSARTClockFreq(Instance); or LL_RCC_GetUARTClockFreq(Instance); depending on USART/UART instance

      In this example, Peripheral Clock is expected to be equal to 80000000 Hz => equal to SystemCoreClock
  */
  LL_USART_SetBaudRate(USART_USBVCP, SystemCoreClock, LL_USART_OVERSAMPLING_16, baudrate);

  /* (4) Enable USART **********************************************************/
  LL_USART_Enable(USART_USBVCP);

  /* Polling USART initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(USART_USBVCP))) || (!(LL_USART_IsActiveFlag_REACK(USART_USBVCP))))
  {
  }

  /* Enable DMA RX Interrupt */
  LL_USART_EnableDMAReq_RX(USART_USBVCP);
  LL_USART_DisableDMAReq_TX(USART_USBVCP);
}

#if 0
void Configure_USART3(uint32_t baudrate)
{
  /* DMA1 used for USART3 Transmission and Reception
   */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  LL_DMA_ConfigTransfer(DMA1, LL_DMA_CHANNEL_3,
            LL_DMA_DIRECTION_PERIPH_TO_MEMORY |
            LL_DMA_PRIORITY_HIGH              |
            LL_DMA_MODE_CIRCULAR              |
            LL_DMA_PERIPH_NOINCREMENT         |
            LL_DMA_MEMORY_INCREMENT           |
            LL_DMA_PDATAALIGN_BYTE            |
            LL_DMA_MDATAALIGN_BYTE);
  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_3,
             LL_USART_DMA_GetRegAddr(USART3, LL_USART_DMA_REG_DATA_RECEIVE),
             (uint32_t)uart3_buffer,
             LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3));
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, sizeof(uart3_buffer));
  LL_DMA_SetChannelSelection(DMA1, LL_DMA_CHANNEL_3, LL_DMA_REQUEST_2);

  /* Enable DMA Channel Rx */
  LL_DMA_EnableStream(DMA1, LL_DMA_CHANNEL_3);

  /* Configure Tx Pin as : Alternate function, High Speed, Push pull, Pull up */
  LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_10, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(GPIOC, LL_GPIO_PIN_10, LL_GPIO_AF_7);
  LL_GPIO_SetPinSpeed(GPIOC, LL_GPIO_PIN_10, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(GPIOC, LL_GPIO_PIN_10, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_10, LL_GPIO_PULL_UP);

  /* Configure Rx Pin as : Alternate function, High Speed, Push pull, Pull up */
  LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_11, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(GPIOC, LL_GPIO_PIN_11, LL_GPIO_AF_7);
  LL_GPIO_SetPinSpeed(GPIOC, LL_GPIO_PIN_11, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(GPIOC, LL_GPIO_PIN_11, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_11, LL_GPIO_PULL_UP);

  /* (2) Enable USART peripheral clock and clock source ****************/
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);

  /* Set clock source */
  LL_RCC_SetUSARTClockSource(LL_RCC_USART3_CLKSOURCE_PCLK1);

  /* (3) Configure USART functional parameters ********************************/

  /* Disable USART prior modifying configuration registers */
  /* Note: Commented as corresponding to Reset value */
  // LL_USART_Disable(USART3);

  /* TX/RX direction */
  LL_USART_SetTransferDirection(USART3, LL_USART_DIRECTION_TX_RX);

  /* 8 data bit, 1 start bit, 1 stop bit, no parity */
  LL_USART_ConfigCharacter(USART3, LL_USART_DATAWIDTH_8B, LL_USART_PARITY_NONE, LL_USART_STOPBITS_1);

  /* No Hardware Flow control */
  /* Reset value is LL_USART_HWCONTROL_NONE */
  // LL_USART_SetHWFlowCtrl(USART3, LL_USART_HWCONTROL_NONE);

  /* Oversampling by 16 */
  /* Reset value is LL_USART_OVERSAMPLING_16 */
  // LL_USART_SetOverSampling(USART3, LL_USART_OVERSAMPLING_16);

  /* Set Baudrate to 115200 using APB frequency set to 80000000 Hz */
  /* Frequency available for USART peripheral can also be calculated through LL RCC macro */
  /* Ex :
      Periphclk = LL_RCC_GetUSARTClockFreq(Instance); or LL_RCC_GetUARTClockFreq(Instance); depending on USART/UART instance

      In this example, Peripheral Clock is expected to be equal to 80000000 Hz => equal to SystemCoreClock
  */
  LL_USART_SetBaudRate(USART3, SystemCoreClock, LL_USART_OVERSAMPLING_16, baudrate);

  /* (4) Enable USART **********************************************************/
  LL_USART_Enable(USART3);

  /* Polling USART initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(USART3))) || (!(LL_USART_IsActiveFlag_REACK(USART3))))
  {
  }

  /* Enable DMA RX Interrupt */
  LL_USART_EnableDMAReq_RX(USART3);
  LL_USART_DisableDMAReq_TX(USART3);
}
#endif

void Configure_UART4(uint32_t baudrate)
{
  /* DMA used for USART4 Transmission and Reception
   * RX: DMA1 Stream 2 Channel 4
   */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  LL_DMA_ConfigTransfer(DMA1, LL_DMA_STREAM_2,
            LL_DMA_DIRECTION_PERIPH_TO_MEMORY |
            LL_DMA_PRIORITY_HIGH              |
            LL_DMA_MODE_CIRCULAR              |
            LL_DMA_PERIPH_NOINCREMENT         |
            LL_DMA_MEMORY_INCREMENT           |
            LL_DMA_PDATAALIGN_BYTE            |
            LL_DMA_MDATAALIGN_BYTE);
  LL_DMA_ConfigAddresses(DMA1, LL_DMA_STREAM_2,
             LL_USART_DMA_GetRegAddr(UART4, LL_USART_DMA_REG_DATA_RECEIVE),
             (uint32_t)uart4_buffer,
             LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_STREAM_2));
  LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_2, sizeof(uart4_buffer));
  LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_2, LL_DMA_CHANNEL_4);

  /* Enable DMA Channel Rx */
  LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_2);

  /* Configure Tx Pin as : Alternate function, High Speed, Push pull, Pull up */
  LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_10, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(GPIOC, LL_GPIO_PIN_10, LL_GPIO_AF_8);
  LL_GPIO_SetPinSpeed(GPIOC, LL_GPIO_PIN_10, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(GPIOC, LL_GPIO_PIN_10, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_10, LL_GPIO_PULL_UP);

  /* Configure Rx Pin as : Alternate function, High Speed, Push pull, Pull up */
  LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_11, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(GPIOC, LL_GPIO_PIN_11, LL_GPIO_AF_8);
  LL_GPIO_SetPinSpeed(GPIOC, LL_GPIO_PIN_11, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(GPIOC, LL_GPIO_PIN_11, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_11, LL_GPIO_PULL_UP);

  /* (2) Enable USART peripheral clock and clock source ****************/
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART4);

  /* Set clock source */
  LL_RCC_SetUSARTClockSource(LL_RCC_UART4_CLKSOURCE_PCLK1);

  /* (3) Configure USART functional parameters ********************************/

  /* Disable USART prior modifying configuration registers */
  /* Note: Commented as corresponding to Reset value */
  // LL_USART_Disable(USART4);

  /* TX/RX direction */
  LL_USART_SetTransferDirection(UART4, LL_USART_DIRECTION_TX_RX);

  /* 8 data bit, 1 start bit, 1 stop bit, no parity */
  LL_USART_ConfigCharacter(UART4, LL_USART_DATAWIDTH_8B, LL_USART_PARITY_NONE, LL_USART_STOPBITS_1);

  /* No Hardware Flow control */
  /* Reset value is LL_USART_HWCONTROL_NONE */
  // LL_USART_SetHWFlowCtrl(USART4, LL_USART_HWCONTROL_NONE);

  /* Oversampling by 16 */
  /* Reset value is LL_USART_OVERSAMPLING_16 */
  // LL_USART_SetOverSampling(USART4, LL_USART_OVERSAMPLING_16);

  /* Set Baudrate to 115200 using APB frequency set to 80000000 Hz */
  /* Frequency available for USART peripheral can also be calculated through LL RCC macro */
  /* Ex :
      Periphclk = LL_RCC_GetUSARTClockFreq(Instance); or LL_RCC_GetUARTClockFreq(Instance); depending on USART/UART instance

      In this example, Peripheral Clock is expected to be equal to 80000000 Hz => equal to SystemCoreClock
  */
  LL_USART_SetBaudRate(UART4, SystemCoreClock, LL_USART_OVERSAMPLING_16, baudrate);

  /* (4) Enable USART **********************************************************/
  LL_USART_Enable(UART4);

  /* Polling USART initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(UART4))) || (!(LL_USART_IsActiveFlag_REACK(UART4))))
  {
  }

  /* Enable DMA RX Interrupt */
  LL_USART_EnableDMAReq_RX(UART4);
  LL_USART_DisableDMAReq_TX(UART4);
}

void Configure_UART5(uint32_t baudrate)
{
  // NO DMA for UART5

  /* Configure Tx Pin as : Alternate function, High Speed, Push pull, Pull up */
  LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_12, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(GPIOC, LL_GPIO_PIN_12, LL_GPIO_AF_8);
  LL_GPIO_SetPinSpeed(GPIOC, LL_GPIO_PIN_12, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(GPIOC, LL_GPIO_PIN_12, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_12, LL_GPIO_PULL_UP);

  /* Configure Rx Pin as : Alternate function, High Speed, Push pull, Pull up */
  LL_GPIO_SetPinMode(GPIOD, LL_GPIO_PIN_2, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7(GPIOD, LL_GPIO_PIN_2, LL_GPIO_AF_8);
  LL_GPIO_SetPinSpeed(GPIOD, LL_GPIO_PIN_2, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(GPIOD, LL_GPIO_PIN_2, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(GPIOD, LL_GPIO_PIN_2, LL_GPIO_PULL_UP);

  /* (2) Enable USART peripheral clock and clock source ****************/
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART5);

  /* Set clock source */
  LL_RCC_SetUSARTClockSource(LL_RCC_UART5_CLKSOURCE_PCLK1);

  /* (3) Configure USART functional parameters ********************************/

  /* Disable USART prior modifying configuration registers */
  /* Note: Commented as corresponding to Reset value */
  // LL_USART_Disable(USART4);

  /* TX/RX direction */
  LL_USART_SetTransferDirection(UART5, LL_USART_DIRECTION_TX_RX);

  /* 8 data bit, 1 start bit, 1 stop bit, no parity */
  LL_USART_ConfigCharacter(UART5, LL_USART_DATAWIDTH_8B, LL_USART_PARITY_NONE, LL_USART_STOPBITS_1);

  /* No Hardware Flow control */
  /* Reset value is LL_USART_HWCONTROL_NONE */
  // LL_USART_SetHWFlowCtrl(USART4, LL_USART_HWCONTROL_NONE);

  /* Oversampling by 16 */
  /* Reset value is LL_USART_OVERSAMPLING_16 */
  // LL_USART_SetOverSampling(USART4, LL_USART_OVERSAMPLING_16);

  /* Set Baudrate to 115200 using APB frequency set to 80000000 Hz */
  /* Frequency available for USART peripheral can also be calculated through LL RCC macro */
  /* Ex :
      Periphclk = LL_RCC_GetUSARTClockFreq(Instance); or LL_RCC_GetUARTClockFreq(Instance); depending on USART/UART instance

      In this example, Peripheral Clock is expected to be equal to 80000000 Hz => equal to SystemCoreClock
  */
  LL_USART_SetBaudRate(UART5, SystemCoreClock, LL_USART_OVERSAMPLING_16, baudrate);

  /* (4) Enable USART **********************************************************/
  LL_USART_Enable(UART5);

  /* Polling USART initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(UART5))) || (!(LL_USART_IsActiveFlag_REACK(UART5))))
  {
  }

  /* Enable DMA RX Interrupt */
  LL_USART_DisableDMAReq_RX(UART5);
  LL_USART_DisableDMAReq_TX(UART5);
}
