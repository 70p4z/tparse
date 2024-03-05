#include "main.h"


size_t spi_xfer(uint8_t* buf, size_t len) {
  size_t l = len;
  while (l--) {
    while (!LL_SPI_IsActiveFlag_TXE(SPI1));
    LL_SPI_TransmitData8(SPI1, *buf);
    while (!LL_SPI_IsActiveFlag_RXNE(SPI1));
    *buf++ = LL_SPI_ReceiveData8(SPI1);
  }
  return len;
}

int32_t Configure_SPI1(void) {
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);

  /* (1) Enables GPIO clock and configures the SPI1 pins ********************/
  /* Enable the peripheral clock of GPIOB */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);

  /* Configure SCK Pin connected to pin 31 of CN10 connector */
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_3, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_3, LL_GPIO_AF_5);
  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_3, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_3, LL_GPIO_PULL_DOWN);

  /* Configure MISO Pin connected to pin 27 of CN10 connector */
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_4, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_4, LL_GPIO_AF_5);
  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_4, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_4, LL_GPIO_PULL_DOWN);

  /* Configure MOSI Pin connected to pin 29 of CN10 connector */
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_5, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_5, LL_GPIO_AF_5);
  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_5, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_5, LL_GPIO_PULL_DOWN);

  LL_SPI_Disable(SPI1);

  /* Configure SPI1 communication */
  LL_SPI_SetBaudRatePrescaler(SPI1, LL_SPI_BAUDRATEPRESCALER_DIV32);
  LL_SPI_SetTransferDirection(SPI1, LL_SPI_FULL_DUPLEX);
  // this combination won't generate start conditions when muxed with i2c
  LL_SPI_SetClockPhase(SPI1, LL_SPI_PHASE_1EDGE);
  LL_SPI_SetClockPolarity(SPI1, LL_SPI_POLARITY_LOW);
  /* Reset value is LL_SPI_MSB_FIRST */
  LL_SPI_SetTransferBitOrder(SPI1, LL_SPI_MSB_FIRST);
  LL_SPI_SetDataWidth(SPI1, LL_SPI_DATAWIDTH_8BIT);
  LL_SPI_SetNSSMode(SPI1, LL_SPI_NSS_SOFT);
  LL_SPI_SetRxFIFOThreshold(SPI1, LL_SPI_RX_FIFO_TH_QUARTER);
  LL_SPI_SetMode(SPI1, LL_SPI_MODE_MASTER);

  LL_SPI_Enable(SPI1);

  return 0;
}
