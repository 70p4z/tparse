#include "main.h"

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
    uint32_t timeout = uwTick + TIMEOUT_1S;
    while (!LL_I2C_IsActiveFlag_RXNE(I2C1)) {
      if (uwTick - timeout < 0x80000000UL) {
        i2c_stop();
        return -3;
      }
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
    uint32_t timeout = uwTick + TIMEOUT_1S;
    while (!LL_I2C_IsActiveFlag_TXIS(I2C1)) {
      if (uwTick - timeout < 0x80000000UL) {
        i2c_stop();
        return -3;
      }
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
  if (! flag && !gpio_get(0, 9)) {
    // it sounds like we missed the EXTI !!
    flag = 1;
  }
  i2c_i_flag = 0;
  NVIC_EnableIRQ(EXTI9_5_IRQn);
  return flag;
}
#endif // I2C_FLAG_EXTI

void Configure_I2C1(void) {
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
  //#define I2C_TIMING            0x00200720 // 1mbps
  // #define I2C_TIMING            0x00200F33

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
