#include "main.h"

const struct {
  uint8_t port;
  uint8_t pin;
} gpio_reserved[] = {
  {0, 2},  // STLINK USART TX (D1)
  {0, 3},  // STLINK USART RX (D0)
  {0, 13}, // STLINK SWD
  {0, 14}, // STLINK SWD
  //(0, 5),  // ISO SWP         (D13) =1 at boot, mutable by UART
  //{0, 6},  // I2C INT         (D12)
  {0, 8},  // ISO CLK         (D7)
  {0, 9},  // ISO IO          (D8)
  {1, 6},  // ISO GND         (D10) (SE POWER)
  {2, 7},  // ISO RST         (D9)
  {1, 8},  // I2C SCL         (D15)
  {1, 9},  // I2C SDA         (D14)
  //(1, 10),  // ISO SWP        (D6) =0 at boot, mutable by UART
  {0, 11}, // CAN RX          (CN10-14)
  {0, 12}, // CAN TX          (CN10-12)
  {2, 10},  // USART3 TX      (CN7-1)
  {2, 11},  // USART3 RX      (CN7-2)
};

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
