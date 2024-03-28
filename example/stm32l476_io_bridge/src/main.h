/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "string.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_ll_dma.h"
#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_ll_usart.h"
#include "stm32l4xx_ll_rcc.h"
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_utils.h"
#include "stm32l4xx_ll_system.h"
#include "stm32l4xx_ll_i2c.h"
#include "stm32l4xx_ll_spi.h"
#include "stm32l4xx_ll_exti.h"

#ifndef MAX
#define MAX(x,y) ((x)>(y)?(x):(y))
#endif // MAX
#ifndef MIN
#define MIN(x,y) ((x)<(y)?(x):(y))
#endif // MIN

#ifndef __bswap_32
/* Swap bytes in 32 bit value.  */
#define __bswap_32(x) \
     ((((x) & 0xff000000) >> 24) | (((x) & 0x00ff0000) >>  8) |                      \
      (((x) & 0x0000ff00) <<  8) | (((x) & 0x000000ff) << 24))
#endif // __bswap_32

#define CPU_CLOCK 80000000
#define USART_BAUDRATE_USART2 921600
#define USART_BAUDRATE_USART3 921600
#define USART_BAUDRATE_UART4 9600
#define USART_BAUDRATE_UART5 115200
#define NO_TIMEOUT 0
#define TIMEOUT_1S 1000
#define CAN_FIFO_RX_ENTRY_COUNT 256

void Configure_USART2_USBVCP(uint32_t baudrate);
void Configure_USART3(uint32_t baudrate);
void Configure_UART4(uint32_t baudrate);
void Configure_UART5(uint32_t baudrate);
extern char uart_usbvcp_buffer[32+512*2];
extern char uart3_buffer[32+512*2];
extern char uart4_buffer[512];
void uart_send_mem(const void* _ptr, size_t len);
void uart_send(const char* string);
void uart_send_hex(const void* _buf, size_t len);
void uart_send_dma(const void* _buf, size_t len);
void uart_select_intf(USART_TypeDef* usart);
// rx is done through DMA, use the CNDTR value to detect were the data are

uint32_t gpio_get(uint32_t port, uint32_t pin);
void gpio_cfg_input(uint32_t port, uint32_t pin);
void gpio_set(uint32_t port, uint32_t pin, uint32_t value);

int32_t Configure_I2C1(uint32_t khz);
uint32_t i2c_strobe(uint32_t addr);
size_t i2c_read(uint8_t addr, uint8_t* buf, size_t maxlen);
size_t i2c_write(uint8_t addr, uint8_t* buf, size_t len);

void Configure_CAN(uint32_t frequency, uint32_t auto_retransmit);
#define CAN_ID_STANDARD_LEN 11
#define CAN_ID_EXTENDED_LEN 29
size_t can_fifo_avail(void);
size_t can_fifo_rx(uint32_t * id, size_t * id_bitlen, uint8_t* frame, size_t frame_max_len);
size_t can_tx(uint32_t id, size_t id_bitlen, uint8_t *frame, size_t frame_len);

int32_t Configure_SPI1(void);
size_t spi_xfer(uint8_t* buf, size_t len);

void Configure_USART1_ISO(void);
void Configure_USART1_ISO_CLK(uint32_t smartcard_clock);
#include "iso7816.h"

void interp(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
