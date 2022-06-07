#pragma once

#include "stddef.h"
#include "stdint.h"

size_t iso_powercycle(uint8_t* atr, size_t atr_max_len);
size_t iso_powercycle_TA_1(uint8_t* atr, size_t atr_max_len, uint32_t TA_1);
void iso_power_down(void);
size_t iso_apdu_t0(uint8_t* apdu, size_t length);

// HAL
void iso_usart_ETU(uint32_t etu);
void iso_delay_ms(uint32_t ms);
void iso_usart_flush(void);
size_t iso_usart_recv(uint8_t* buffer, size_t length, uint32_t timeout);
void iso_usart_send(const uint8_t* buffer, size_t length);
void iso_rst(uint32_t level);
void iso_gnd(uint32_t level);
