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

/* Private functions ---------------------------------------------------------*/

char uart_buffer[32+512];
uint8_t tmp[300];
char iso_buffer[MAX(256+2, 5+255)]; // max T=0 command/responselength
size_t iso_offset_write;
size_t iso_offset_read;

const struct {
	uint8_t port;
	uint8_t pin;
} gpio_reserved[] = {
	{0, 2},  // STLINK USART TX (D1)
	{0, 3},  // STLINK USART RX (D0)
	{0, 13}, // STLINK SWD
	{0, 14}, // STLINK SWD
	{0, 6},  // I2C INT     (D12)
	{0, 8},  // ISO CLK     (D7)
	{0, 9},  // ISO IO      (D8)
	{1, 6},  // ISO GND     (D10) (SE POWER)
	{2, 7},  // ISO RST     (D9)
	{1, 8},  // I2C SCL     (D15)
	{1, 9},  // I2C SDA     (D14)
};

#include "tparse.h"
#include "stddef.h"
#include "iso7816.h"

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

uint32_t i2c_i_flag;
void EXTI9_5_IRQHandler(void) {
	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_6);
	//NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
	NVIC_DisableIRQ(EXTI9_5_IRQn);
	i2c_i_flag = 1;
}

uint32_t i2c_consume_int(void) {
	// don't use EXTI->PRx register to avoid race condition
	// between read and clear and a external set
	__disable_irq();
	uint32_t flag = i2c_i_flag;
	i2c_i_flag = 0;
	__enable_irq();
	// avoid triggering it non stop in the background if the wire is floating
	NVIC_EnableIRQ(EXTI9_5_IRQn);
	return flag;
}

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
	iso_offset_read = iso_offset_write = 0;
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

void iso_usart_send(uint8_t* buffer, size_t length) {
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
      while ((USART1->ISR & (USART_ISR_TXE|USART_ISR_IDLE|USART_ISR_BUSY) ) != (USART_ISR_TXE|USART_ISR_IDLE) );

      // shall be done quickly to avoid the real reply to be concat to our sent data
      l = MIN(iso_usart_available(), MIN(sizeof(iso_buffer), length));

      iso_usart_recv(NULL, l, NO_TIMEOUT /*has been tested as available*/);
      length -= MIN(sizeof(iso_buffer), length);
    }
  }
}


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

	  const char * const cmds[] = {
			  "i2cr", "i2cw", "t0", "i2ci",
			  "gpo", "gpi", "atr", "off",
			  "info", "on", "i2ciwait", "i2cscan",
			  "reset",
	  };
	  len = tparse_has_line(&tp);
	  if (len) {
		  ts = sizeof(tmp);
		  cmd = tparse_token_in(&tp, (char**)cmds, sizeof(cmds)/sizeof(cmds[0]), (char*)tmp, &ts);
		  switch (cmd) {
		  default:
		  case 0:
			  uart_reply("ERROR: unsupported: ");
			  uart_reply_mem(tmp, ts);
			  uart_reply("\n");
			  break;
		  case 1:
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
		  case 2:
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
		  case 3:
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
		  case 4:
			  // I2C interrupt read
			  uart_reply("OK:");
			  uart_reply_hex((uint8_t*)(i2c_consume_int()?&"\x01":&"\x00"), 1);
			  uart_reply("\n");
			  break;
		  case 5:
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
		  case 6:
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
		  case 7:
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
		  case 8:
			  // OFF
			  iso_power_down();
			  uart_reply("OK:\n");
			  break;
		  case 9:
			  // info
			  uart_reply("INFO:");
			  len = tparse_token_hex(&tp, tmp, sizeof(tmp));
			  if (len) {
				  uart_reply_hex(tmp, len);
				  uart_reply(",");
			  }
			  uart_reply("VERSION=0.1\n");
			  break;
		  case 10:
			  // ON
			  iso_gnd(0);
			  uart_reply("OK:\n");
			  break;
		  case 11:
			  // i2ciwait
			  len = uwTick + 30*TIMEOUT_1S;
			  // until timeout
			  for(;;) {
				  if ((uwTick - len) < 0x80000000UL) {
					  uart_reply("TIMEOUT:\n");
					  break;
				  }
				  if (i2c_consume_int()) {
					  uart_reply("OK:\n");
					  break;
				  }
			  }
			  break;
		  case 12:
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
		  case 13:
			  uart_reply("OK:\n");
			  LL_mDelay(10);
			  NVIC_SystemReset();
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

  /* (1) Enable GPIO clock and configures the USART pins **********************/

  /* Enable the peripheral clock of GPIO Port */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);

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
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);

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
	LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_EXTI_LINE_6);
	LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_6);
	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_6);
	NVIC_EnableIRQ(EXTI9_5_IRQn);

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

  /* (1) Enable GPIO clock and configures the USART pins **********************/

  /* Enable the peripheral clock of GPIO Port */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);

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
  iso_offset_write = iso_offset_read = 0;

  LL_USART_Enable(USART1);

  /* Polling USART initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(USART1))) || (!(LL_USART_IsActiveFlag_REACK(USART1))))
  {
  }
  /* Enable DMA RX Interrupt */
  LL_USART_EnableDMAReq_RX(USART1);
}

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

  /* Configure USARTx (USART IP configuration and related GPIO initialization) */
  Configure_USART();

  Configure_I2C();

  Configure_ISO();

  /* Infinite loop */
  while (1)
  {
	  interp();
  }
}
