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
#include "tparse.h"
#include "stddef.h"

/* Private functions ---------------------------------------------------------*/

uint8_t tmp[512+32];

uint32_t tparse_al_time(void) {
  return uwTick; /* todo bind the systick count here */;
}

extern uint8_t n2h(uint8_t c);

tparse_ctx_t tp_vcp;
tparse_ctx_t tp_u3;

tparse_ctx_t* tparse_check_command(void) {
  tparse_finger(&tp_vcp, sizeof(uart_usbvcp_buffer) - DMA1_Channel6->CNDTR);
  tparse_finger(&tp_u3, sizeof(uart3_buffer) - DMA1_Channel3->CNDTR);
  uint32_t len;
  tparse_ctx_t *tp = NULL;
  uart_select_intf(USART2); // default to send on pc (for background tasks)
  len = tparse_has_line(&tp_vcp);
  if (len) {
    tp = &tp_vcp;
    uart_select_intf(USART2);
  }
  else {
    len = tparse_has_line(&tp_u3);
    if (len) {
      uart_select_intf(USART3);
      tp = &tp_u3;
    }
  }
  return tp;
}

__attribute__((weak)) void interp(void) {
  uint32_t previous_sw;
  uint8_t sw[2];
  uint32_t flags= 0;
  #define FLAGS_CAN_INTRX 1

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  tparse_init(&tp_vcp, uart_usbvcp_buffer, sizeof(uart_usbvcp_buffer), " \n");
  tparse_init(&tp_u3, uart3_buffer, sizeof(uart3_buffer), " \n");

  // set one PIN in each level at startup, to allow for specific reset condition
  // (device factory reset for example). Those pins are switchable but have a default
  // and known default value
  gpio_set(0, 5, 1);
  gpio_set(1, 10, 0);

  uart_select_intf(USART3);
  uart_send("RESET:\n");
  uart_select_intf(USART2);
  uart_send("RESET:\n");

  previous_sw=0; // no previous t0c

  while (1)
  {
    uint32_t cmd;
    uint32_t addr;
    uint32_t len;
    uint32_t port;
    uint32_t pin;
    uint32_t val;
    uint32_t tlen;
    uint32_t timeout;
    uint32_t t;
    size_t ts,ts2;

    // check flags for interrupt mode
    if (flags & FLAGS_CAN_INTRX) {
      if (can_fifo_avail()) {
        // can rx
        len = can_fifo_rx(&addr, &ts, tmp, sizeof(tmp));
        if (ts == 0) {
          uart_send("ERROR: no frame available\n");
          break;
        }
        uart_send("INTCRX:");
        addr = __bswap_32(addr); // change endianess (LE cpu => BE uart)
        uart_send_hex(&addr, 4);
        switch(ts) {
          case CAN_ID_STANDARD_LEN:
            uart_send(",s,");
            break;
          case CAN_ID_EXTENDED_LEN:
            uart_send(",e,");
            break;
          default:
            uart_send(",U,");
            break;
        }
        uart_send_hex(tmp, len);
        uart_send("\n");
      }
    }

    tparse_ctx_t *tp = tparse_check_command();

    if (tp) {
      ts = sizeof(tmp);
      static const char * const cmds[] = {
          "i2cs", "i2cr", "i2cw", 
          "t0", "t0c", "t0clast", 
          "i2ci",
          "gpo", "gpi", "cfgi", "atr", "off",
          "info", "on", "i2ciwait", "i2cscan",
          "reset",
          "ctx", "crx", "cavail", "ccfg",
          "i2cfg", "isocfg",
          "i2cwc", "i2cwclast",
          "i2crxfer", "spix", "rel",
      };
      cmd = tparse_token_in(tp, (char**)cmds, sizeof(cmds)/sizeof(cmds[0]), (char*)tmp, &ts);
      switch (cmd) {
      default:
      case __COUNTER__:
        uart_send("ERROR: unsupported: ");
        ts = MIN(ts, 32);
        uart_send_mem(tmp, ts);
        uart_send("\n");
        break;
      case __COUNTER__:
        // I2C strobe
        addr = tparse_token_u32(tp);
        if (addr >= 0x100 || addr == -1) {
          uart_send("ERROR: invalid address\n");
          break;
        }
        i2c_strobe(addr);
        uart_send("OK:");
        uart_send("\n");
        break;

      case __COUNTER__:
        // I2C read
        addr = tparse_token_u32(tp);
        if (addr >= 0x100 || addr == -1) {
          uart_send("ERROR: invalid address\n");
          break;
        }
        len = tparse_token_u32(tp);
        if (len < 0) {
          uart_send("ERROR: invalid length\n");
          break;
        }
        // optional retry count
        ts = 0;
        if (tparse_token_size(tp)) {
          ts = tparse_token_u32(tp);
        }
        len = MIN(len, sizeof(tmp));
        // until timeout)
      read_again:
        val = i2c_read(addr, tmp, len);
        if (len != val) {
          if (ts--) {
            goto read_again;
          }
          uart_send("ERROR: not all bytes read: ");
          uart_send_hex((uint8_t*)&val, 4);
          uart_send("\n");
          break;
        }
        uart_send("OK:");
        uart_send_hex(tmp, len);
        uart_send("\n");
        break;
      case __COUNTER__:
        // I2C write
        // i2cw <addr> <data> [<retrycount>]
        addr = tparse_token_u32(tp);
        if (addr >= 0x100 || addr == -1) {
          uart_send("ERROR: invalid address\n");
          break;
        }
        len = tparse_token_hex(tp, tmp, sizeof(tmp));
        if (len == 0) {
          uart_send("ERROR: invalid data\n");
          break;
        }
        // optional retry count
        ts = 0;
        if (tparse_token_size(tp)) {
          ts = tparse_token_u32(tp);
        }
      write_again:
        val = i2c_write(addr, tmp, len);
        if (len != val) {
          if (ts--) {
            goto write_again;
          }
          uart_send("ERROR: not all bytes written: ");
          uart_send_hex((uint8_t*)&val, 4);
          uart_send("\n");
          break;
        }
        uart_send("OK:");
        // len written
        len=__bswap_32(len);
        uart_send_hex(&len, 4);
        uart_send("\n");
        break;
      case __COUNTER__:
        // T=0 APDU
        len = tparse_token_hex(tp, tmp, sizeof(tmp));
        if (len > 255+5 || len < 5) {
          uart_send("ERROR: invalid T=0 apdu\n");
          break;
        }
        if (len > 5 && len - 5 != tmp[4]) {
          uart_send("ERROR: malformed apdu: ");
          uart_send_hex(tmp, len);
          uart_send("\n");
          break;
        }
        len = iso_apdu_t0(tmp, len);
        uart_send("OK:");
        uart_send_hex(tmp, len);
        uart_send("\n");
        break;
      case __COUNTER__:
        // t0c
        if (previous_sw) {
          uart_send("OK:");
          uart_send_hex(sw, 2);
          uart_send("\n");
          previous_sw = 0;
        }
        else {
          // fake reply to speedup
          uart_send("OK:9000\n");
        }
        // T0 APDU in cache (to allow for next APDU to be transferred while transferring to the SE)
        len = tparse_token_hex(tp, tmp, sizeof(tmp));
        if (len > 255+5 || len < 5) {
          uart_send("ERROR: invalid T=0 apdu\n");
          break;
        }
        if (len > 5 && len - 5 != tmp[4]) {
          uart_send("ERROR: malformed apdu: ");
          uart_send_hex(tmp, len);
          uart_send("\n");
          break;
        }
        len = iso_apdu_t0(tmp, len);
        if (len > 2) {
          // won't be retained, error?
          uart_send("ERROR: more than a SW returned, unsupported by t0c");
          uart_send("\n");
        }
        if (len == 2) {
          memmove(sw, tmp+len-2, 2);
          previous_sw = 1;
        }
        else {
          uart_send("ERROR: T0 APDU exchange failed");
          // faked status word for next fetch, to detect timeout/error
          memmove(sw, "\x6E\xEE", 2);
          previous_sw = 1;
        }
        break;
      case __COUNTER__:
        // t0clast: retrieve the last t0c reply from the smartcard
        if (previous_sw) {
          uart_send("OK:");
          uart_send_hex(sw, 2);
          uart_send("\n");
          previous_sw = 0;
        }
        else {
          uart_send("ERROR: last SW already returned or t0c not used");
          uart_send("\n");
        }
        break;
      case __COUNTER__:
        // I2C interrupt read
        uart_send("OK:");
#ifdef I2C_FLAG_EXTI
        uart_send_hex((uint8_t*)(i2c_consume_int()?&"\x01":&"\x00"), 1);
#else // I2C_FLAG_EXTI
        uart_send_hex((uint8_t*)(!gpio_get(0, 9)?&"\x01":&"\x00"), 1);
#endif // I2C_FLAG_EXTI
        uart_send("\n");
        break;
      case __COUNTER__:
        // GPO
        port = tparse_token_u32(tp);
        if (port > 7) {
          uart_send("ERROR: invalid port\n");
          break;
        }
        pin = tparse_token_u32(tp);
        if (pin > 15) {
          uart_send("ERROR: invalid pin\n");
          break;
        }
        val = tparse_token_u32(tp);
        if (val == -1) {
          uart_send("ERROR: invalid value\n");
          break;
        }
        gpio_set(port, pin, val);
        uart_send("OK:\n");
        break;
      case __COUNTER__:
        // GPI
        port = tparse_token_u32(tp);
        if (port > 7) {
          uart_send("ERROR: invalid port\n");
          break;
        }
        pin = tparse_token_u32(tp);
        if (pin > 15) {
          uart_send("ERROR: invalid pin\n");
          break;
        }
        val = gpio_get(port, pin);
        uart_send("OK:");
        val = val?1:0;
        uart_send_hex((uint8_t*)&val, 1);
        uart_send("\n");
        break;
      case __COUNTER__:
        // cfgi
        port = tparse_token_u32(tp);
        if (port > 7) {
          uart_send("ERROR: invalid port\n");
          break;
        }
        pin = tparse_token_u32(tp);
        if (pin > 15) {
          uart_send("ERROR: invalid pin\n");
          break;
        }
        gpio_cfg_input(port, pin);
        uart_send("OK:\n");
        break;
      case __COUNTER__:
        // ATR
        val = 0;
        // fixed TA provided?
        if (tparse_token_size(tp)) {
          val = tparse_token_u32(tp);
        }
        len = iso_powercycle_TA_1(tmp, sizeof(tmp), val);
        if (len < 2) {
          iso_power_down();
          uart_send("ERROR: no card detected\n");
          break;
        }
        uart_send("OK:");
        uart_send_hex(tmp, len);
        uart_send("\n");
        break;
      case __COUNTER__:
        // OFF
        iso_power_down();
        uart_send("OK:\n");
        break;
      case __COUNTER__:
        // info
        uart_send("INFO:");
        len = tparse_token_hex(tp, tmp, sizeof(tmp));
        if (len) {
          uart_send_hex(tmp, len);
          uart_send(",");
        }
        uart_send("VERSION=" VERSION "\n");
        break;
      case __COUNTER__:
        // ON
        iso_gnd(0);
        uart_send("OK:\n");
        break;
      case __COUNTER__:
        // i2ciwait
        len = uwTick + 30*TIMEOUT_1S;

        // port
        ts = 0;
        if (tparse_token_size(tp)) {
          ts = tparse_token_u32(tp);
        }
        // pin
        val = 9;
        if (tparse_token_size(tp)) {
          val = tparse_token_u32(tp);
        }
        tparse_discard_line(tp);
        // until timeout
        for(;;) {
          if ((uwTick - len) < 0x80000000UL) {
            uart_send("TIMEOUT:\n");
            break;
          }
#ifdef I2C_FLAG_EXTI
          if (i2c_consume_int()) {
#else // I2C_FLAG_EXTI
          // check ISO7816 IO (PA9 D8) and INT I2C (PA6 D12) 
          if (!gpio_get(ts, val)) {
#endif // I2C_FLAG_EXTI
            uart_send("OK:\n");
            break;
          }

          // check if a new command is available and abort
          if (tparse_check_command()) {
            // don't reply to the aborted command, as it would reply to the newly received instead
            //uart_send("ABORT:\n");
            break;
          }
        }
        break;
      case __COUNTER__:
        // I2C scan
        len = 0;
        uart_send("OK:");
        while (len<256) {
          val = 10;
          ts = 0;
          while (val-- && !ts) {
            ts = i2c_strobe(len);
            LL_mDelay(1);
          }
          if (ts) {
            uart_send("0x");
            uart_send_hex((uint8_t*)&len, 1);
            uart_send(",");
          }
          len+=2; // skip the READ address each time
        }
        uart_send("\n");
        break;
      case __COUNTER__:
        // reset
        uart_send("OK:\n");
        LL_mDelay(10);
        NVIC_SystemReset();
        break;
      case __COUNTER__:
        // can tx
        val = tparse_token_u32(tp); // id
        // Standard/Extended
        if (1 != tparse_token(tp, (char*)tmp, 1)) {
        error_can_style:
          uart_send("ERROR: invalid Standard/Extended indication\n");
          break;
        } 
        switch(tmp[0]) {
          case 'E':
          case 'e':
            ts = CAN_ID_EXTENDED_LEN;
            break;
          case 'S':
          case 's':
            ts = CAN_ID_STANDARD_LEN;
            break;
          default:
            goto error_can_style;
        }
        len = tparse_token_hex(tp, tmp, sizeof(tmp));
        if (len > 8) {
          uart_send("ERROR: invalid frame len (max 8 bytes)\n");
          break;
        }
        ts = can_tx(val, ts, tmp, len);
        if (len != ts) {
          uart_send("ERROR: CAN transmit failed,");
          uart_send_hex(&ts,4);
          uart_send("\n");
          break;
        }
        uart_send("OK:\n");
        break;
      case __COUNTER__:
        // can rx
        len = can_fifo_rx(&addr, &ts, tmp, sizeof(tmp));
        if (ts == 0) {
          uart_send("ERROR: no frame available\n");
          break;
        }
        uart_send("OK:");
        addr = __bswap_32(addr); // change endianess (LE cpu => BE uart)
        uart_send_hex(&addr, 4);
        switch(ts) {
          case CAN_ID_STANDARD_LEN:
            uart_send(",s,");
            break;
          case CAN_ID_EXTENDED_LEN:
            uart_send(",e,");
            break;
          default:
            uart_send(",U,");
            break;
        }
        uart_send_hex(tmp, len);
        uart_send("\n");
        break;
      case __COUNTER__:
        // can available
        len = can_fifo_avail();
        uart_send("OK:");
        uart_send_hex(&len, 1);
        uart_send("\n");
        break;
      case __COUNTER__:
        // can config
        // ccfg
        // CAN bus frequency 
        val = tparse_token_u32(tp);
        if (val > SystemCoreClock/4) {
          uart_send("ERROR: invalid frequency parameter\n");
          break;
        }
        Configure_CAN(val, 0);
        // CAN bus interrupt reception mode?
        flags &= ~FLAGS_CAN_INTRX;
        if (tparse_token_size(tp)) {
          if (tparse_token_u32(tp) > 0) {
            flags |= FLAGS_CAN_INTRX;
          }
        }
        uart_send("OK:\n");
        break;
      case __COUNTER__:
        // i2cfg
        val = tparse_token_u32(tp);
        if (Configure_I2C1(val)) {
          uart_send("ERROR: unsupported khz value (100,400,1000,1500)");
          break;
        }
        uart_send("OK:\n");
        break;
      case __COUNTER__:
        // isocfg
        val = 0;
        if (tparse_token_size(tp)) {
          val = tparse_token_u32(tp);
        }
        Configure_USART1_ISO_CLK(val);
        uart_send("OK:\n");
        break;

      case __COUNTER__:
        // quick reply
        if (previous_sw && ts2<=0) {
          uart_send("ERROR:\n");
        }
        // i2cwc <addr> <data> [<retrycount>] 
        addr = tparse_token_u32(tp);
        if (addr >= 0x100 || addr == -1) {
          uart_send("ERROR: invalid address\n");
          break;
        }
        len = tparse_token_hex(tp, tmp, sizeof(tmp));
        if (len == 0) {
          uart_send("ERROR: invalid data\n");
          break;
        }
        // optional retry count
        ts = 0;
        if (tparse_token_size(tp)) {
          ts = tparse_token_u32(tp);
        }
        previous_sw = 0;
        // only send the ok for the next command when the 
        // buffer has been moved to avoid race with the host
        uart_send("OK:\n");
      write_again_c:
        val = i2c_write(addr, tmp, len);
        previous_sw = 1;
        ts2 = val;
        if (len != val) {
          if (ts--) {
            goto write_again_c;
          }
          break;
        }
        break;
      case __COUNTER__:
        // i2cwclast // last reply
        if (previous_sw && ts2<=0) {
          uart_send("ERROR:\n");
        }
        else {
          uart_send("OK:\n");
        }
        previous_sw = 0;
        break;

      case __COUNTER__:
        // i2crxfer <addr> <notify_port> <notify_pin> <notify_timeout_ms> <format> [<retry_count>]
        // addr
        addr = tparse_token_u32(tp);
        if (addr >= 0x100 || addr == -1) {
          uart_send("ERROR: invalid address\n");
          break;
        }
        // port
        if (!tparse_token_size(tp)) {
        usage_i2crxfer:
          uart_send("ERROR: i2crxfer <notify_port> <notify_pin> <notify_timeout_ms> <format:0=cargo> [<retry_count>]");
          break;
        }
        port = tparse_token_u32(tp);
        // pin
        if (!tparse_token_size(tp)) {
          goto usage_i2crxfer;
        }
        pin = tparse_token_u32(tp);
        // timeout
        if (!tparse_token_size(tp)) {
          goto usage_i2crxfer;
        }
        timeout = tparse_token_u32(tp);
        // format
        if (!tparse_token_size(tp)) {
          goto usage_i2crxfer;
        }
        if (tparse_token_u32(tp) != 0) {
          goto usage_i2crxfer;
        }
        // optional retry count
        ts = 0;
        if (tparse_token_size(tp)) {
          ts = tparse_token_u32(tp);
        }
        tparse_discard_line(tp);
        // process CARGO instructions
        tlen=0;
        do {
          // read the chunk
          if (tlen == 0) {
            // until timeout
            t = uwTick + timeout;
            for(;;) {
              if ((uwTick - t) < 0x80000000UL) {
                uart_send("TIMEOUT:\n");
                goto end_cmd;
              }
              // check ISO7816 IO (PA9 D8) and INT I2C (PA6 D12) 
              if (!gpio_get(port, pin)) {
                // there is data signalled, start reading them out
                break;
              }
            }
            len = 3; // read only the header
          }
          else {
            // should take a MTU parameter!
            len = MIN(255,MIN(tlen, sizeof(tmp)));
          }
          ts2 = ts;
        read_again_c:
          val = i2c_read(addr, tmp, len);
          if (len != val) {
            if (ts2--) {
              goto read_again_c;
            }
            uart_send_dma("ERROR: not all bytes read: ", 27);
            val = 'z' + val;
            uart_send_dma((uint8_t*)&val, 1);
            uart_send_dma("\n", 1);
            goto end_cmd;
          }
          //interp header
          if (tlen == 0) {
            tlen = ((tmp[1]&0xFF)<<8) | (tmp[2]&0xFF);
            uart_send("OK:");
          }
          // decrement data read
          else {
            // output data (except the header!)
            for (uint32_t i = 0; i < len; i++) {
              uart_usbvcp_buffer[i*2] = n2h(tmp[i]>>4);
              uart_usbvcp_buffer[i*2+1] = n2h(tmp[i]&0xF);
            }
            uart_send_dma(uart_usbvcp_buffer, len*2);
            tlen -= len;
          }
        }
        // until all cargo length has been read
        while (tlen);
        uart_send_dma("\n", 1); // we're DMA enabled in that transfer;
        break;

      case __COUNTER__:
        // spix <data>
        len = tparse_token_hex(tp, tmp, sizeof(tmp));
        if (len > 256) {
          uart_send("ERROR: invalid frame len (max 256 bytes)\n");
          break;
        }
        len = spi_xfer(tmp, len);
        uart_send("OK:");
        uart_send_hex(tmp, len);
        uart_send("\n");
        break;
      case __COUNTER__:
        uart_send("OK:\n");
        // rel
        // this command release all PINs, to allow for external IO operation, it requires a 'reset' afterwards
        // before resuming operations
        __GPIOA_CLK_ENABLE();
        GPIOA->MODER = (GPIOA->MODER&0x3C0000F0)|0xC3FFFF0FU;
        __GPIOB_CLK_ENABLE();
        GPIOB->MODER = 0xFFFFFFFFU;
        __GPIOC_CLK_ENABLE();
        GPIOC->MODER = 0xFFFFFFFFU;
        #ifdef GPIOD_BASE
        __GPIOD_CLK_ENABLE();
        GPIOD->MODER = 0xFFFFFFFFU;
        #endif // GPIOD_BASE
        #ifdef GPIOE_BASE
        __GPIOE_CLK_ENABLE();
        GPIOE->MODER = 0xFFFFFFFFU;
        #endif // GPIOE_BASE
        #ifdef GPIOF_BASE
        __GPIOF_CLK_ENABLE();
        GPIOF->MODER = 0xFFFFFFFFU;
        #endif // GPIOF_BASE
        #ifdef GPIOG_BASE
        __GPIOG_CLK_ENABLE();
        GPIOG->MODER = 0xFFFFFFFFU;
        #endif // GPIOG_BASE
        #ifdef GPIOH_BASE
        __GPIOH_CLK_ENABLE();
        GPIOH->MODER = 0xFFFFFFFFU;
        #endif // GPIOH_BASE
        break;
      }
    end_cmd:
      // discard any remnant of the processed line
      tparse_discard_line(tp);
    }
  }
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
  LL_SetSystemCoreClock(CPU_CLOCK);

  /* Set systick to 1ms in using frequency set to 80MHz */
  /* This frequency can be calculated through LL RCC macro */
  /* ex: __LL_RCC_CALC_PLLCLK_FREQ(__LL_RCC_CALC_MSI_FREQ(LL_RCC_MSIRANGESEL_RUN, LL_RCC_MSIRANGE_6),
                                  LL_RCC_PLLM_DIV_1, 40, LL_RCC_PLLR_DIV_2)*/
  SysTick_Config(CPU_CLOCK/1000);
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

  // USART used for USBVCP communication
  Configure_USART2_USBVCP(USART_BAUDRATE_USART2);
  // USART used for interboard communication
  Configure_USART3(USART_BAUDRATE_USART3);
  // Usart used for external device 
  Configure_UART4(USART_BAUDRATE_UART4);
  // Usart for live display
  Configure_UART5(USART_BAUDRATE_UART5);

  Configure_I2C1(400);

  Configure_SPI1();

  Configure_USART1_ISO();

#ifdef CAN
  Configure_CAN(500000, 0);
#endif // CAN

  while (1)
  {
    interp();
  }
}
