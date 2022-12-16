
#include "main.h"
#include "tparse.h"
#include "stddef.h"
#include "stdio.h"

#ifdef MODE_SOLAX_BMS

extern uint8_t tmp[300];

/**
SOLAX X1 <==CAN==> nucleo MODE_BMS_CAN <=(USART3)====(USART3)=> nucleo SLAVE <==CAN==> Pylontech SC0500
                   iobridge in solax makefile                  regular iobridge
*/
#define BMS_RECONNECT_DELAY 20000

#define BMS_PING_INTERVAL_MS 1000

#define SLAVE_TIMEOUT 100
#define SOLAX_PW_TIMEOUT 1000 // give it a second for 0xA0 bytes @ 9600bps
#define SOLAX_PW_NEXT_TIMEOUT 1000 // every 2 seconds, check it

#define SOLAX_PV_POWER_OPT_THRESHOLD_V 140
#define SOLAX_PV_POWER_OPT_THRESHOLD_W 25
#define SOLAX_GRID_EXPORT_OPT_THRESHOLD_W -25

#define DISPLAY_TIMEOUT 1000;

#define S2LE(buf, off) ((int16_t)((int16_t)((int16_t)((int16_t)(buf)[off+1])<<8l) | (int16_t)((int16_t)(buf)[off]&0xFFl) ))
#define U2LE(buf, off) ((((buf)[off+1]&0xFFu)<<8) | ((buf)[off]&0xFFu) )

// for snprintf to work as expected
void _sbrk(void) {

}

void master_log(char* buffer) {
  uart_select_intf(USART2);
  uart_send(buffer);
}

void master_log_mem(void* _buffer, size_t length) {
  uint8_t* buffer = (uint8_t*)_buffer;
  uart_select_intf(USART2);
  uart_send_mem(buffer, length);
}

void master_log_hex(void* _buffer, size_t length) {
  uint8_t* buffer = (uint8_t*)_buffer;
  uart_select_intf(USART2);
  uart_send_hex(buffer, length);
}

void master_log_can(char* prefix, uint32_t cid, size_t cid_bitlen, uint8_t* canmsg, size_t canmsg_len) {
  master_log(prefix);
  master_log(" 0x");
  // BE to LE for printing
  cid = __bswap_32(cid);
  switch(cid_bitlen) {
    case CAN_ID_EXTENDED_LEN:
      master_log_hex(&cid, 4);
      master_log(" e ");
      break;
    case CAN_ID_STANDARD_LEN:
      cid_bitlen = 2;
      master_log_hex(&cid, 2);
      master_log(" e ");
      break;
    default:
      master_log_hex(&cid, 4);
      master_log(" U ");
      break;
  }
  master_log_hex(canmsg, canmsg_len);
  master_log("\n");
}

void slave_send_mem(void* _buffer, size_t buffer_length) {
  uint8_t* buffer = (uint8_t*)_buffer;
  uart_select_intf(USART3);
  uart_send_mem(buffer, buffer_length);
}

void slave_send_hex(void* _buffer, size_t buffer_length) {
  uint8_t* buffer = (uint8_t*)_buffer;
  uart_select_intf(USART3);
  uart_send_hex(buffer, buffer_length);
}

void slave_send(char* str) {
  slave_send_mem(str, strlen(str));
}

size_t slave_recv(tparse_ctx_t* tp, uint8_t *buffer, size_t buffer_maxlength) {
next_line:
  tparse_finger(tp, sizeof(uart3_buffer) - DMA1_Channel3->CNDTR);
  if (tparse_has_line(tp)) {
    // parse the reply status
    buffer_maxlength = tparse_token(tp, (char*)buffer, buffer_maxlength);
    uint32_t ok = memcmp(buffer, "OK", MIN(sizeof("OK")-1, buffer_maxlength)) == 0;
    // reset received, discard line and check next
    if (!ok && memcmp(buffer, "RESET", MIN(sizeof("RESET")-1,buffer_maxlength)) == 0) {
      tparse_discard_line(tp);
      goto next_line;
    }
    return ok;
  }
  return 0;
}

size_t slave_avail(tparse_ctx_t* tp) {
  tparse_finger(tp, sizeof(uart3_buffer) - DMA1_Channel3->CNDTR);
  return tparse_has_line(tp);
}

void can_tx_log(uint32_t cid, size_t cid_bitlen, uint8_t* canmsg, size_t canmsg_len) {
  master_log_can("    >>> inv | ", cid, cid_bitlen, canmsg, canmsg_len);
  can_tx(cid, cid_bitlen, canmsg, canmsg_len);
}

enum slave_state_e {
  SLAVE_IDLE,
  SLAVE_CAVAIL_SENT,
  SLAVE_CRX_SENT,
  SLAVE_CTX_SENT,
};

enum solax_pw_state_e {
  SOLAX_PW_IDLE,
  SOLAX_PW_REQ_SENT,
  SOLAX_PW_WAIT_NEXT,
};

void interp(void) {
  uint32_t forward;
  enum slave_state_e slave_state = SLAVE_IDLE;
  enum solax_pw_state_e solax_pw_state = SOLAX_PW_IDLE;
  uint32_t slave_cmd_timeout;
  uint32_t solax_pw_timeout;
  uint32_t bms_ping_timeout;
  size_t len;
  uint32_t cid;
  size_t cid_bitlen;
  uint32_t enable_battery = 0;
  //uint32_t wait_bms_info = 1;
  //uint8_t bms_info[8];
  uint32_t batt_drain_fix=0;
  uint32_t batt_drain_fix_cause = 0;
  struct {
    uint16_t pv1_voltage;
    uint16_t pv2_voltage;
    uint16_t pv1_current;
    uint16_t pv2_current;
    uint16_t pv1_wattage;
    uint16_t pv2_wattage;
    int16_t bat_wattage;
    uint16_t bat_SoC;
    int16_t bat_temp;
    int16_t grid_wattage;
    int16_t grid_export_wattage;
    int16_t eps_current;
  } solax;
  struct {
    uint16_t voltage;
    int16_t current;
    uint16_t soc;
    int32_t wattage;
  } pylontech;
  uint32_t timeout_next_display = uwTick;

#ifdef MODE_FAKE_SOLAX
  while (1) {
    can_tx_log(0x1871, CAN_ID_EXTENDED_LEN, (uint8_t*)"\x01\x00\x01\x00\x00\x00\x00\x00", 8);
    LL_mDelay(BMS_PING_INTERVAL_MS);
  }
#endif // MODE_FAKE_SOLAX

  // interface with the slave nucleo
  tparse_ctx_t tp_u3;
  tparse_init(&tp_u3, uart3_buffer, sizeof(uart3_buffer), " \n:,");
  uint32_t bms_reconnect_at = uwTick;

  tparse_ctx_t tp_u4;
  tparse_init(&tp_u4, uart4_buffer, sizeof(uart4_buffer), "");

  master_log("Reset\n");

  // resynch slave
  for (;;) {
    // discard data
    tparse_discard_line(&tp_u3);
    // send info
    slave_send("info 0x1\n");
    slave_cmd_timeout = uwTick + SLAVE_TIMEOUT;
    while (!slave_avail(&tp_u3)) {
      if (uwTick - slave_cmd_timeout < 0x80000000UL) {
        break;
      }
    }
    // wait info reply
    if ( tparse_token(&tp_u3, tmp, sizeof(tmp)) ) {
      if (memcmp(tmp, "INFO", sizeof("INFO")-1) == 0) {
        master_log("Slave resynch ok\n");
        // resynch success!
        break;
      }
      master_log("Retry resynch\n");
    }
    // if not info, do again
  }
  tparse_discard_line(&tp_u3);

  // sent ping to the BMS to wake it up at reset moment
  master_log_can("    >>> bms | ", 0x1871, 29, (uint8_t*)"\x01\x00\x01\x00\x00\x00\x00\x00", 8);
  slave_send("ctx 0x1871 e 0100010000000000\n");
  bms_ping_timeout = uwTick + BMS_PING_INTERVAL_MS;

  // will fetch reply before sending anything else
  slave_state = SLAVE_CTX_SENT;
  slave_cmd_timeout = uwTick + SLAVE_TIMEOUT;

  while (1) {
    // check for messages from the inverter
    if (slave_state == SLAVE_IDLE && can_fifo_avail()) {
      len = can_fifo_rx(&cid, &cid_bitlen, tmp, sizeof(tmp));
      master_log_can("inv >>>     | ", cid, cid_bitlen, tmp, len);
      // other request from the inverter are discarded
      if (cid_bitlen && cid == 0x1871) {
        forward = 0;
        switch(tmp[0]) {
          case 1:
            // get data
            forward = 1;
            if (enable_battery==1) {
              can_tx_log(0x0100A001, CAN_ID_EXTENDED_LEN, NULL, 0);
              enable_battery = 2;
            }
            can_tx_log(0x1801, CAN_ID_EXTENDED_LEN, (uint8_t*)"\x01\x00\x01\x00\x00\x00\x00\x00", 8);
            break;
          case 2:
            // disconnect request (fault seen from the inverter's side)
            // if sent to the BMS, the SC0500 goes to slumber and no command can wake it up, 
            // have it disconnect after period of inactivity from the inverter instead.
            bms_reconnect_at = uwTick + BMS_RECONNECT_DELAY;
            break;

          case 3:
            // not timestamp
            if (tmp[1] != 6) {
              bms_reconnect_at = uwTick + BMS_RECONNECT_DELAY;
            }
            break;

          case 5:
            // ping?
            break;
        }

        // only forward when the bms is allowed (not timing out for juice cut)
        if (forward && uwTick - bms_reconnect_at < 0x80000000UL) {
          // reset to reenable battery
          enable_battery = 0;
          bms_reconnect_at = uwTick; // make sure to avoid overflow when no reconnection request for a while
          slave_send("ctx 0x");
          uint32_t cidbe = __bswap_32(cid);
          slave_send_hex(&cidbe, 4);
          slave_send(" e ");
          slave_send_hex(tmp, len);
          slave_send("\n");
          master_log_can("    >>> bms | ", cid, cid_bitlen, tmp, len);
          slave_state = SLAVE_CTX_SENT;
          slave_cmd_timeout = uwTick + SLAVE_TIMEOUT;
        }
      }
    }

    // check for messages from the bms
    switch(slave_state) {
      case SLAVE_IDLE:
        // time for a ping
        if (uwTick - bms_ping_timeout < 0x80000000UL) {
          slave_send("ctx 0x1871 e 0100010000000000\n");
          slave_state = SLAVE_CTX_SENT;
          bms_ping_timeout = uwTick + BMS_PING_INTERVAL_MS;
          break;
        }
        slave_send("cavail\n");
        slave_state = SLAVE_CAVAIL_SENT;
        slave_cmd_timeout = uwTick + SLAVE_TIMEOUT;
        __attribute__((fallthrough));
      case SLAVE_CAVAIL_SENT:
        if (slave_recv(&tp_u3, tmp, sizeof(tmp))) {
          len = tparse_token_hex(&tp_u3, tmp, sizeof(tmp));
          tparse_discard_line(&tp_u3);
          if (len == 1 && tmp[0] > 0) {
            slave_send("crx\n");
            slave_state = SLAVE_CRX_SENT;
            slave_cmd_timeout = uwTick + SLAVE_TIMEOUT;
            goto case_SLAVE_CRX_SENT;
          }
          // other reply ?
          slave_state = SLAVE_IDLE;
        }
        else if (uwTick - slave_cmd_timeout < 0x80000000UL) {
          slave_state = SLAVE_IDLE;
        }
        break;
      case SLAVE_CRX_SENT:
      case_SLAVE_CRX_SENT:
        if (slave_recv(&tp_u3, tmp, sizeof(tmp))) {
          slave_state = SLAVE_IDLE;
          cid = tparse_token_u32_base(&tp_u3, 16);// cid is in hex base, not advertise (no 0x leading)
          tparse_token(&tp_u3, (char*)tmp, sizeof(tmp));
          if (tmp[0] == 'e') {
            cid_bitlen = CAN_ID_EXTENDED_LEN;
          }
          else if (tmp[0] == 's') {
            cid_bitlen = CAN_ID_STANDARD_LEN;
          }
          len = 0;
          if (! tparse_eol_reached(&tp_u3) && tparse_token_size(&tp_u3) > 0) {
            len = tparse_token_hex(&tp_u3, tmp, sizeof(tmp));
          }
          tparse_discard_line(&tp_u3);
          master_log_can("bms >>>     | ", cid, cid_bitlen, tmp, len);
          forward = 0;
          switch(cid) {
            case 0x1873:
              // may reinterpret SoC depending on battery voltage instead of relying on BMS
              pylontech.voltage = U2LE(tmp,0);
              pylontech.current = S2LE(tmp,2);
              pylontech.soc = U2LE(tmp,4);
              pylontech.wattage = ((int32_t)pylontech.voltage)*((int32_t)pylontech.current)/((int32_t)100); // unit 0.1V x 0.1A
              if (batt_drain_fix || pylontech.soc <= 10) {
                tmp[4] = 100;
                tmp[5] = 0;
              }
              forward = 1;
              break;
            case 0x1872:
              // avoid BMS adjustements at runtime, may occur to force batt equalization, which leads to discharge?
              // if (wait_bms_info) {
              //   memmove(bms_info, tmp, 8);
              //   // could overwrite voltage bounds and max ch/disch currents
              //   wait_bms_info = 0;
              // }
              // else {
              //   memmove(tmp, bms_info, 8);
              // }
              // force disabling charge, in order to avoid driving the battery to retrieve too less energy from the PVs.
              if (batt_drain_fix) {
                tmp[4] = 0;
                tmp[5] = 0;
              }
              // disallow discharge when battery is too low
              if (pylontech.soc <= 10) {
                tmp[6] = 0;
                tmp[7] = 0;
              }
              forward = 1;
              break;
            case 0x1871:
            case 0x1874:
            case 0x1875:
            case 0x1876:
            case 0x1878:
              forward = 1;
              break;
            case 0x1877:
              // override message to tell the inverter of the battery configuration
              memmove(tmp, "\x00\x00\x00\x00\x52\x00\x00\x00", 8); // OK 2H48050, OK 4 H48050
              len = 8;
              forward = 1;
              // if (enable_battery == 0) 
              {
                enable_battery = 1;
              }
              break;
          }
          if (forward) {
            can_tx_log(cid, cid_bitlen, tmp, len);
          }
        }
        else if (uwTick - slave_cmd_timeout < 0x80000000UL) {
          slave_state = SLAVE_IDLE;
        }
        break;
      case SLAVE_CTX_SENT:
        if (slave_recv(&tp_u3, tmp, sizeof(tmp))) {
          // and discard response, best effort here
          tparse_discard_line(&tp_u3);
          slave_state = SLAVE_IDLE;
        }
        else if (uwTick - slave_cmd_timeout < 0x80000000UL) {
          slave_state = SLAVE_IDLE;
        }
        break;
      default:
        // what the state?
        break;
    }

    // handle solax PocketWifi port to get the pv arrays status
    tparse_finger(&tp_u4, sizeof(uart4_buffer) - DMA2_Channel5->CNDTR);
    switch(solax_pw_state) {
      case SOLAX_PW_IDLE:
        tparse_discard(&tp_u4);
        // send info request to solax
        uart_select_intf(UART4);
        uart_send_mem("\xAA\x55\x07\x01\x10\x17\x01", 7);
        solax_pw_state = SOLAX_PW_REQ_SENT;
        solax_pw_timeout = uwTick + SOLAX_PW_TIMEOUT;
        break;

      case SOLAX_PW_REQ_SENT:
        // if the reply is complete
        if (tparse_avail(&tp_u4) >= 0x97) {
          tparse_token(&tp_u4, tmp, sizeof(tmp));
          // check it's the expected response
          if (tmp[0] == 0xAA && tmp[1] == 0x55 && tmp[2] == 0x97 && tmp[3] == 0x01 && tmp[4] == 0x90 ) {
            // extract fields
            solax.grid_wattage = S2LE(tmp, 9);
            solax.pv1_voltage = U2LE(tmp, 13);
            solax.pv2_voltage = U2LE(tmp, 15);
            solax.pv1_current = U2LE(tmp, 17);
            solax.pv2_current = U2LE(tmp, 19);
            solax.pv1_wattage = U2LE(tmp, 21);
            solax.pv2_wattage = U2LE(tmp, 23);
            solax.bat_wattage = S2LE(tmp, 37);
            solax.bat_temp = S2LE(tmp, 39);
            solax.bat_SoC = U2LE(tmp, 41);
            solax.eps_current = U2LE(tmp, 65);
            solax.grid_export_wattage = S2LE(tmp, 69);

            // reenable at next read
            batt_drain_fix = 0;
            // if total PV voltage is below a limit, just AVOID wasting battery energy, 
            // and disable battery charging to force the solax stopping the MPPT draining 
            // energy from the battery to operate.
            // ok but when not connected to grid, could not start! // if (solax.pv1_wattage + solax.pv2_wattage < 100) 
            // OFFGRID or NIGHT could still have that condition true 
            if (solax.pv1_voltage + solax.pv2_voltage < SOLAX_PV_POWER_OPT_THRESHOLD_V*10 
                &&
                solax.pv1_wattage + solax.pv2_wattage < SOLAX_PV_POWER_OPT_THRESHOLD_W)  // requires some insight on the total PV array connection
            {
              batt_drain_fix_cause = 1;
              batt_drain_fix = 1;
            }
            // void if OFFGRID
            else if (solax.grid_export_wattage < SOLAX_GRID_EXPORT_OPT_THRESHOLD_W) {
              // if importing, then panels does not cover the house, disable charging to avoid draining
              batt_drain_fix_cause = 2;
              batt_drain_fix = 1;
            }
            // condition won't last long in OFFGRID
            else if (pylontech.soc >= 100) {
              batt_drain_fix_cause = 3;
              batt_drain_fix = 1;
            }

          }
          tparse_discard(&tp_u4);
          solax_pw_state = SOLAX_PW_WAIT_NEXT;
          solax_pw_timeout = uwTick + SOLAX_PW_NEXT_TIMEOUT;
        }
        else if (uwTick - solax_pw_timeout < 0x80000000UL) {
          tparse_discard(&tp_u4);
          solax_pw_state = SOLAX_PW_WAIT_NEXT;
          solax_pw_timeout = uwTick + SOLAX_PW_NEXT_TIMEOUT;
        }
        break;

      case SOLAX_PW_WAIT_NEXT:
        if (uwTick - solax_pw_timeout < 0x80000000UL) {
          solax_pw_state = SOLAX_PW_IDLE;
        }
        break;
    }

    // display
    if (uwTick - timeout_next_display < 0x80000000UL) {
      // prepare next sending
      timeout_next_display = uwTick + DISPLAY_TIMEOUT;
      uart_select_intf(UART5);
      // wipe screen
      if (solax.grid_export_wattage >= 0) {
        snprintf(tmp, sizeof(tmp), "GRID << %dW\n", solax.grid_export_wattage);
      }
      else {
        snprintf(tmp, sizeof(tmp), "GRID >> %dW %s\n", -solax.grid_export_wattage, batt_drain_fix_cause==2?"IMPORT":"");
      }
      uart_send_mem(tmp, strlen(tmp));
      if (solax.grid_wattage >= 0) {
        snprintf(tmp, sizeof(tmp), "HOUSE << %dW\n", solax.grid_wattage);
      }
      else {
        snprintf(tmp, sizeof(tmp), "HOUSE >> %dW\n", -solax.grid_wattage);
      }
      uart_send_mem(tmp, strlen(tmp));
      // BEGIN BATT
      if (pylontech.wattage >= 0) {
        snprintf(tmp, sizeof(tmp), "BAT << %dW ", pylontech.wattage);
      }
      else {
        snprintf(tmp, sizeof(tmp), "BAT >> %dW ", -pylontech.wattage);
      }
      uart_send_mem(tmp, strlen(tmp));
      int16_t cur = pylontech.current;
      if (cur < 0) {
        uart_send_mem("-", 1);
        cur = -cur;
      }
      snprintf(tmp, sizeof(tmp), "%d.%dA %u%% %s%s\n", cur/10, cur%10, pylontech.soc, batt_drain_fix?"OPT":"",batt_drain_fix_cause==3?"+":"");
      uart_send_mem(tmp, strlen(tmp));
      // END BAT
      snprintf(tmp, sizeof(tmp), "PV %uV %dW | %uV %dW %s\n", solax.pv1_voltage/10, solax.pv1_wattage, solax.pv2_voltage/10, solax.pv2_wattage, batt_drain_fix_cause==1?"LOW":"");
      uart_send_mem(tmp, strlen(tmp));
    }

  } // end infinited loop

}
#endif // MODE_SOLAX_BMS
