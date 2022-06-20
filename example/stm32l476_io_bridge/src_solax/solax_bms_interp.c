
#include "main.h"
#include "tparse.h"
#include "stddef.h"

#ifdef MODE_SOLAX_BMS

uint8_t tmp[300];

/**
SOLAX X1 <==CAN==> nucleo MODE_BMS_CAN <=(USART3)====(USART3)=> nucleo SLAVE <==CAN==> Pylontech SC0500
*/
#define BMS_RECONNECT_DELAY 20000

#define BMS_PING_INTERVAL_MS 1000

#define SLAVE_TIMEOUT 100

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
  tparse_finger(tp, sizeof(uart3_buffer) - DMA1_Channel3->CNDTR);
  if (tparse_has_line(tp)) {
    // parse the reply status
    buffer_maxlength = tparse_token(tp, (char*)buffer, buffer_maxlength);
    uint32_t ok = memcmp(buffer, "OK", MIN(sizeof("OK")-1, buffer_maxlength)) == 0;
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

void interp(void) {
  uint32_t forward;
  enum slave_state_e slave_state = SLAVE_IDLE;
  uint32_t slave_cmd_timeout;
  uint32_t bms_ping_timeout;
  size_t len;
  uint32_t cid;
  size_t cid_bitlen;

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

  master_log("Reset\n");

  // sent ping to the BMS to wake it up at reset moment
  master_log_can("    >>> bms | ", 0x1871, 29, (uint8_t*)"\x01\x00\x01\x00\x00\x00\x00\x00", 8);
  slave_send("ctx 0x1871 e 0100010000000000\n");
  bms_ping_timeout = uwTick + BMS_PING_INTERVAL_MS;

  // will fetch reply before sending anything else
  slave_state = SLAVE_CTX_SENT;
  slave_cmd_timeout = uwTick + SLAVE_TIMEOUT;

  while (1) {
    // check for messages from the inverter
    if (can_fifo_avail() && slave_state == SLAVE_IDLE) {
      len = can_fifo_rx(&cid, &cid_bitlen, tmp, sizeof(tmp));
      master_log_can("inv >>>     | ", cid, cid_bitlen, tmp, len);
      // other request from the inverter are discarded
      if (cid_bitlen && cid == 0x1871) {
        forward = 0;
        switch(tmp[0]) {
          case 1:
            // get data
            forward = 1;
            can_tx_log(0x0100A001, CAN_ID_EXTENDED_LEN, NULL, 0);
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
        }
        else {
          slave_send("cavail\n");
          slave_state = SLAVE_CAVAIL_SENT;
        }
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
              forward = 1;
              break;
            case 0x1871:
            case 0x1872:
              // could overwrite voltage bounds and max ch/disch currents
            case 0x1874:
            case 0x1875:
            case 0x1876:
            case 0x1878:
              forward = 1;
              break;
            case 0x1877:
              memmove(tmp, "\x00\x00\x00\x00\x52\x00\x00\x00", 8); // OK 2H48050
              len = 8;
              forward = 1;
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
  }
}
#endif // MODE_SOLAX_BMS
