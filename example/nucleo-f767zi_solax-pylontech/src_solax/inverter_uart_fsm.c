#include "main.h"
#include "tparse.h"
#include "stddef.h"
#include "stdio.h"
#include "stdbool.h"
#include "globals.h"

#define INVERTER_UART_TIMEOUT_MS 10000 // give few seconds for 400 bytes @ 9600bps
#define INVERTER_UART_NEXT_TIMEOUT 1000 // pocket wifi link update
#define INVERTER_UART_INVALID_RETRY_TIMEOUT 500 // 100ms before retrying in case of an error on the pocketwifi serial response

void inverter_uart_idle(void);
uint32_t inverter_uart_parse_response(uint8_t* buffer, uint32_t length);

enum inverter_uart_state_e {
  INVERTER_UART_IDLE,
  INVERTER_UART_SEND,
  INVERTER_UART_REQ_SENT,
  INVERTER_UART_WAIT_NEXT,
  INVERTER_UART_INVALID_NEXT,
} inverter_uart_state;

uint32_t inverter_uart_timeout;

#define INVERTER_UART_QUEUE_SIZE 3 // schedule a mode change while reading a status response
struct {
  uint8_t*  cmd;
  uint32_t  cmd_len;
  uint32_t  rep_len;
} inverter_usart_queue[INVERTER_UART_QUEUE_SIZE];

void inverter_usart_queue_pop(void) {
  // consume the first slot
  memmove(&inverter_usart_queue[0], &inverter_usart_queue[1], sizeof(inverter_usart_queue)-sizeof(inverter_usart_queue[0]));
  memset(&inverter_usart_queue[2], 0, sizeof(inverter_usart_queue[2]));
}

uint32_t inverter_usart_queue_free(void) {
  uint32_t idx=0;
  // seek for first free slot
  while (inverter_usart_queue[idx].cmd_len != 0 && idx < INVERTER_UART_QUEUE_SIZE) {
    idx++;
  }
  return INVERTER_UART_QUEUE_SIZE - idx;
}

void inverter_usart_queue_push(const uint8_t* cmd, uint32_t cmd_len, uint32_t rep_len) {
  uint32_t idx=0;
  // seek for first free slot
  while (inverter_usart_queue[idx].cmd_len != 0 && idx < INVERTER_UART_QUEUE_SIZE) {
    idx++;
  }
  // full
  if (idx >= INVERTER_UART_QUEUE_SIZE) {
    return;
  }
  inverter_usart_queue[idx].cmd = (uint8_t*)cmd;
  inverter_usart_queue[idx].cmd_len = cmd_len;
  inverter_usart_queue[idx].rep_len = rep_len;
}

tparse_ctx_t tp_solax_pw;

void inverter_uart_init(void) {
  inverter_uart_timeout=0;
  inverter_uart_state = INVERTER_UART_IDLE;
  memset(inverter_usart_queue, 0, sizeof(inverter_usart_queue));
  tparse_init(&tp_solax_pw, uart_pw_buffer, sizeof(uart_pw_buffer), "");
}

void inverter_uart_update(void) {
  // handle solax PocketWifi port to get the solax status
  tparse_finger(&tp_solax_pw, sizeof(uart_pw_buffer) - DMA_Stream_PW->NDTR);
  switch(inverter_uart_state) {
    case INVERTER_UART_IDLE:
      // is no command scheduled for sending?
      if (inverter_usart_queue_free() == INVERTER_UART_QUEUE_SIZE) {
        inverter_uart_idle();
        inverter_uart_state = INVERTER_UART_SEND;
      }
      break;

    case INVERTER_UART_SEND:
      // data request to be transmitted toward the inverter
      if (inverter_usart_queue_free() != INVERTER_UART_QUEUE_SIZE) {
        tparse_discard(&tp_solax_pw);
        master_log("UART >> ");
        master_log_hex(inverter_usart_queue[0].cmd, inverter_usart_queue[0].cmd_len);
        master_log("\n");
        // send info request to solax
        uart_select_intf(UARTPW);
        uart_send_mem(inverter_usart_queue[0].cmd, inverter_usart_queue[0].cmd_len);
        inverter_uart_state = INVERTER_UART_REQ_SENT;
        inverter_uart_timeout = EXPIRE_IN(INVERTER_UART_TIMEOUT_MS);
      }
      break;

    case INVERTER_UART_REQ_SENT:
      // if the reply is complete
      if (tparse_avail(&tp_solax_pw) >= inverter_usart_queue[0].rep_len) {
        size_t read = tparse_read(&tp_solax_pw, (char*)tmp, inverter_usart_queue[0].rep_len);
        if (read < inverter_usart_queue[0].rep_len) {
          master_log("UART reading error ");
          master_log_hex(&read, 4);
          read = tparse_avail(&tp_solax_pw);
          master_log_hex(&read, 4);
          master_log("\n");
          goto invalid;
        }
        master_log("UART << ");
        master_log_hex(tmp, inverter_usart_queue[0].rep_len);
        master_log("\n");

        if (inverter_uart_parse_response(tmp, inverter_usart_queue[0].rep_len)) {
        invalid:
          tparse_reset(&tp_solax_pw);
          inverter_uart_state = INVERTER_UART_INVALID_NEXT;
          inverter_uart_timeout = EXPIRE_IN(INVERTER_UART_INVALID_RETRY_TIMEOUT);
          inverter_usart_queue_pop();
          break;
        }

        // whatever the reply, discard the data after this point
        inverter_uart_state = INVERTER_UART_WAIT_NEXT; // switch state before possibly scheduling a command to send
        inverter_uart_timeout = EXPIRE_IN(INVERTER_UART_NEXT_TIMEOUT);
        inverter_usart_queue_pop();
      }
      // timing out first entry if any
      else if (inverter_usart_queue_free() != INVERTER_UART_QUEUE_SIZE 
        && inverter_uart_timeout && EXPIRED(inverter_uart_timeout)) {
        master_log("UART TIMEOUT\n");
        //master_log_hex(uart_pw_buffer, sizeof(uart_pw_buffer));
        inverter_uart_state = INVERTER_UART_WAIT_NEXT;
        inverter_uart_timeout = EXPIRE_IN(INVERTER_UART_NEXT_TIMEOUT);
        tparse_reset(&tp_solax_pw);
        inverter_usart_queue_pop();

        inverter_uart_force_bitrate();
      }
      break;

    case INVERTER_UART_INVALID_NEXT:
    case INVERTER_UART_WAIT_NEXT:
      // skip to next command sending immediately, this is not a new attempt/request
      if ((inverter_uart_timeout && EXPIRED(inverter_uart_timeout))
        || inverter_usart_queue_free() != INVERTER_UART_QUEUE_SIZE) {
        inverter_uart_state = INVERTER_UART_IDLE;
      }
      break;
  }
}


#define SOLAX_MAX_PV_VOLTAGE_V 600 // from user manual

// 0x28 0x173B // charge or discharge end period
// 0xC4 0x173B // charge of discharge end period

const uint8_t solax_pw_cmd_change_bitrate[] = { 0xAA, 0x55, 0x07, 0x01, 0x85, 0x8C, 0x01};

const uint8_t solax_pw_cmd_get_stat_0x197[] = { 0xAA, 0x55, 0x07, 0x01, 0x10, 0x17, 0x01};
const uint8_t solax_pw_cmd_get_stat_0x25F[] = { 0xAA, 0x55, 0x07, 0x01, 0x13, 0x1A, 0x01};

const uint8_t solax_pw_cmd_mode_self_use[]= { 0xAA, 0x55, 0x09, 0x09, 0x1C, 0x00, 0x00, 0x2D, 0x01 };
//const uint8_t solax_pw_cmd_mode_feedinprio[]= { 0xAA, 0x55, 0x09, 0x09, 0x1C, 0x01, 0x00, 0x2E, 0x01 };
//const uint8_t solax_pw_cmd_mode_backup[]= { 0xAA, 0x55, 0x09, 0x09, 0x1C, 0x02, 0x00, 0x2F, 0x01 };
//const uint8_t solax_pw_cmd_mode_manual[]= { 0xAA, 0x55, 0x09, 0x09, 0x1C, 0x03, 0x00, 0x30, 0x01 };
      
//const uint8_t solax_pw_cfg_manual_stop[]= { 0xAA, 0x55, 0x09, 0x09, 0x24, 0x00, 0x00, 0x35, 0x01 };
//const uint8_t solax_pw_cfg_manual_charge[]= { 0xAA, 0x55, 0x09, 0x09, 0x24, 0x01, 0x00, 0x36, 0x01 };
// useless // const uint8_t solax_pw_cfg_manual_discharge[]= { 0xAA, 0x55, 0x09, 0x09, 0x24, 0x02, 0x00, 0x37, 0x01 };

/*
#set gmppt high
aa5509096a03007e01 aa550709eaf901
#set gmppt low
aa5509096a01007c01 aa550709eaf901
#set gmppt off
aa5509096a00007b01 aa550709eaf901

# set system off
aa5509092f00004001 aa550709afbe01
# set system on
aa5509092f01004101 aa550709afbe01
*/

// enter default PIN: 2014
const uint8_t solax_pw_cfg_PIN[] = {
  0xAA, 0x55, 0x09, 0x09, 0x00, 0xDE, 0x07, 0xF6, 0x01
};

const uint8_t solax_pw_cfg_gmppt_pv1_off[] = {
  0xaa, 0x55, 0x09, 0x09, 0x6a, 0x00, 0x00, 0x7b, 0x01
};
const uint8_t solax_pw_cfg_gmppt_pv1_low[] = {
  0xaa, 0x55, 0x09, 0x09, 0x6a, 0x01, 0x00, 0x7c, 0x01
};
const uint8_t solax_pw_cfg_gmppt_pv1_high[] = {
  0xaa, 0x55, 0x09, 0x09, 0x6a, 0x03, 0x00, 0x7e, 0x01
};

const uint8_t solax_pw_cfg_gmppt_pv2_off[] = {
  0xaa, 0x55, 0x09, 0x09, 0xc2, 0x00, 0x00, 0xd3, 0x01
};
const uint8_t solax_pw_cfg_gmppt_pv2_low[] = {
  0xaa, 0x55, 0x09, 0x09, 0xc2, 0x01, 0x00, 0xd4, 0x01
};
const uint8_t solax_pw_cfg_gmppt_pv2_high[] = {
  0xaa, 0x55, 0x09, 0x09, 0xc2, 0x03, 0x00, 0xd6, 0x01
};

uint32_t solax_checksum_compute(uint8_t *buf, uint16_t len) {
  uint16_t acc=0;
  uint16_t off=0;
  uint16_t l=len-2;
  while (l--) {
    acc+=buf[off++];
  }
  return acc;
}

uint32_t solax_checksum_verify(uint8_t* packet, uint16_t len) {
  return solax_checksum_compute(packet, len) == U2LE(packet, len-2);
}

void solax_pw_gmppt1_off(void);
void solax_pw_gmppt1_high(void);
void solax_pw_gmppt2_off(void);
void solax_pw_gmppt2_high(void);
void solax_pw_mode_self_use(void);

void solax_pw_gmppt1_off(void) {
  if (inverter_usart_queue_free()>=2) {
    inverter_usart_queue_push(solax_pw_cfg_PIN, sizeof(solax_pw_cfg_PIN), 7);
    inverter_usart_queue_push(solax_pw_cfg_gmppt_pv1_off, sizeof(solax_pw_cfg_gmppt_pv1_off), 7);
  }
}

void solax_pw_gmppt1_high(void) {
  if (inverter_usart_queue_free()>=2) {
    inverter_usart_queue_push(solax_pw_cfg_PIN, sizeof(solax_pw_cfg_PIN), 7);
    inverter_usart_queue_push(solax_pw_cfg_gmppt_pv1_high, sizeof(solax_pw_cfg_gmppt_pv1_high), 7);
  }
}

void solax_pw_gmppt2_off(void) {
  if (inverter_usart_queue_free()>=2) {
    inverter_usart_queue_push(solax_pw_cfg_PIN, sizeof(solax_pw_cfg_PIN), 7);
    inverter_usart_queue_push(solax_pw_cfg_gmppt_pv2_off, sizeof(solax_pw_cfg_gmppt_pv2_off), 7);
  }
}

void solax_pw_gmppt2_high(void) {
  if (inverter_usart_queue_free()>=2) {
    inverter_usart_queue_push(solax_pw_cfg_PIN, sizeof(solax_pw_cfg_PIN), 7);
    inverter_usart_queue_push(solax_pw_cfg_gmppt_pv2_high, sizeof(solax_pw_cfg_gmppt_pv2_high), 7);
  }
}

void solax_pw_mode_self_use(void) {
  inverter_usart_queue_push(solax_pw_cmd_mode_self_use, sizeof(solax_pw_cmd_mode_self_use), 7);
}

void inverter_uart_force_bitrate(void) {
  // rest the baudrate of the link just in case (especially during inverter reboot)
  Configure_UARTPW(9600, 0);
  uart_select_intf(UARTPW);
  uart_send_mem(solax_pw_cmd_change_bitrate, sizeof(solax_pw_cmd_change_bitrate));  
  // wait until bitrate is taken into account
  LL_mDelay(250);
  Configure_UARTPW(115200, 0);
}

void inverter_uart_idle(void) {
  // inverter_usart_queue_push(solax_pw_cmd_get_stat_0x197, sizeof(solax_pw_cmd_get_stat_0x197), 0x197);
  inverter_usart_queue_push(solax_pw_cmd_get_stat_0x25F, sizeof(solax_pw_cmd_get_stat_0x25F), 0x25F);
}

uint32_t inverter_uart_parse_response(uint8_t* reply, uint32_t length) {
  // check it's the expected response
  if (reply[0] == 0xAA && reply[1] == 0x55 && reply[2] == 0x5F && reply[3] == 0x81 && reply[4] == 0x90 ) {

    // invalid until tested valid
    solax.valid_data = 0;

    // extract fields
    solax.grid_wattage = S2LE(reply, 9);
    solax.pv1_voltage = U2LE(reply, 13);
    solax.pv2_voltage = U2LE(reply, 15);
    solax.pv1_current = U2LE(reply, 17);
    solax.pv2_current = U2LE(reply, 19);
    solax.pv1_wattage = U2LE(reply, 21);
    solax.pv2_wattage = U2LE(reply, 23);
    if (reply[25] != solax.status) {
      solax.status_count=0;
    }
    solax.status      = reply[25];
    if (solax.status_count<255) {
      solax.status_count++;
    }
    solax.bat_wattage = S2LE(reply, 37);
    solax.bat_temp = S2LE(reply, 39);
    solax.bat_SoC = U2LE(reply, 41);
    solax.output_va = U2LE(reply, 55);
    solax.eps_power = U2LE(reply, 61);
    solax.eps_voltage = U2LE(reply, 63);
    solax.eps_current = U2LE(reply, 65);
    solax.grid_meter_ct = S2LE(reply, 69);
    solax.seconds = reply[203];
    solax.minute = reply[204];
    solax.hour = reply[205];
    solax.day = reply[206];
    solax.month = reply[207];
    solax.year = reply[208] + 2000;

    uint32_t valid_crc = solax_checksum_verify(reply+2,reply[2]-2);

    snprintf((char*)tmp, sizeof(tmp), "PV1: %dW (%d.%dV %d.%dA)\nPV2: %dW (%d.%dV %d.%dA)\n", solax.pv1_wattage, solax.pv1_voltage/10,solax.pv1_voltage%10, solax.pv1_current/10, solax.pv1_current%10, solax.pv2_wattage, solax.pv2_voltage/10, solax.pv2_voltage%10, solax.pv2_current/10, solax.pv2_current%10);
    master_log((char*)tmp);
    snprintf((char*)tmp, sizeof(tmp), "AC: Grid: %dW (meter %dW) EPS: %dW Output: %dVA\n", solax.grid_wattage, solax.grid_meter_ct, solax.eps_power, solax.output_va);
    master_log((char*)tmp);

    if (!valid_crc) {
      return -3;
    }

    //int32_t pylontech_wattage = pylontech.precise_wattage?pylontech.precise_wattage:pylontech.wattage;
    //int32_t power_balance_w = solax.pv1_wattage + solax.pv2_wattage - (solax.grid_wattage + pylontech_wattage );
    // check for invalid data (glitch sometimes returned by the inverter)
    if (solax.pv1_voltage > SOLAX_MAX_PV_VOLTAGE_V*10 || solax.pv2_voltage > SOLAX_MAX_PV_VOLTAGE_V*10) {
      master_log("cause 71\n");
      return -1;
    }

    /* this is triggered too easily when fluctuating power
    if (solax.pv1_voltage && solax.pv1_wattage > 100 && solax.pv1_voltage/10*solax.pv1_current/10 > 150*solax.pv1_wattage/100) {
      master_log("cause 72\n");
      goto invalid;
    }
    if (solax.pv1_voltage && solax.pv1_wattage > 100 && solax.pv1_voltage/10*solax.pv1_current/10 < 50*solax.pv1_wattage/100) {
      master_log("cause 73\n");
      goto invalid; 
    }
    if (solax.pv2_voltage && solax.pv2_wattage > 100 && solax.pv2_voltage/10*solax.pv2_current/10 > 150*solax.pv2_wattage/100) {
      master_log("cause 74\n");
      goto invalid; 
    }
    if (solax.pv2_voltage && solax.pv2_wattage > 100 && solax.pv2_voltage/10*solax.pv2_current/10 < 50*solax.pv2_wattage/100) {
      master_log("cause 75\n");
      goto invalid; 
    }
    */
    /*
    // check power balance is correct (with a +- variance)
    if (power_balance_w < 0 && power_balance_w < - SOLAX_SELF_CONSUMPTION_MPPT_W - SOLAX_SELF_CONSUMPTION_INVERTER_W) {
      master_log("cause 76\n");
      goto invalid; 
    }
    if (power_balance_w > 0 && power_balance_w > SOLAX_SELF_CONSUMPTION_MPPT_W + SOLAX_SELF_CONSUMPTION_INVERTER_W) {
      master_log("cause 77\n");
      goto invalid; 
    }
    */
    // detect invalid packet (no power flows :s)
    if (solax.grid_wattage == 0 && solax.pv1_voltage == 0 && solax.pv2_voltage == 0 && solax.bat_wattage == 0 && solax.eps_voltage == 0 && solax.output_va == 0 && solax.grid_meter_ct == 0) {
      master_log("cause 78\n");
      return -2;
    }

    // only reset condition when a packet can be interpreted
    solax.valid_data = 1;
  
    solax_process_data();
  }  
}