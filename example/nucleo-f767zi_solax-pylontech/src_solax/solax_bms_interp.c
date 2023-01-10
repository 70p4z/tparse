
#include "main.h"
#include "tparse.h"
#include "stddef.h"
#include "stdio.h"

#ifdef MODE_SOLAX_BMS

#define USBVCP USART3

/**
SOLAX X1 <==CAN==> nucleo MODE_BMS_CAN <=(USART3)====(USART3)=> nucleo SLAVE <==CAN==> Pylontech SC0500
                   iobridge in solax makefile                  regular iobridge
*/
//#define BMS_PING
#define BMS_PING_INTERVAL_MS 5000

#define BMS_RECONNECT_DELAY 20000

#define SOLAX_PW_TIMEOUT 1000 // give it a second for 0xA0 bytes @ 9600bps
#define SOLAX_PW_NEXT_TIMEOUT 1000 // every 2 seconds, check it
#define SOLAX_PW_INVALID_RETRY_TIMEOUT 100 // 100ms before retrying in case of an error on the pocketwifi serial response
#define SOLAX_PW_MODE_CHANGE_MIN_INTERVAL 10000 // avoid changing mode constantly
#define PYLONTECH_ACTIVITY_TIMEOUT 500


#define SOLAX_PV_POWER_OPT_THRESHOLD_V 150
#define SOLAX_PV_POWER_OPT_THRESHOLD_W 25
#define SOLAX_GRID_EXPORT_OPT_THRESHOLD_W 50

#define SOLAX_MAX_PV_VOLTAGE_V 600 // from user manual

#define SOLAX_DAY_THRESHOLD_V 50 // below 50v is considered NIGHT (with high exposure nights, it has been measured as much)
#define SOLAX_SELF_CONSUMPTION_MPPT_W 140 // observed inverter consumption when MPPT is working
#define SOLAX_SELF_CONSUMPTION_INVERTER_W 40 // observed inverter consumption with only inverter enabled (not system off)

// before 80% of charge of battery, be conservative, and charge first
#define SOLAX_SELFUSE_MIN_BATTERY_SOC 80

#define DISPLAY_TIMEOUT 1000

#define S2LE(buf, off) ((int16_t)((int16_t)((int16_t)((int16_t)(buf)[off+1])<<8l) | (int16_t)((int16_t)(buf)[off]&0xFFl) ))
#define U2LE(buf, off) ((((buf)[off+1]&0xFFu)<<8) | ((buf)[off]&0xFFu) )
#define U2BE(buf, off) ((((buf)[off]&0xFFu)<<8) | ((buf)[off+1]&0xFFu) )

// for snprintf to work as expected
void _sbrk(void) {

}

uint32_t solax_checksum_verify(uint8_t *buf, uint16_t len) {
  uint16_t acc=0;
  uint16_t off=0;
  uint16_t l=len-2;
  while (l>=2) {
    acc+=U2BE(buf, off);
    off+=2;
    l-=2;
  }
  if (l) {
    acc+=buf[off]<<8;
    off++;
  }
  return acc == U2BE(buf,off);
}

void master_log(char* buffer) {
  uart_select_intf(USBVCP);
  uart_send(buffer);
}

void master_log_mem(void* _buffer, size_t length) {
  uint8_t* buffer = (uint8_t*)_buffer;
  uart_select_intf(USBVCP);
  uart_send_mem(buffer, length);
}

void master_log_hex(void* _buffer, size_t length) {
  uint8_t* buffer = (uint8_t*)_buffer;
  uart_select_intf(USBVCP);
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
      master_log(" s ");
      break;
    default:
      master_log_hex(&cid, 4);
      master_log(" U ");
      break;
  }
  master_log_hex(canmsg, canmsg_len);
  master_log("\n");
}

void can_solax_tx_log(uint32_t cid, size_t cid_bitlen, uint8_t* canmsg, size_t canmsg_len) {
  master_log_can("    >>> inv | ", cid, cid_bitlen, canmsg, canmsg_len);
  can_tx(CAN1, cid, cid_bitlen, canmsg, canmsg_len);
}

void can_bms_tx_log(uint32_t cid, size_t cid_bitlen, uint8_t* canmsg, size_t canmsg_len) {
  master_log_can("    >>> bms | ", cid, cid_bitlen, canmsg, canmsg_len);
  can_tx(CAN3, cid, cid_bitlen, canmsg, canmsg_len);
}

enum solax_pw_state_e {
  SOLAX_PW_IDLE,
  SOLAX_PW_SEND,
  SOLAX_PW_REQ_SENT,
  SOLAX_PW_WAIT_NEXT,
  SOLAX_PW_INVALID_NEXT,
};

const uint8_t solax_pw_cmd_get_stat[] = { 0xAA, 0x55, 0x07, 0x01, 0x10, 0x17, 0x01};

const uint8_t solax_pw_cmd_mode_self_use[]= { 0xAA, 0x55, 0x09, 0x09, 0x1C, 0x00, 0x00, 0x2D, 0x01 };
const uint8_t solax_pw_cmd_mode_manual[]= { 0xAA, 0x55, 0x09, 0x09, 0x1C, 0x03, 0x00, 0x30, 0x01 };
// useless // const uint8_t solax_pw_cmd_mode_backup[]= { 0xAA, 0x55, 0x09, 0x09, 0x1C, 0x02, 0x00, 0x30, 0x01 };
      
const uint8_t solax_pw_cfg_manual_stop[]= { 0xAA, 0x55, 0x09, 0x09, 0x24, 0x00, 0x00, 0x35, 0x01 };
const uint8_t solax_pw_cfg_manual_charge[]= { 0xAA, 0x55, 0x09, 0x09, 0x24, 0x01, 0x00, 0x35, 0x01 };
// useless // const uint8_t solax_pw_cfg_manual_discharge[]= { 0xAA, 0x55, 0x09, 0x09, 0x24, 0x02, 0x00, 0x35, 0x01 };

#define SOLAX_PW_QUEUE_SIZE 3 // schedule a mode change while reading a status response
struct {
  uint8_t*  cmd;
  uint32_t  cmd_len;
  uint32_t  rep_len;
} solax_pw_queue[SOLAX_PW_QUEUE_SIZE];

void solax_pw_queue_pop(void) {
  // consume the first slot
  memmove(&solax_pw_queue[0], &solax_pw_queue[1], sizeof(solax_pw_queue)-sizeof(solax_pw_queue[0]));
  memset(&solax_pw_queue[2], 0, sizeof(solax_pw_queue[0]));
}

uint32_t solax_pw_queue_free(void) {
  uint32_t idx=0;
  // seek for first free slot
  while (solax_pw_queue[idx].cmd_len != 0 && idx < SOLAX_PW_QUEUE_SIZE) {
    idx++;
  }
  return SOLAX_PW_QUEUE_SIZE - idx;
}

void solax_pw_queue_push(const uint8_t* cmd, uint32_t cmd_len, uint32_t rep_len) {
  uint32_t idx=0;
  // seek for first free slot
  while (solax_pw_queue[idx].cmd_len != 0 && idx < SOLAX_PW_QUEUE_SIZE) {
    idx++;
  }
  // full
  if (idx >= SOLAX_PW_QUEUE_SIZE) {
    return;
  }
  solax_pw_queue[idx].cmd = cmd;
  solax_pw_queue[idx].cmd_len = cmd_len;
  solax_pw_queue[idx].rep_len = rep_len;
}

const char * const solax_forced_mode_str [] = {
  "--",
  "SU",
  "BK",
  "F-",
  "FC",
  "FD",
};

int32_t batt_forced_soc = -1;
int32_t batt_forced_charge = -1;
uint32_t batt_drain_fix_cause = 0;
enum {
  SOLAX_FORCED_MODE_NONE,
  SOLAX_FORCED_MODE_SELF_USE,
  SOLAX_FORCED_MODE_BACKUP,
  SOLAX_FORCED_MODE_MANUAL_STOP,
  SOLAX_FORCED_MODE_MANUAL_CHARGE,
  SOLAX_FORCED_MODE_MANUAL_DISCHARGE,
} solax_forced_mode;
struct {
  uint16_t pv1_voltage;
  uint16_t pv2_voltage;
  uint16_t pv1_current;
  uint16_t pv2_current;
  uint16_t pv1_wattage;
  uint16_t pv2_wattage;
  int16_t bat_wattage;
  uint8_t mode;
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
  int16_t max_charge;
  int16_t max_discharge;
} pylontech;

void interp(void) {
  uint32_t forward;
  enum solax_pw_state_e solax_pw_state = SOLAX_PW_IDLE;
  uint32_t bms_reconnect_at;
  uint32_t solax_pw_timeout;
  uint32_t bms_ping_timeout;
  size_t len;
  uint32_t cid;
  size_t cid_bitlen;
  uint32_t enable_battery = 0;
  //uint32_t wait_bms_info = 1;
  //uint8_t bms_info[8];
  uint32_t timeout_next_display = uwTick;
  uint32_t solax_pw_mode_change_ready;
  uint32_t pylontech_timeout = 0;

  memset(&solax, 0, sizeof(solax));
  memset(&pylontech, 0, sizeof(pylontech));
  // init the queue
  memset(solax_pw_queue, 0, sizeof(solax_pw_queue));
  // ensure starting with SELF USE mode
  solax_pw_queue_push(solax_pw_cmd_mode_self_use, sizeof(solax_pw_cmd_mode_self_use), 7);
  solax_forced_mode = SOLAX_FORCED_MODE_SELF_USE;
  solax_pw_mode_change_ready = uwTick + SOLAX_PW_MODE_CHANGE_MIN_INTERVAL;

#ifdef MODE_FAKE_SOLAX
  while (1) {
    can_solax_tx_log(0x1871, CAN_ID_EXTENDED_LEN, (uint8_t*)"\x01\x00\x01\x00\x00\x00\x00\x00", 8);
    LL_mDelay(BMS_PING_INTERVAL_MS);
  }
#endif // MODE_FAKE_SOLAX

  tparse_ctx_t tp_u4;
  tparse_init(&tp_u4, uart4_buffer, sizeof(uart4_buffer), "");

  master_log("Reset\n");

  // sent ping to the BMS to wake it up at reset moment
  can_bms_tx_log(0x1871, CAN_ID_EXTENDED_LEN, (uint8_t*)"\x01\x00\x01\x00\x00\x00\x00\x00", 8);
  bms_ping_timeout = uwTick + BMS_PING_INTERVAL_MS;
  bms_reconnect_at = uwTick;
  pylontech_timeout = uwTick + PYLONTECH_ACTIVITY_TIMEOUT;

  while (1) {
    // check for messages from the inverter
    if (can_fifo_avail(CAN1)) {
      len = can_fifo_rx(CAN1, &cid, &cid_bitlen, tmp, sizeof(tmp));
      master_log_can("inv >>>     | ", cid, cid_bitlen, tmp, len);
      // other request from the inverter are discarded
      if (cid_bitlen && cid == 0x1871) {
        forward = 0;
        switch(tmp[0]) {
          case 1:
            // get data
            forward = 1;
            if (enable_battery==1) {
              can_solax_tx_log(0x0100A001, CAN_ID_EXTENDED_LEN, NULL, 0);
              enable_battery = 2;
            }
            can_solax_tx_log(0x1801, CAN_ID_EXTENDED_LEN, (uint8_t*)"\x01\x00\x01\x00\x00\x00\x00\x00", 8);
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
          can_bms_tx_log(cid, cid_bitlen, tmp, len);
        }
      }
    }

#ifdef BMS_PING
    // check for messages from the bms
     // time for a ping
    if (uwTick - bms_ping_timeout < 0x80000000UL) {
      can_bms_tx_log(0x1871, CAN_ID_EXTENDED_LEN, (uint8_t*)"\x01\x00\x01\x00\x00\x00\x00\x00", 8);
      //can_solax_tx_log(0x1801, CAN_ID_EXTENDED_LEN, (uint8_t*)"\x01\x00\x01\x00\x00\x00\x00\x00", 8);
      bms_ping_timeout = uwTick + BMS_PING_INTERVAL_MS;
    }
#endif // BMS_PING

    // timeout the pylontech status to avoid "bad" display
    if (pylontech_timeout && uwTick - pylontech_timeout < 0x80000000UL ) {
      memset(&pylontech, 0, sizeof(pylontech));
      pylontech_timeout = 0;
    }

    if (can_fifo_avail(CAN3)) {
      len = can_fifo_rx(CAN3, &cid, &cid_bitlen, tmp, sizeof(tmp));
      master_log_can("bms >>>     | ", cid, cid_bitlen, tmp, len);
      pylontech_timeout = uwTick + PYLONTECH_ACTIVITY_TIMEOUT;
      forward = 0;
      switch(cid) {
        case 0x1873:
          // may reinterpret SoC depending on battery voltage instead of relying on BMS
          pylontech.voltage = U2LE(tmp,0);
          pylontech.current = S2LE(tmp,2);
          pylontech.soc = U2LE(tmp,4);
          pylontech.wattage = ((int32_t)pylontech.voltage)*((int32_t)pylontech.current)/((int32_t)100); // unit 0.1V x 0.1A
          if (batt_forced_soc >= 0) {
            tmp[4] = batt_forced_soc&0xFF;
            tmp[5] = 0;
          }
          forward = 1;
          break;
        case 0x1872:
          pylontech.max_charge = S2LE(tmp, 4);
          pylontech.max_discharge = S2LE(tmp, 6);

          // add 0.9A to cover for the INVERTER wrong current computation 
          // (it includes its own DC consumption into the battery DC link)
          // when not full
          if (pylontech.max_charge && pylontech.soc != 100) {
            // NOTE: when charging from solar (grid charge is cheating), then MPPT is ON
            uint32_t self_consumption_current_dA = (SOLAX_SELF_CONSUMPTION_MPPT_W
                                                    + SOLAX_SELF_CONSUMPTION_INVERTER_W)*100/*cW*/ 
                                                   / pylontech.voltage/*dV*/;
            tmp[4] = (pylontech.max_charge+self_consumption_current_dA)&0xFF;
            tmp[5] = ((pylontech.max_charge+self_consumption_current_dA)>>8)&0xFF;
          }

          // force disabling charge, in order to avoid driving the battery to retrieve too less energy from the PVs.
          if (batt_forced_charge >= 0) {
            tmp[4] = batt_forced_charge&0xFF;
            tmp[5] = (batt_forced_charge>>8)&0xFF;
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
        can_solax_tx_log(cid, cid_bitlen, tmp, len);
      }
    }

    // handle solax PocketWifi port to get the pv arrays status
    tparse_finger(&tp_u4, sizeof(uart4_buffer) - DMA1_Stream2->NDTR);
    switch(solax_pw_state) {
      case SOLAX_PW_IDLE:
        // is no command scheduled for sending?
        if (solax_pw_queue_free() == SOLAX_PW_QUEUE_SIZE) {
          solax_pw_queue_push(solax_pw_cmd_get_stat, sizeof(solax_pw_cmd_get_stat), 0x197);
        }
        solax_pw_state = SOLAX_PW_SEND;
        break;

      case SOLAX_PW_SEND:
        tparse_discard(&tp_u4);
        // send info request to solax
        uart_select_intf(UART4);
        uart_send_mem(solax_pw_queue[0].cmd, solax_pw_queue[0].cmd_len);
        solax_pw_state = SOLAX_PW_REQ_SENT;
        solax_pw_timeout = uwTick + SOLAX_PW_TIMEOUT;
        break;

      case SOLAX_PW_REQ_SENT:
        // if the reply is complete
        if (tparse_avail(&tp_u4) >= solax_pw_queue[0].rep_len) {
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
            solax.mode        = tmp[25];
            solax.bat_wattage = S2LE(tmp, 37);
            solax.bat_temp = S2LE(tmp, 39);
            solax.bat_SoC = U2LE(tmp, 41);
            solax.eps_current = U2LE(tmp, 65);
            solax.grid_export_wattage = S2LE(tmp, 69);


            // if (!solax_checksum_verify(tmp+2,tmp[2]-2)) {
            //   batt_drain_fix_cause = 100;
            //   tparse_discard(&tp_u4);
            //   solax_pw_state = SOLAX_PW_INVALID_NEXT;
            //   solax_pw_timeout = uwTick + SOLAX_PW_INVALID_RETRY_TIMEOUT;
            //   break;
            // }


            int32_t power_balance_w = solax.pv1_wattage + solax.pv2_wattage - (solax.grid_wattage + pylontech.wattage );
            // check for invalid data (glitch sometimes returned by the inverter)
            if (solax.pv1_voltage > SOLAX_MAX_PV_VOLTAGE_V*10 || solax.pv2_voltage > SOLAX_MAX_PV_VOLTAGE_V*10) {
              batt_drain_fix_cause = 101;
            invalid:
              tparse_discard(&tp_u4);
              solax_pw_state = SOLAX_PW_INVALID_NEXT;
              solax_pw_timeout = uwTick + SOLAX_PW_INVALID_RETRY_TIMEOUT;
              solax_pw_queue_pop();
              break;
            }
            if (solax.pv1_voltage && solax.pv1_wattage > 100 && solax.pv1_voltage/10*solax.pv1_current/10 > 150*solax.pv1_wattage/100) {
              batt_drain_fix_cause = 102;
              goto invalid;
            }
            if (solax.pv1_voltage && solax.pv1_wattage > 100 && solax.pv1_voltage/10*solax.pv1_current/10 < 50*solax.pv1_wattage/100) {
              batt_drain_fix_cause = 103;
              goto invalid; 
            }
            if (solax.pv2_voltage && solax.pv2_wattage > 100 && solax.pv2_voltage/10*solax.pv2_current/10 > 150*solax.pv2_wattage/100) {
              batt_drain_fix_cause = 104;
              goto invalid; 
            }
            if (solax.pv2_voltage && solax.pv2_wattage > 100 && solax.pv2_voltage/10*solax.pv2_current/10 < 50*solax.pv2_wattage/100) {
              batt_drain_fix_cause = 105;
              goto invalid; 
            }
            // check power balance is correct (with a +- variance)
            if (power_balance_w < 0 && power_balance_w < - SOLAX_SELF_CONSUMPTION_MPPT_W - SOLAX_SELF_CONSUMPTION_INVERTER_W) {
              batt_drain_fix_cause = 106;
              goto invalid; 
            }
            if (power_balance_w > 0 && power_balance_w > SOLAX_SELF_CONSUMPTION_MPPT_W + SOLAX_SELF_CONSUMPTION_INVERTER_W) {
              batt_drain_fix_cause = 107;
              goto invalid; 
            }

            // only reset condition when a packet can be interpreted
            batt_forced_charge = -1;
            batt_forced_soc = -1;
            batt_drain_fix_cause = 0;

            if (pylontech.soc >= 100) {
              batt_drain_fix_cause = 3;
              batt_forced_charge = 0;
              if (solax_forced_mode != SOLAX_FORCED_MODE_SELF_USE
                 && solax_pw_queue_free()
                 && solax_pw_mode_change_ready == 0) {
                solax_pw_queue_push(solax_pw_cmd_mode_self_use, sizeof(solax_pw_cmd_mode_self_use), 7);
                solax_forced_mode = SOLAX_FORCED_MODE_SELF_USE;
                solax_pw_mode_change_ready = uwTick + SOLAX_PW_MODE_CHANGE_MIN_INTERVAL;
              }
            }

            // not NIGHT
            if (solax.pv1_voltage + solax.pv2_voltage > SOLAX_DAY_THRESHOLD_V*10 ) {

              // if total PV voltage is below a limit, just AVOID wasting battery energy, 
              // and disable battery charging to force the solax stopping the MPPT draining 
              // energy from the battery to operate.
              // ok but when not connected to grid, could not start! // if (solax.pv1_wattage + solax.pv2_wattage < 100) 
              // OFFGRID or NIGHT could still have that condition true 
              if (solax.pv1_voltage + solax.pv2_voltage < SOLAX_PV_POWER_OPT_THRESHOLD_V*10 
                  && solax.pv1_wattage + solax.pv2_wattage < SOLAX_PV_POWER_OPT_THRESHOLD_W)  // requires some insight on the total PV array connection
              {
                batt_drain_fix_cause = 1;
                batt_forced_charge = 0; // deny charge (which drains battery when not enough solar)
                // batt_forced_soc = SOLAX_BAT_MIN_SOC_SELFUSE;
                if (solax_pw_queue_free() >= 2 
                  && solax_forced_mode != SOLAX_FORCED_MODE_MANUAL_STOP
                  && solax_pw_mode_change_ready == 0) {
                  solax_pw_queue_push(solax_pw_cmd_mode_manual, sizeof(solax_pw_cmd_mode_manual), 7);
                  solax_pw_queue_push(solax_pw_cfg_manual_stop, sizeof(solax_pw_cfg_manual_stop), 7);
                  solax_forced_mode = SOLAX_FORCED_MODE_MANUAL_STOP;
                  solax_pw_mode_change_ready = uwTick + SOLAX_PW_MODE_CHANGE_MIN_INTERVAL;
                }
              }
              // if the solar is LESS than the house consumes, then don't try to charge
              else if (
                // battery is emptying in the grid, and panels aren't coping with that
                // observed than under a certain pv power, when the grid is connected, then the battery is used to power the mppt
                // grid tied
                solax.grid_wattage /* == SELFUSE*/
                // not enough power from solar to cover solax self consumption (observed threshold)
                && solax.pv1_wattage + solax.pv2_wattage 
                   < SOLAX_SELF_CONSUMPTION_MPPT_W 
                     + SOLAX_SELF_CONSUMPTION_INVERTER_W
                ) {
                batt_drain_fix_cause = 4;
                batt_forced_charge = 0; // deny charge
                // batt_forced_soc = SOLAX_BAT_MIN_SOC_SELFUSE; // deny discharge for self-use

                if (solax_pw_queue_free() >= 2 
                  && solax_forced_mode != SOLAX_FORCED_MODE_MANUAL_STOP
                  && solax_pw_mode_change_ready == 0) {
                  solax_pw_queue_push(solax_pw_cmd_mode_manual, sizeof(solax_pw_cmd_mode_manual), 7);
                  solax_pw_queue_push(solax_pw_cfg_manual_stop, sizeof(solax_pw_cfg_manual_stop), 7);
                  solax_forced_mode = SOLAX_FORCED_MODE_MANUAL_STOP;
                  solax_pw_mode_change_ready = uwTick + SOLAX_PW_MODE_CHANGE_MIN_INTERVAL;
                }
              }
              else if ( 
                // grid tied
                solax.grid_wattage /* == SELFUSE*/ 
                && solax.pv1_wattage + solax.pv2_wattage > SOLAX_SELF_CONSUMPTION_INVERTER_W
                // we're inverting everything to the self use, nothing left for battery charging :(
                && solax.pv1_wattage + solax.pv2_wattage 
                   < solax.grid_wattage
                     + SOLAX_SELF_CONSUMPTION_MPPT_W + SOLAX_SELF_CONSUMPTION_INVERTER_W
                // NOTE: if battery not charging fast (end of charge) then the condition is not respected, and 
                //       we don't stay in selfuse due to these conditions :(
                ) {
                // battery has nothing left to charge, discard it
                batt_drain_fix_cause = 5;
                batt_forced_charge = 0;
                // avoid discharge completely by violating discharge level of the battery
                // batt_forced_soc = SOLAX_BAT_MIN_SOC_SELFUSE; // deny discharge for self-use

                if (solax_pw_queue_free() >= 2 
                  && solax_forced_mode != SOLAX_FORCED_MODE_MANUAL_STOP
                  && solax_pw_mode_change_ready == 0) {
                  solax_pw_queue_push(solax_pw_cmd_mode_manual, sizeof(solax_pw_cmd_mode_manual), 7);
                  solax_pw_queue_push(solax_pw_cfg_manual_stop, sizeof(solax_pw_cfg_manual_stop), 7);
                  solax_forced_mode = SOLAX_FORCED_MODE_MANUAL_STOP;
                  solax_pw_mode_change_ready = uwTick + SOLAX_PW_MODE_CHANGE_MIN_INTERVAL;
                }
              }
              else if (solax.pv1_wattage + solax.pv2_wattage 
                       /*> SOLAX_SELF_CONSUMPTION_MPPT_W + SOLAX_SELF_CONSUMPTION_INVERTER_W*/
                // CHARGE first strategy
                && ( pylontech.soc >= SOLAX_SELFUSE_MIN_BATTERY_SOC
                         // or more power to be drained from panel (possibly) (and is needed because of grid import)
                     || (-solax.grid_export_wattage > SOLAX_GRID_EXPORT_OPT_THRESHOLD_W
                         // if we charging above max charge minus 10%, then consider there is room for self-use
                         && pylontech.current >= pylontech.max_charge - 10*pylontech.max_charge/100  // (in 0.1A)
                         // and still power remaining for the battery (considering the import that will be compensated)
                         && -solax.grid_export_wattage < pylontech.current * pylontech.voltage / 10 / 10 // (in W)
                                                         - 10 * pylontech.current * pylontech.voltage / 10 / 10 / 100
                         ) )
                     ) {
                batt_drain_fix_cause = 6;
                if (solax_pw_queue_free()
                  && solax_forced_mode != SOLAX_FORCED_MODE_SELF_USE
                  && solax_pw_mode_change_ready == 0) {
                  solax_pw_queue_push(solax_pw_cmd_mode_self_use, sizeof(solax_pw_cmd_mode_self_use), 7);
                  solax_forced_mode = SOLAX_FORCED_MODE_SELF_USE;
                  solax_pw_mode_change_ready = uwTick + SOLAX_PW_MODE_CHANGE_MIN_INTERVAL;
                }
              }
            }
            else {
              // NIGHT
              if (solax_pw_queue_free() 
                && solax_forced_mode != SOLAX_FORCED_MODE_SELF_USE
                && solax_pw_mode_change_ready == 0) {
                solax_pw_queue_push(solax_pw_cmd_mode_self_use, sizeof(solax_pw_cmd_mode_self_use), 7);
                solax_forced_mode = SOLAX_FORCED_MODE_SELF_USE;
                solax_pw_mode_change_ready = uwTick + SOLAX_PW_MODE_CHANGE_MIN_INTERVAL;
              }
            }
          }
          // whatever the reply, discard the data after this point
          tparse_discard(&tp_u4);
          solax_pw_state = SOLAX_PW_WAIT_NEXT; // switch state before possibly scheduling a command to send
          solax_pw_timeout = uwTick + SOLAX_PW_NEXT_TIMEOUT;
          solax_pw_queue_pop();
        }
        else if (uwTick - solax_pw_timeout < 0x80000000UL) {
          tparse_discard(&tp_u4);
          solax_pw_state = SOLAX_PW_WAIT_NEXT;
          solax_pw_timeout = uwTick + SOLAX_PW_NEXT_TIMEOUT;
          solax_pw_queue_pop();
          // discard causes due to timeout of the request
          batt_drain_fix_cause = 0;
          batt_forced_charge = -1;
          batt_forced_soc = -1;
        }
        break;

      case SOLAX_PW_INVALID_NEXT:
      case SOLAX_PW_WAIT_NEXT:
        // skip to next command sending immediately, this is not a new attempt/request
        if (uwTick - solax_pw_timeout < 0x80000000UL || solax_pw_queue_free() != SOLAX_PW_QUEUE_SIZE) {
          solax_pw_state = SOLAX_PW_IDLE;
        }
        break;
    }

    if (uwTick - solax_pw_mode_change_ready < 0x80000000UL) {
      solax_pw_mode_change_ready = 0;
    }

    // display
    if (uwTick - timeout_next_display < 0x80000000UL) {
      // prepare next sending
      timeout_next_display = uwTick + DISPLAY_TIMEOUT;
      {
        int32_t bat_wattage_corr = solax.bat_wattage
          +(solax.bat_wattage?(solax.bat_wattage>0?-SOLAX_SELF_CONSUMPTION_INVERTER_W-SOLAX_SELF_CONSUMPTION_MPPT_W:+SOLAX_SELF_CONSUMPTION_INVERTER_W+SOLAX_SELF_CONSUMPTION_MPPT_W):0);
        uart_select_intf(UART5);
        // wipe screen
        if (solax.grid_export_wattage >= 0) {
          snprintf(tmp, sizeof(tmp), VERSION " GRID<< %dW | %s:%d %d\n", solax.grid_export_wattage, solax_forced_mode_str[solax_forced_mode], solax.mode, batt_drain_fix_cause);
        }
        else {
          snprintf(tmp, sizeof(tmp), VERSION " GRID>> %dW | %s:%d %d\n", -solax.grid_export_wattage, solax_forced_mode_str[solax_forced_mode], solax.mode, batt_drain_fix_cause);
        }
        uart_send_mem(tmp, strlen(tmp));
        if (solax.grid_wattage >= 0) {
          snprintf(tmp, sizeof(tmp), "HOUSE<< %dW BAT:%dW\n", solax.grid_wattage, solax.bat_wattage);
        }
        else {
          snprintf(tmp, sizeof(tmp), "HOUSE>> %dW BAT:%dW\n", -solax.grid_wattage, solax.bat_wattage);
        }
        uart_send_mem(tmp, strlen(tmp));

        // BEGIN BATT
        if (pylontech.wattage >= 0) {
          snprintf(tmp, sizeof(tmp), "BAT<< %dW", pylontech.wattage);
        }
        else {
          snprintf(tmp, sizeof(tmp), "BAT>> %dW", -pylontech.wattage);
        }
        uart_send_mem(tmp, strlen(tmp));
        
        #if 0
        // display current flowing through the battery
        int16_t cur = pylontech.current;
        if (cur < 0) {
          uart_send_mem("-", 1);
          cur = -cur;
        }
        snprintf(tmp, sizeof(tmp), " %d.%dA", cur/10, cur%10)
        uart_send_mem(tmp, strlen(tmp));
        #endif
        if (batt_forced_soc== -1) {
          snprintf(tmp, sizeof(tmp), " %u%% %dA\n", pylontech.soc, (batt_forced_charge==-1?pylontech.max_charge:batt_forced_charge)/10);
        }
        else {
          snprintf(tmp, sizeof(tmp), " %u%% %dA F%u%%\n", pylontech.soc, (batt_forced_charge==-1?pylontech.max_charge:batt_forced_charge)/10, batt_forced_soc);
        }
        uart_send_mem(tmp, strlen(tmp));
        // END BAT
        snprintf(tmp, sizeof(tmp), "PV %uV %dW%s%s %d\n", solax.pv1_voltage/10, solax.pv1_wattage, solax.pv1_voltage + solax.pv2_voltage <= SOLAX_DAY_THRESHOLD_V * 10 ? " NI":" DA", batt_drain_fix_cause==1?" LO":"", bat_wattage_corr);
        uart_send_mem(tmp, strlen(tmp));
      }
    }

  } // end infinited loop

}
#endif // MODE_SOLAX_BMS
