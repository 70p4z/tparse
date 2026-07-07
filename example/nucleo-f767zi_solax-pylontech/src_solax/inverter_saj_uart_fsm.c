#include "main.h"
#include "tparse.h"
#include "stddef.h"
#include "stdio.h"
#include "stdbool.h"
#include "globals.h"

#include "hashmap_u16_u32.h"

#ifdef INVERTER_SAJ

#define INVERTER_UART_TIMEOUT_MS 2000 // give few seconds for 400 bytes @ 9600bps
#define INVERTER_UART_NEXT_TIMEOUT 1000 // pocket wifi link update
#define INVERTER_UART_INVALID_RETRY_TIMEOUT 500 // 100ms before retrying in case of an error on the pocketwifi serial response

#define SAJ_COMMAND_READ 0x03
#define SAJ_COMMAND_WRITE_SINGLE 0x06
#define SAJ_COMMAND_WRITE 0x10

void inverter_uart_idle(void);

// return 0 when OK, anything else is error
uint32_t inverter_uart_parse_response(uint8_t* buffer, uint32_t length);

enum inverter_uart_state_e {
  INVERTER_UART_IDLE,
  INVERTER_UART_SEND,
  INVERTER_UART_REQ_SENT,
  INVERTER_UART_WAIT_NEXT,
  INVERTER_UART_INVALID_NEXT,
} inverter_uart_state;

uint32_t inverter_uart_timeout;

// structure to store u32 for each value read inside the inverter
hm_t saj_cache;

#define INVERTER_UART_QUEUE_SIZE 10 // schedule a mode change while reading a status response
struct {
  uint8_t*  cmd;
  uint32_t  cmd_len;
  uint32_t  rep_len;
} inverter_uart_queue[INVERTER_UART_QUEUE_SIZE];


void inverter_uart_queue_pop(void) {
  // consume the first slot
  memmove(&inverter_uart_queue[0], &inverter_uart_queue[1], sizeof(inverter_uart_queue)-sizeof(inverter_uart_queue[0]));
  memset(&inverter_uart_queue[INVERTER_UART_QUEUE_SIZE-1], 0, sizeof(inverter_uart_queue[INVERTER_UART_QUEUE_SIZE-1]));
}

// return last sent command
uint8_t* inverter_uart_queue_get(void) {
  return inverter_uart_queue[0].cmd;
}

uint32_t inverter_uart_queue_free(void) {
  uint32_t idx=0;
  // seek for first free slot
  while (inverter_uart_queue[idx].cmd_len != 0 && idx < INVERTER_UART_QUEUE_SIZE) {
    idx++;
  }
  return INVERTER_UART_QUEUE_SIZE - idx;
}

void inverter_uart_queue_push(const uint8_t* cmd, uint32_t cmd_len, uint32_t rep_len) {
  uint32_t idx=0;
  // seek for first free slot
  while (inverter_uart_queue[idx].cmd_len != 0 && idx < INVERTER_UART_QUEUE_SIZE) {
    idx++;
  }
  // full
  if (idx >= INVERTER_UART_QUEUE_SIZE) {
    return;
  }
  inverter_uart_queue[idx].cmd = (uint8_t*)cmd;
  inverter_uart_queue[idx].cmd_len = cmd_len;
  inverter_uart_queue[idx].rep_len = rep_len;
}

tparse_ctx_t tp_inv_uart;

void inverter_uart_init(void) {
  inverter_uart_timeout=0;
  inverter_uart_state = INVERTER_UART_IDLE;
  memset(inverter_uart_queue, 0, sizeof(inverter_uart_queue));
  tparse_init(&tp_inv_uart, uart_pw_buffer, sizeof(uart_pw_buffer), "");

  // 116200 8N1 INVERTED
  Configure_UARTPW(115200, 1);

  // init value storage
  hm_init(&saj_cache);
}

// abstract UART state machine
void inverter_uart_update(void) {
  // handle solax PocketWifi port to get the solax status
  tparse_finger(&tp_inv_uart, sizeof(uart_pw_buffer) - DMA_Stream_PW->NDTR);
  switch(inverter_uart_state) {
    case INVERTER_UART_IDLE:
      // is no command scheduled for sending?
      if (inverter_uart_queue_free() == INVERTER_UART_QUEUE_SIZE) {
        inverter_uart_idle();
        inverter_uart_state = INVERTER_UART_SEND;
      }
      break;

    case INVERTER_UART_SEND:
      // data request to be transmitted toward the inverter
      if (inverter_uart_queue_free() != INVERTER_UART_QUEUE_SIZE) {
      send_next:
        tparse_discard(&tp_inv_uart);
        master_log("UARTINV >> ");
        master_log_hex(inverter_uart_queue[0].cmd, inverter_uart_queue[0].cmd_len);
        master_log("\n");
        // send info request to solax
        uart_select_intf(UARTPW);
        uart_send_mem(inverter_uart_queue[0].cmd, inverter_uart_queue[0].cmd_len);
        inverter_uart_state = INVERTER_UART_REQ_SENT;
        inverter_uart_timeout = EXPIRE_IN(INVERTER_UART_TIMEOUT_MS);
      }
      break;

    case INVERTER_UART_REQ_SENT:
      // if the reply is complete
      if (tparse_avail(&tp_inv_uart) >= inverter_uart_queue[0].rep_len) {
        size_t read = tparse_read(&tp_inv_uart, (char*)tmp, inverter_uart_queue[0].rep_len);
        if (read < inverter_uart_queue[0].rep_len) {
          master_log("UARTINV reading error ");
          master_log_hex(&read, 4);
          read = tparse_avail(&tp_inv_uart);
          master_log_hex(&read, 4);
          master_log("\n");
          goto invalid;
        }
        master_log("UARTINV << ");
        master_log_hex(tmp, inverter_uart_queue[0].rep_len);
        master_log("\n");

        uint32_t parse_error = inverter_uart_parse_response(tmp, inverter_uart_queue[0].rep_len);
        inverter_uart_queue_pop();

        if (parse_error) {
        invalid:
          inverter_uart_state = INVERTER_UART_INVALID_NEXT;
          inverter_uart_timeout = EXPIRE_IN(INVERTER_UART_NEXT_TIMEOUT);
          goto error_flush;
        }

        // parsing was ok, still some command to send
        if (inverter_uart_queue_free() != INVERTER_UART_QUEUE_SIZE) {
          goto send_next;
        }
        // parsing was ok, no more command to send
        else {
          // will enter idle again after timeout
          inverter_uart_state = INVERTER_UART_WAIT_NEXT; 
          inverter_uart_timeout = EXPIRE_IN(INVERTER_UART_NEXT_TIMEOUT);
        }
      }
      // timing out first entry if any
      else if (inverter_uart_queue_free() != INVERTER_UART_QUEUE_SIZE 
        && inverter_uart_timeout && EXPIRED(inverter_uart_timeout)) {
        master_log("UARTINV TIMEOUT\n");
        //master_log_hex(uart_pw_buffer, sizeof(uart_pw_buffer));
        inverter_uart_state = INVERTER_UART_WAIT_NEXT;
        inverter_uart_timeout = EXPIRE_IN(1); // RIGHT NOW
      error_flush:
        tparse_reset(&tp_inv_uart);
        // flush queue
        while (inverter_uart_queue_free() != INVERTER_UART_QUEUE_SIZE) {
          inverter_uart_queue_pop();
        }
      }
      break;

    case INVERTER_UART_INVALID_NEXT:
    case INVERTER_UART_WAIT_NEXT:
      // skip to next command sending immediately, this is not a new attempt/request
      if ((inverter_uart_timeout && EXPIRED(inverter_uart_timeout))
        || inverter_uart_queue_free() != INVERTER_UART_QUEUE_SIZE) {
        inverter_uart_timeout = 0;
        inverter_uart_state = INVERTER_UART_IDLE;
      }
      break;
  }
}


#define MAX_PV_VOLTAGE_V 600 // from user manual

static const uint16_t crc16_modbus_table[256] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040,
};

uint16_t crc16_modbus(const uint8_t *buf, size_t len)
{
    uint16_t crc = 0xFFFF;

    while (len--) {
        crc = (crc >> 8) ^ crc16_modbus_table[(crc ^ *buf++) & 0xFF];
    }

    return crc;
}

uint32_t saj_checksum_verify(uint8_t* packet, uint16_t len) {
  return crc16_modbus(packet, len-2) == U2LE(packet, len-2);
}

void inverter_uart_force_bitrate(void) {
  // Configure_UARTPW(115200, 0);
}

/*
SAJ MODBUS PROTOCOL REVERSE
===========================

Read
  D@  CM  REGAD REGNB CRC16
  01  03  8F 00 00 1D AF 17

Read 
  @@ CMD START COUNT  CRC16
  01 03  34 21 00 04  DA 30

  Resp Read
  @@ CMD DATA...             CRC16
  01 03  AABB CCDD EEFF GGHH XX YY
  with AABB = *0x3421, CCDD = *0x3422 ... 

Write Multiple
  @@ CMD START COUNT BYTES REGVAL CRC16
  01 10  34 21 00 01 02    00 05  15 21 

  Response
  @@ CMD START COUNT CRC16
  01 10  34 21 00 01 XX YY

*/

/*
REALTIME_DATA_MAP = [  37: 0x25
    ("mpvmode", None), # 0x4004
    ("faultMsg0", "32u"), # 0x4005 0x4006
    ("faultMsg1", "32u"), # 0x4007 0x4008
    ("faultMsg2", "32u"), # 0x4009 0x400A
    (None, "skip_bytes", 8), # 0x400B 0x400C 0x400D 0x400E
    ("errorcount", None),    # 0x400F # not in doc
    ("SinkTemp", "16i", 0.1), # 0x4010
    ("AmbTemp", "16i", 0.1), # 0x4011
    ("gfci", None),          # 0x4012
    ("iso1", None),          # 0x4013
    ("iso2", None),          # 0x4014
    ("iso3", None),          # 0x4015
    ("iso4", None),          # 0x4016
    ("DRM_HW_STAT", None),          # 0x4017
    ("DRM_SW_STAT", None),          # 0x4018
    ("GridConnCountdown", None),          # 0x4019
    ("ErrDataSN", None),          # 0x401A
    ("SettingDataSN", None),          # 0x401B
    ("FCATriggerFlag", None),          # 0x401C
    ("FunctionOne", None),          # 0x401D
    ("FunctionTwo", None),          # 0x401E
    ("FunctionThree", None),          # 0x401F
    ("FunctionFour", None),          # 0x4020
    ("FunctionFive", None),          # 0x4021
    ("AppMode", None),          # 0x4022
    ("InvDisPowerSet", None),          # 0x4023
    ("InvChgPowerSet", None),          # 0x4024
    ("BatDisCurrSet", None),          # 0x4025
    ("BatChgCurrSet", None),          # 0x4026
    ("BatStatusDisp", None),          # 0x4027
    ("BatProtocolSet", None),          # 0x4028
]

ADDITIONAL_DATA_1_PART_1_MAP = [ # 15: 0x0F
    ("BatTemp", "16i", 0.1),   # 0x406E
    ("batEnergyPercent", None), #0x406F
    (None, "skip_bytes", 2), # 0x4070
    ("pv1Voltage", None, 0.1), # 0x4071
    ("pv1TotalCurrent", None), # 0x4072
    ("pv1Power", None, 1), # 0x4073
    ("pv2Voltage", None, 0.1), # 0x4074
    ("pv2TotalCurrent", None), # 0x4075
    ("pv2Power", None, 1), # 0x4076
    ("pv3Voltage", None, 0.1), # 0x4077
    ("pv3TotalCurrent", None), # 0x4078
    ("pv3Power", None, 1), # 0x4079
    ("pv4Voltage", None, 0.1), # 0x407A
    ("pv4TotalCurrent", None), # 0x407B
    ("pv4Power", None, 1), # 0x407C
]

ADDITIONAL_DATA_1_PART_2_MAP = [ # 26: 0x1A
    ("directionPV", None),  # 0x4095
    ("directionBattery", "16i"), # 0x4096
    ("directionGrid", "16i"), # 0x4097
    ("directionOutput", None), # 0x4098
    (None, "skip_bytes", 14), # 0x4099 pv to load, 
                              # 0x409A grid to load, 
                              # 0x409B pv to grid, 
                              # 0x409C bat to grid, 
                              # 0x409D bat to load
                              # 0x409E pv to bat
                              # 0x409F grid to bat
    ("TotalLoadPower", "16i"),  # 0x40A0 total load power
    ("CT_GridPowerWatt", "16i"), # 0x40A1 CT grid real power
    ("CT_GridPowerVA", "16i"), # 0x40A2 CT grid apparent powerVA
    ("CT_PVPowerWatt", "16i"), # 0x40A3 CT PV real power
    ("CT_PVPowerVA", "16i"), # 0x40A4 CT PV apparent power
    ("pvPower", "16i"), # 0x40A5 total PV power
    ("batteryPower", "16i"), # 0x40A6 total bat power
    ("totalgridPower", "16i"), # 0x40A7 total grid power
    ("totalgridPowerVA", "16i"), # 0x40A8 total grid apparent power
    ("inverterPower", "16i"), # 0x40A9 totla inverter real power
    ("TotalInvPowerVA", "16i"),# 0x40AA total inverter apparent power
    ("BackupTotalLoadPowerWatt", None), # 0x40AB total backup real power
    ("BackupTotalLoadPowerVA", None), # 0x40AC total backup apparent power
    ("gridPower", "16i"), # 0x40AD grid system real power
    ("gridPower", "16i"), # 0x40AE grid system real power


PASSIVE_BATTERY_DATA_MAP = [ 
    ("passive_charge_enable", "16u", 1),     # 0x3636
    ("passive_grid_charge_power", "16u"),    # 0x3637
    ("passive_grid_discharge_power", "16u"), # 0x3638
    ("passive_bat_charge_power", "16u"),     # 0x3639 
    ("passive_bat_discharge_power", "16u"),  # 0x363A
    (None, "skip_bytes", 18),        # 0x363B 0x363C 0x363D 0x363E 0x363F 0x3640 0x3641 0x3642 0x3643
    ("BatOnGridDisDepth", "16u", 1), # 0x3644
    ("BatOffGridDisDepth", "16u", 1), # 0x3645
    ("BatcharDepth", "16u", 1), # 0x3646
    ("AppMode", "16u", 1), # 0x3647
    (None, "skip_bytes", 10), # 0x3648 0x3649 0x364A 0x364B 0x364C
    ("BatChargePower", "16u"), # 0x364D
    ("BatDischargePower", "16u"), # 0x364E
    ("GridChargePower", "16u"), # 0x364F
    ("GridDischargePower", "16u"), # 0x3650
    (None, "skip_bytes", 18), # 0x3651 0x3652 0x3653 0x3654 0x3655 0x3656 0x3657 0x3658 0x3659
    ("AntiRefluxPowerLimit", "16u", 1), # 0x365A
    ("AntiRefluxCurrentLimit", "16u", 1), # 0x365B
    ("AntiRefluxCurrentmode_raw", "16u", 1), # 0x365C
    (None, "skip_bytes", 4), # 0x365D 0x365E
    ("tou_outside_mode", "16u", 1),  # 0x365F: 0=Standby, 1=Self Use Mode
    ("time_bat_dis", "16u", 1),  # 0x3660: 0=Not allow, 1=Allow charge/discharge in time-sharing mode
]

*/

// values to be firstly decoded as u32
const uint16_t saj_u32_values[] = {
  0x4005,
  0x4007,
  0x4009,
};

// TODO compute CRC on the fly

const uint8_t saj_read_rt[] = {
  // READDDD              START       COUNT       CRC16
  0x01, SAJ_COMMAND_READ, 0x40, 0x04, 0x00, 0x25,      0xD0, 0x10
};

const uint8_t saj_read_pv[] = {
  // READDDD              START       COUNT       CRC16
  0x01, SAJ_COMMAND_READ, 0x40, 0x6E, 0x00, 0x0F,      0x71, 0xD3
};

const uint8_t saj_read_power[] = {
  // READDDD              START       COUNT       CRC16
  0x01, SAJ_COMMAND_READ, 0x40, 0x95, 0x00, 0x1A,      0xC1, 0xED
};

const uint8_t saj_read_settings[] = {
  // READDDD              START       COUNT       CRC16
  0x01, SAJ_COMMAND_READ, 0x36, 0x00, 0x00, 0x60,      0x4A, 0x6A
};

#define READ_MULTIPLE_REPLY_LENGTH(reg_count) (2+1+reg_count*2+2)
#define READ_MULTIPLE_REPLY_LENGTH_FROM_CMD(cmd) (2+1+((cmd)[5])*2+2) /*ignore high byte, not supported!*/

void inverter_uart_idle(void) {
  inverter_uart_queue_push(saj_read_rt, sizeof(saj_read_rt), READ_MULTIPLE_REPLY_LENGTH_FROM_CMD(saj_read_rt));
  inverter_uart_queue_push(saj_read_pv, sizeof(saj_read_pv), READ_MULTIPLE_REPLY_LENGTH_FROM_CMD(saj_read_pv));
  inverter_uart_queue_push(saj_read_power, sizeof(saj_read_power), READ_MULTIPLE_REPLY_LENGTH_FROM_CMD(saj_read_power));
  inverter_uart_queue_push(saj_read_settings, sizeof(saj_read_settings), READ_MULTIPLE_REPLY_LENGTH_FROM_CMD(saj_read_settings));
}

const uint8_t test_crc[] = {
  0x01, 0x03, 0x4a, 0x00, 0x04, 0x00, 0x00, 0x00, 0x22, 0x10, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0xd7, 0x00, 0xbe, 0xff, 0xf5, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x48, 0x1b, 0x80, 0x00, 0x00, 0x00, 0x79, 0xfc, 0x21, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x15, 0x13, 0x9E
};

uint32_t inverter_uart_parse_response(uint8_t* reply, uint32_t length) {
  uint8_t* cmd = inverter_uart_queue_get();

  // validate checksum first
  if (!saj_checksum_verify(reply, length)) {
    return -1;
  }

  // extract values from the command
  if (reply[1] == SAJ_COMMAND_READ) {
    uint16_t reg_addr = U2BE(cmd, 2);
    uint16_t reg_count = U2BE(cmd, 4);
    uint16_t bytes_count = reply[2];

    // invalid response encoding
    // should 
    if (length != READ_MULTIPLE_REPLY_LENGTH(reg_count)
      || length != 2 + 1 + bytes_count + 2) {
      return -2;
    }

    uint8_t* ptr = reply+3;
    while(reg_count && bytes_count) {
      hm_put(&saj_cache, reg_addr, U2BE(ptr, 0));
      ptr+=2;
      reg_addr++;
      bytes_count-=2;
      reg_count--;
    }
  }

  // when last status is received, then run the value converter
  if (inverter_uart_queue_free() == INVERTER_UART_QUEUE_SIZE - 1) {
    uint32_t val;
    #define VAL_U16(dest, reg_addr) { if (hm_get(&saj_cache, reg_addr, &val)) {inverter. dest = val;} }
    #define VAL_I16(dest, reg_addr) { if (hm_get(&saj_cache, reg_addr, &val)) {inverter. dest = (int16_t)val;} }
    VAL_U16(status, 0x4004);
    VAL_I16(grid_wattage, 0x40A1);
    VAL_U16(pv1_wattage, 0x4073);
    VAL_U16(pv2_wattage, 0x4076);
    //VAL_U16(pv3_wattage, 0x4079);
    // inverter.grid_wattage = 
    VAL_I16(bat_wattage, 0x40A6);
    VAL_I16(eps_power, 0x40AB);
    VAL_I16(grid_meter_ct, 0x40A7); // total grid power
  }

  // check it's the expected response
  if (reply[0] == 0xAA && reply[1] == 0x55 && reply[2] == 0x5F && reply[3] == 0x81 && reply[4] == 0x90 ) {

    // invalid until tested valid
    inverter.valid_data = 0;

    // extract fields
    inverter.grid_wattage = S2LE(reply, 9);
    inverter.pv1_voltage = U2LE(reply, 13);
    inverter.pv2_voltage = U2LE(reply, 15);
    inverter.pv1_current = U2LE(reply, 17);
    inverter.pv2_current = U2LE(reply, 19);
    inverter.pv1_wattage = U2LE(reply, 21);
    inverter.pv2_wattage = U2LE(reply, 23);
    if (reply[25] != inverter.status) {
      inverter.status_count=0;
    }
    inverter.status      = reply[25];
    if (inverter.status_count<255) {
      inverter.status_count++;
    }
    inverter.bat_wattage = S2LE(reply, 37);
    inverter.bat_temp = S2LE(reply, 39);
    inverter.bat_SoC = U2LE(reply, 41);
    inverter.output_va = U2LE(reply, 55);
    inverter.eps_power = U2LE(reply, 61);
    inverter.eps_voltage = U2LE(reply, 63);
    inverter.eps_current = U2LE(reply, 65);
    inverter.grid_meter_ct = S2LE(reply, 69);
    inverter.seconds = reply[203];
    inverter.minute = reply[204];
    inverter.hour = reply[205];
    inverter.day = reply[206];
    inverter.month = reply[207];
    inverter.year = reply[208] + 2000;

    //uint32_t valid_crc = solax_checksum_verify(reply+2,reply[2]-2);

    snprintf((char*)tmp, sizeof(tmp), "PV1: %dW (%d.%dV %d.%dA)\nPV2: %dW (%d.%dV %d.%dA)\n", inverter.pv1_wattage, inverter.pv1_voltage/10,inverter.pv1_voltage%10, inverter.pv1_current/10, inverter.pv1_current%10, inverter.pv2_wattage, inverter.pv2_voltage/10, inverter.pv2_voltage%10, inverter.pv2_current/10, inverter.pv2_current%10);
    master_log((char*)tmp);
    snprintf((char*)tmp, sizeof(tmp), "AC: Grid: %dW (meter %dW) EPS: %dW Output: %dVA\n", inverter.grid_wattage, inverter.grid_meter_ct, inverter.eps_power, inverter.output_va);
    master_log((char*)tmp);

    // if (!valid_crc) {
    //   return -3;
    // }

    //int32_t pylontech_wattage = pylontech.precise_wattage?pylontech.precise_wattage:pylontech.wattage;
    //int32_t power_balance_w = inverter.pv1_wattage + inverter.pv2_wattage - (inverter.grid_wattage + pylontech_wattage );
    // check for invalid data (glitch sometimes returned by the inverter)
    if (inverter.pv1_voltage > MAX_PV_VOLTAGE_V*10 || inverter.pv2_voltage > MAX_PV_VOLTAGE_V*10) {
      master_log("cause 71\n");
      return -1;
    }

    /* this is triggered too easily when fluctuating power
    if (inverter.pv1_voltage && inverter.pv1_wattage > 100 && inverter.pv1_voltage/10*inverter.pv1_current/10 > 150*inverter.pv1_wattage/100) {
      master_log("cause 72\n");
      goto invalid;
    }
    if (inverter.pv1_voltage && inverter.pv1_wattage > 100 && inverter.pv1_voltage/10*inverter.pv1_current/10 < 50*inverter.pv1_wattage/100) {
      master_log("cause 73\n");
      goto invalid; 
    }
    if (inverter.pv2_voltage && inverter.pv2_wattage > 100 && inverter.pv2_voltage/10*inverter.pv2_current/10 > 150*inverter.pv2_wattage/100) {
      master_log("cause 74\n");
      goto invalid; 
    }
    if (inverter.pv2_voltage && inverter.pv2_wattage > 100 && inverter.pv2_voltage/10*inverter.pv2_current/10 < 50*inverter.pv2_wattage/100) {
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
    if (inverter.grid_wattage == 0 && inverter.pv1_voltage == 0 && inverter.pv2_voltage == 0 && inverter.bat_wattage == 0 && inverter.eps_voltage == 0 && inverter.output_va == 0 && inverter.grid_meter_ct == 0) {
      master_log("cause 78\n");
      return -2;
    }

    // only reset condition when a packet can be interpreted
    inverter.valid_data = 1;
  
    solax_process_data();
  }  
  return 0;
}

void solax_pw_gmppt1_off(void) {
}

void solax_pw_gmppt1_high(void) {
}

void solax_pw_gmppt2_off(void) {
}

void solax_pw_gmppt2_high(void) {
}

void solax_pw_mode_self_use(void) {
}

#endif // INVERTER_SOJ