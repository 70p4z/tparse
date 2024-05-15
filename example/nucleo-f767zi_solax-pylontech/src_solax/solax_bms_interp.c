
#include "main.h"
#include "tparse.h"
#include "stddef.h"
#include "stdio.h"

#ifdef MODE_SOLAX_BMS

#define USBVCP USART3

// ensure 5 seconds steady state before making up a decision
#define WORKAROUND_SOLAX_INJECTION_SURGE // avoid too much charged battery to force inverter injecting surplus with clouds' surges
#define GRID_SWITCH_STATE_COUNT 3
#define GRID_CONNECT_SOC    25 // best if equals to the value as the self use end of injection, so that in the end, the inverter is offgrid most of the time
#define GRID_DISCONNECT_SOC 30 // disconnect grid when over or equal


#define SOLAX_MAX_CHARGE_SOC 98 // limit battery wearing

// Solax X1G4 has an offset when respecting battery max charge current (maybe some inside DC bus consumption)
// 50W / 200 = 
// X1G4 for type 0x83 1.8A => 1.0A
// X1G4 for type 0x83 2.0A => 1.2A
// X1G4 for type 0x83 2.0A => 1.6A (when not output EPS power)
// W1G4 for type 0x83 1.0A => 0.1A
//#define SOLAX_BATTERY_CHARGE_OFFSET_DA 4 WITH NO EPS POWERING
#define SOLAX_BATTERY_CHARGE_OFFSET_DA 8

//#define SUPPORT_PYLONTECH_RECONNECT // don't support reconnect to avoid loss of power in EPS, and no conflict with the pylotnech caching stuff
// #define SOLAX_REPLY_0x0100A001_AND_0x1801 # not needed on Solax X1G4

// required with version ARM=1.07+DSP=1.09, 
// not required with ARM=1.28+DSP=1.30 => optimization is far better with this firmware version
//#define HAVE_LOWLIGHT_OPT 

#define EXPIRED(time) (((uint32_t)uwTick - (uint32_t)(time)) < (uint32_t)0x80000000)

//#define BMS_PING
#define BMS_PING_INTERVAL_MS 5000
#define BMS_UART_TIMEOUT 10000
#define BMS_UART_NEXT_INTERVAL 1000

#define BMS_KIND_BLANK 0x50
#define BMS_KIND_BAK 0x51
#define BMS_KIND_REPT 0x52 // ok 4x H48050
#define BMS_KIND_SINOWATT 0x53
#define BMS_KIND_GOT 0x54
#define BMS_KIND_BLANK2 0x55
#define BMS_KIND_TP200 0x81
#define BMS_KIND_TP201 0x82 
#define BMS_KIND_TP202 0x83 // ok 8x H48050
// change depending on the battery configuration if it doesn't work out
#define BMS_KIND BMS_KIND_TP202

#define BMS_RECONNECT_DELAY 20000

#define SOLAX_PW_TIMEOUT 10000 // give few seconds for 400 bytes @ 9600bps
#define SOLAX_PW_NEXT_TIMEOUT 1000 // pocket wifi link update
#define SOLAX_PW_INVALID_RETRY_TIMEOUT 500 // 100ms before retrying in case of an error on the pocketwifi serial response
#define SOLAX_PW_MODE_CHANGE_MIN_INTERVAL 10000 // avoid changing mode constantly
#define PYLONTECH_REPLY_TIMEOUT 1000 // auto reply when pylontech timeoutsn (reset or what not), to avoid the inverter to disconnect


#define SOLAX_PV_POWER_OPT_THRESHOLD_V 150
#define SOLAX_PV_POWER_OPT_THRESHOLD_W 25
#define SOLAX_GRID_EXPORT_OPT_THRESHOLD_W 50

#define SOLAX_MAX_PV_VOLTAGE_V 600 // from user manual

#define SOLAX_DAY_THRESHOLD_V 50 // below 50v is considered NIGHT (with high exposure nights, it has been measured as much)
#define SOLAX_SELF_CONSUMPTION_MPPT_W 40 // observed inverter consumption when MPPT is working
#define SOLAX_SELF_CONSUMPTION_INVERTER_W 40 // observed inverter consumption with only inverter enabled (not system off)

// before 80% of charge of battery, be conservative, and charge first
#define SOLAX_SELFUSE_MIN_BATTERY_SOC (GRID_DISCONNECT_SOC-1) // ensure starting selfuse before disconecting the grid to avoid glitch and mini outtage
#define SOLAX_SELFUSE_MIN_BATTERY_CHARGE_PERCENTAGE_SW_SU 30 /* at least % of the solar power usable must go to the battery */
#define SOLAX_SELFUSE_MIN_BATTERY_CHARGE_PERCENTAGE_SW_FS 20
#define HAVE_SOLAX_SWITCH_MODE

#define SOLAX_PV_CUTOFF_VOLTAGE_V 90
#define SOLAX_PV_CUTOFF_TIMEOUT 2000
#define SOLAX_PV_SWITCHON_TIMEOUT 3000

// not more than a switch into EPS mode per 10 minute, to avoid wearing the physical switch (not a SSR)
#define SOLAX_EPS_MODE_SWITCH_TIMEOUT_MS 600000 // 10min interval 20 years durability for a 1M toggling cycles device
#define SOLAX_EPS_MODE_SWITCH_MIN_SOC 80 // won't switch EPS if not at least charged at 80% (normally)

#define DISPLAY_TIMEOUT 1000
// at least a CAN communication must have taken place within that period
#define TIMEOUT_LAST_ACTIVITY 180000

#define S2LE(buf, off) ((int16_t)((int16_t)((int16_t)((int16_t)(buf)[off+1])<<8l) | (int16_t)((int16_t)(buf)[off]&0xFFl) ))
#define U2LE(buf, off) ((((buf)[off+1]&0xFFu)<<8) | ((buf)[off]&0xFFu) )
#define U4LE(buf, off) ((U2LE(buf, off+2)<<16) | (U2LE(buf, off)&0xFFFFu))
#define U2BE(buf, off) ((((buf)[off]&0xFFu)<<8) | ((buf)[off+1]&0xFFu) )
#define U4BE_ENCODE(buf, off, value) {(buf)[(off)+0] = ((value)>>24)&0xFF;(buf)[(off)+1] = ((value)>>16)&0xFF;(buf)[(off)+2] = ((value)>>8)&0xFF;(buf)[(off)+3] = ((value))&0xFF;}

void Configure_I2C_Slave(void);

// for snprintf to work as expected
void _sbrk(void) {

}

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
  master_log("0x");
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

enum bms_uart_state_e {
  BMS_UART_STATE_IDLE,
  BMS_UART_STATE_WAIT,
  BMS_UART_STATE_WAIT_USER,
  BMS_UART_STATE_WAIT_PWR,
  BMS_UART_STATE_WAIT_INFO,
  BMS_UART_STATE_WAIT_UNIT,
};

enum solax_pw_state_e {
  SOLAX_PW_IDLE,
  SOLAX_PW_SEND,
  SOLAX_PW_REQ_SENT,
  SOLAX_PW_WAIT_NEXT,
  SOLAX_PW_INVALID_NEXT,
};

const uint8_t solax_pw_cmd_change_bitrate[] = { 0xAA, 0x55, 0x07, 0x01, 0x85, 0x8C, 0x01};

const uint8_t solax_pw_cmd_get_stat_0x197[] = { 0xAA, 0x55, 0x07, 0x01, 0x10, 0x17, 0x01};
const uint8_t solax_pw_cmd_get_stat_0x25F[] = { 0xAA, 0x55, 0x07, 0x01, 0x13, 0x1A, 0x01};

const uint8_t solax_pw_cmd_mode_self_use[]= { 0xAA, 0x55, 0x09, 0x09, 0x1C, 0x00, 0x00, 0x2D, 0x01 };
const uint8_t solax_pw_cmd_mode_manual[]= { 0xAA, 0x55, 0x09, 0x09, 0x1C, 0x03, 0x00, 0x30, 0x01 };
// useless // const uint8_t solax_pw_cmd_mode_backup[]= { 0xAA, 0x55, 0x09, 0x09, 0x1C, 0x02, 0x00, 0x30, 0x01 };
      
const uint8_t solax_pw_cfg_manual_stop[]= { 0xAA, 0x55, 0x09, 0x09, 0x24, 0x00, 0x00, 0x35, 0x01 };
const uint8_t solax_pw_cfg_manual_charge[]= { 0xAA, 0x55, 0x09, 0x09, 0x24, 0x01, 0x00, 0x35, 0x01 };
// useless // const uint8_t solax_pw_cfg_manual_discharge[]= { 0xAA, 0x55, 0x09, 0x09, 0x24, 0x02, 0x00, 0x35, 0x01 };

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


#define SOLAX_PW_QUEUE_SIZE 3 // schedule a mode change while reading a status response
struct {
  uint8_t*  cmd;
  uint32_t  cmd_len;
  uint32_t  rep_len;
} solax_pw_queue[SOLAX_PW_QUEUE_SIZE];

void solax_pw_queue_pop(void) {
  // consume the first slot
  memmove(&solax_pw_queue[0], &solax_pw_queue[1], sizeof(solax_pw_queue)-sizeof(solax_pw_queue[0]));
  memset(&solax_pw_queue[2], 0, sizeof(solax_pw_queue[2]));
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
  solax_pw_queue[idx].cmd = (uint8_t*)cmd;
  solax_pw_queue[idx].cmd_len = cmd_len;
  solax_pw_queue[idx].rep_len = rep_len;
}

#define PYLONTECH_CACHE_COUNT 8 // 1871:1878
struct {
  uint32_t used:1;
  uint32_t can_id:31;
  uint8_t can_msg[8];
} pylontech_cached_infos[PYLONTECH_CACHE_COUNT];

void pylontech_cache_clear(void) {
  memset(&pylontech_cached_infos, 0, sizeof(pylontech_cached_infos));
}

// assume message is always 8 bytes long
void pylontech_cache_set(uint32_t _can_id, uint8_t* _can_msg) {
  
  // reuse entry ?
  for (uint8_t i=0; i<PYLONTECH_CACHE_COUNT; i++) {
    if (pylontech_cached_infos[i].used 
      && pylontech_cached_infos[i].can_id == _can_id) {
      memmove(pylontech_cached_infos[i].can_msg, _can_msg, 8);
      pylontech_cached_infos[i].can_id = _can_id;
      pylontech_cached_infos[i].used = 1;
      return;
    }
  }

  // or allocate a new one
  for (uint8_t i=0; i<PYLONTECH_CACHE_COUNT; i++) {
    if (!pylontech_cached_infos[i].used) {
      memmove(pylontech_cached_infos[i].can_msg, _can_msg, 8);
      pylontech_cached_infos[i].can_id = _can_id;
      pylontech_cached_infos[i].used = 1;
      return;
    }
  }
}

const char * const solax_forced_work_mode_str [] = {
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
uint32_t self_use_auto;
uint32_t eps_mode_switch_auto;
uint32_t eps_forced_state;
enum {
  SOLAX_FORCED_WORK_MODE_NONE,
  SOLAX_FORCED_WORK_MODE_SELF_USE,
  SOLAX_FORCED_WORK_MODE_BACKUP,
  SOLAX_FORCED_WORK_MODE_MANUAL_STOP,
  SOLAX_FORCED_WORK_MODE_MANUAL_CHARGE,
  SOLAX_FORCED_WORK_MODE_MANUAL_DISCHARGE,
} solax_forced_work_mode;
struct {
  uint16_t pv1_voltage;
  uint16_t pv2_voltage;
  uint16_t pv1_current;
  uint16_t pv2_current;
  uint16_t pv1_wattage;
  uint16_t pv2_wattage;
  int16_t bat_wattage;
  #define INVERTER_STATUS_WAITING 0
  #define INVERTER_STATUS_CHECKING 1
  #define INVERTER_STATUS_NORMAL 2
  #define INVERTER_STATUS_FAULT 3
  #define INVERTER_STATUS_ERROR 4
  #define INVERTER_STATUS_UPDATE 5
  #define INVERTER_STATUS_EPS_WAIT 6
  #define INVERTER_STATUS_EPS 7
  #define INVERTER_STATUS_SELFTEST 8
  #define INVERTER_STATUS_IDLE 9
  #define INVERTER_STATUS_STANDBY 10
  uint8_t status;
  uint8_t status_count; // account for number of times the same state has shown
  uint8_t powered_on; // inverter_status != standby
  uint8_t work_mode;
  uint16_t bat_SoC;
  int16_t bat_temp;
  int16_t grid_wattage;
  int16_t grid_meter_ct;
  int16_t output_va;
  int16_t eps_current;
  int16_t eps_voltage;
  int16_t eps_power;
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t seconds;
  uint8_t pv1_switch_on;
  uint8_t pv2_switch_on;
  uint8_t grid_connect_soc;
  uint8_t grid_disconnect_soc;
} solax;

#define PYLONTECH_MAX_BMUS 20
struct {
  uint16_t voltage;
  uint32_t precise_voltage;
  int16_t current;
  int32_t precise_current;
  uint8_t soc;
  uint8_t soc_mWh;
  uint32_t precise_mAh;
  uint32_t precise_mAh_ts; // to compute approx current drive, when 0 is reported
  uint32_t precise_mWh;
  int32_t wattage;
  int32_t precise_wattage;
  int16_t max_charge;
  int16_t max_discharge;
  uint16_t packs;
  uint8_t max_charge_soc;
  uint8_t contactor_on;
  uint8_t charge_request;
  uint16_t cycles;
  uint8_t fix2_31;
  uint32_t total_capacity_mAh;
  uint8_t bmu_idx;
  uint8_t bmu_idx_tmp;
  struct {
    uint8_t soc;
    uint8_t soc_mWh;
    uint16_t vlow;
    uint16_t vhigh;
    uint8_t pcba[32+1];
  } bmu[PYLONTECH_MAX_BMUS];
} pylontech;

void pv1_switch(uint32_t state) {
  // anode green led 
  // cathode of the optocoupler 1 = off => mosfet defaulty closed, 0 = on => mosfet open
  gpio_set(1, 0, state); // PB0 (green led)
#ifndef BOARD_DEV
  gpio_set(3, 14, state); // PD14
#endif // BOARD_DEV
  solax.pv1_switch_on = state; 
}

void pv2_switch(uint32_t state) {
  // anode blue led 
  // cathode of the optocoupler 1 = off => mosfet defaulty closed, 0 = on => mosfet open
  gpio_set(1, 7, state); // PB7 (blue led)
#ifndef BOARD_DEV
  gpio_set(3, 15, state); // PD14
#endif // BOARD_DEV
  solax.pv2_switch_on = state;
}

/*
GPIO=1 + EPS=230V -> (TRIAC=CLOSED) + EPS_relay(NO=COM) -> GRID_relay(OPENED) -> INV=|=GRID
GPIO=* + EPS=0V   -> (TRIAC=*) + EPS_relay(NC=COM) -> GRID_relay(CLOSED) -> INV===GRID
*/
void eps_mode_switch(uint32_t eps_mode_requested) {
  eps_forced_state = eps_mode_requested;
  // switch the physical switch to force EPS mode (disconnect from the grid)
  // gpio = 1 => phototriac closed => contactor coil powered => grid contact are severed (Normally Closed)
  gpio_set(1, 14, eps_mode_requested); // PB14 (red led)
#ifndef BOARD_DEV
  gpio_set(5, 12, eps_mode_requested); // PF12
#endif // BOARD_DEV
}

void interp(void) {
  uint32_t forward;
  enum bms_uart_state_e bms_uart_state = BMS_UART_STATE_IDLE;
  uint32_t bms_uart_next = 0;
  uint32_t bms_uart_timeout = 0;
  enum solax_pw_state_e solax_pw_state = SOLAX_PW_IDLE;
#ifdef SUPPORT_PYLONTECH_RECONNECT
  uint32_t bms_reconnect_at;
#endif // SUPPORT_PYLONTECH_RECONNECT  
  uint32_t solax_pw_timeout=0;
#ifdef BMS_PING
  uint32_t bms_ping_timeout;
#endif // BMS_PING
  size_t len;
  uint32_t cid;
  size_t cid_bitlen;
#ifdef SOLAX_REPLY_0x0100A001_AND_0x1801
  uint32_t enable_battery = 0;
#endif // SOLAX_REPLY_0x0100A001_AND_0x1801
  //uint32_t wait_bms_info = 1;
  //uint8_t bms_info[8];
#ifdef USART5_HUMAN_READABLE_SUMMARY_LOG
  uint32_t timeout_next_display = 0;
#endif // USART5_HUMAN_READABLE_SUMMARY_LOG
  uint32_t solax_pw_mode_change_ready;
  uint32_t pylontech_timeout = 0;
  uint32_t last_INV_CAN_activity_timeout = 0;
  uint32_t last_BMS_CAN_activity_timeout = 0;
  uint32_t timeout_pv1_switch_on=0;
  uint32_t timeout_pv1_switch_off=0;
  uint32_t timeout_pv2_switch_on=0;
  uint32_t timeout_pv2_switch_off=0;
  uint32_t eps_mode_switch_timeout=0;

  memset(&solax, 0, sizeof(solax));
  memset(&pylontech, 0, sizeof(pylontech));
  // init the queue
  memset(solax_pw_queue, 0, sizeof(solax_pw_queue));
  pylontech_cache_clear();

  // automatic state switching and eps disconnect mode by default
  self_use_auto = 1;
  eps_mode_switch_auto = 1;
  eps_forced_state = 0;
  solax.grid_connect_soc = GRID_CONNECT_SOC;
  solax.grid_disconnect_soc = GRID_DISCONNECT_SOC;
  pylontech.max_charge_soc = SOLAX_MAX_CHARGE_SOC;

  // use BARE HSI (16MHz)
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

  LL_SetSystemCoreClock(16000000);
  SysTick_Config(16000000/1000);
  
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOE);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);

  pv1_switch(1);
  pv2_switch(1);

  // USART used for USBVCP communication
  Configure_USBVCP(USART_BAUDRATE_USBVCP);
  // Usart for BMS communication (UART6 PC6-TX PC7-RX)
  Configure_UARTBMS(115200);

  Configure_CAN1(500000);
  Configure_CAN3(500000);

  //solax_pw_queue_push(solax_pw_cmd_change_bitrate, sizeof(solax_pw_cmd_change_bitrate), 7);
  // switch bitrate on the solax, no reply expected
  Configure_UARTPW(9600);
  uart_select_intf(UARTPW);
  uart_send_mem(solax_pw_cmd_change_bitrate, sizeof(solax_pw_cmd_change_bitrate));  
  // wait until bitrate is taken into account
  LL_mDelay(250);
  Configure_UARTPW(115200);

  // ensure starting with SELF USE mode
  solax_pw_queue_push(solax_pw_cmd_mode_self_use, sizeof(solax_pw_cmd_mode_self_use), 7);
  solax_forced_work_mode = SOLAX_FORCED_WORK_MODE_SELF_USE;

  Configure_I2C_Slave();

#ifdef MODE_FAKE_SOLAX
  while (1) {
    can_solax_tx_log(0x1871, CAN_ID_EXTENDED_LEN, (uint8_t*)"\x01\x00\x01\x00\x00\x00\x00\x00", 8);
    LL_mDelay(BMS_PING_INTERVAL_MS);
  }
#endif // MODE_FAKE_SOLAX

  tparse_ctx_t tp_solax_pw;
  tparse_init(&tp_solax_pw, uart_pw_buffer, sizeof(uart_pw_buffer), "");
  tparse_ctx_t tp_bms;
  tparse_init(&tp_bms, uart_bms_buffer, sizeof(uart_bms_buffer), " %\n"); // add % as delim to parse percentage easier
  tparse_ctx_t tp_master;
  tparse_init(&tp_master, uart_usbvcp_buffer, sizeof(uart_usbvcp_buffer), "\n");

  master_log("Reset\n");

  // sent ping to the BMS to wake it up at reset moment
  can_bms_tx_log(0x1871, CAN_ID_EXTENDED_LEN, (uint8_t*)"\x01\x00\x01\x00\x00\x00\x00\x00", 8);

  solax_pw_mode_change_ready = uwTick + SOLAX_PW_MODE_CHANGE_MIN_INTERVAL;
#ifdef BMS_PING
  bms_ping_timeout = uwTick + BMS_PING_INTERVAL_MS;
#endif // BMS_PING
#ifdef SUPPORT_PYLONTECH_RECONNECT  
  bms_reconnect_at = uwTick;
#endif // SUPPORT_PYLONTECH_RECONNECT
  last_INV_CAN_activity_timeout = uwTick + TIMEOUT_LAST_ACTIVITY;
  last_BMS_CAN_activity_timeout = uwTick + TIMEOUT_LAST_ACTIVITY;
  pylontech_timeout = uwTick + PYLONTECH_REPLY_TIMEOUT;
#ifdef USART5_HUMAN_READABLE_SUMMARY_LOG
  timeout_next_display = uwTick;
#endif // USART5_HUMAN_READABLE_SUMMARY_LOG

  // startup the watchdog
  /* Enable the peripheral clock of DBG register (uncomment for debug purpose) */
  /* ------------------------------------------------------------------------- */
  /*  LL_DBGMCU_APB1_GRP1_FreezePeriph(LL_DBGMCU_APB1_GRP1_IWDG_STOP); */
  
  /* Enable the peripheral clock IWDG */
  /* -------------------------------- */
  LL_RCC_LSI_Enable();
  while (LL_RCC_LSI_IsReady() != 1)
  {
  }

  /* Configure the IWDG with window option disabled */
  /* ------------------------------------------------------- */
  /* (1) Enable the IWDG by writing 0x0000 CCCC in the IWDG_KR register */
  /* (2) Enable register access by writing 0x0000 5555 in the IWDG_KR register */
  /* (3) Write the IWDG prescaler by programming IWDG_PR from 0 to 7 - LL_IWDG_PRESCALER_4 (0) is lowest divider*/
  /* (4) Write the reload register (IWDG_RLR) */
  /* (5) Wait for the registers to be updated (IWDG_SR = 0x0000 0000) */
  /* (6) Refresh the counter value with IWDG_RLR (IWDG_KR = 0x0000 AAAA) */
  LL_IWDG_Enable(IWDG);                             /* (1) */
  LL_IWDG_EnableWriteAccess(IWDG);                  /* (2) */
  LL_IWDG_SetPrescaler(IWDG, LL_IWDG_PRESCALER_4);  /* (3) */
  LL_IWDG_SetReloadCounter(IWDG, 0xFEE);            /* (4) */
  while (LL_IWDG_IsReady(IWDG) != 1)                /* (5) */
  {
  }
  LL_IWDG_ReloadCounter(IWDG);                      /* (6) */  

  while (1) {

    /* Refresh IWDG down-counter to default value */
    LL_IWDG_ReloadCounter(IWDG);
    // reset in case of CAN activity timeout
    if (EXPIRED(last_INV_CAN_activity_timeout)
      || EXPIRED(last_BMS_CAN_activity_timeout)) {
      master_log("INV or CAN activity timeout\n");
      NVIC_SystemReset();
    }

    // check for messages from the inverter
    if (can_fifo_avail(CAN1)) {
      last_INV_CAN_activity_timeout = uwTick + TIMEOUT_LAST_ACTIVITY; // we've received something
      len = can_fifo_rx(CAN1, &cid, &cid_bitlen, tmp, sizeof(tmp));
      if (cid == 0x1871) {
        // clear screen before starting decoding
        // annoying when trying to log into a single window // master_log("\x1b[2J");
      }
      master_log_can("inv >>>     | ", cid, cid_bitlen, tmp, len);
      // other request from the inverter are discarded
      if (cid_bitlen && cid == 0x1871) {
        forward = 0;
        switch(tmp[0]) {
          case 1:
            {
              uint8_t b[4];
              U4BE_ENCODE(b, 0, uwTick);
              master_log("            | Timestamp: ");
              master_log_hex(b, 4);
              master_log("\n");
            }
            solax.powered_on = tmp[2];
            // get data
            forward = 1;
#ifdef SOLAX_REPLY_0x0100A001_AND_0x1801
            if (enable_battery==1) {
              can_solax_tx_log(0x0100A001, CAN_ID_EXTENDED_LEN, NULL, 0);
              enable_battery = 2;
            }
            can_solax_tx_log(0x1801, CAN_ID_EXTENDED_LEN, (uint8_t*)"\x01\x00\x01\x00\x00\x00\x00\x00", 8);
#endif // SOLAX_REPLY_0x0100A001_AND_0x1801
            break;
#ifdef SUPPORT_PYLONTECH_RECONNECT
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
#endif // SUPPORT_PYLONTECH_RECONNECT
        }

        // only forward when the bms is allowed (not timing out for juice cut)
        if (forward 
#ifdef SUPPORT_PYLONTECH_RECONNECT
          && EXPIRED(bms_reconnect_at)
#endif // SUPPORT_PYLONTECH_RECONNECT
          ) {
#ifdef SOLAX_REPLY_0x0100A001_AND_0x1801
          // reset to reenable battery
          enable_battery = 0;
#endif // SOLAX_REPLY_0x0100A001_AND_0x1801
          if (pylontech_timeout == 0) {
            // only schedule timeout when no previous timeout scheduled
            pylontech_timeout = uwTick + PYLONTECH_REPLY_TIMEOUT;
          }
#ifdef SUPPORT_PYLONTECH_RECONNECT
          bms_reconnect_at = uwTick; // make sure to avoid overflow when no reconnection request for a while
#endif // SUPPORT_PYLONTECH_RECONNECT
          can_bms_tx_log(cid, cid_bitlen, tmp, len);
        }
      }
    }

#ifdef BMS_PING
    // check for messages from the bms
     // time for a ping
    if (EXPIRED(bms_ping_timeout)) {
      can_bms_tx_log(0x1871, CAN_ID_EXTENDED_LEN, (uint8_t*)"\x01\x00\x01\x00\x00\x00\x00\x00", 8);
      //can_solax_tx_log(0x1801, CAN_ID_EXTENDED_LEN, (uint8_t*)"\x01\x00\x01\x00\x00\x00\x00\x00", 8);
      bms_ping_timeout = uwTick + BMS_PING_INTERVAL_MS;
    }
#endif // BMS_PING

    // pylontech timeout, serve the cached infos
    if (pylontech_timeout && EXPIRED(pylontech_timeout)) {
      master_log("bms can timeout\n");
      // avoid retriggering timeout
      pylontech_timeout = 0;
      // reply with cache infos
      for (uint8_t i=0; i< PYLONTECH_CACHE_COUNT; i++) {
        if (pylontech_cached_infos[i].used) {
          can_solax_tx_log(pylontech_cached_infos[i].can_id, CAN_ID_EXTENDED_LEN, 
                           pylontech_cached_infos[i].can_msg, 8);
        }
      }
    }

    if (can_fifo_avail(CAN3)) {
      len = can_fifo_rx(CAN3, &cid, &cid_bitlen, tmp, sizeof(tmp));
      master_log_can("bms >>>     | ", cid, cid_bitlen, tmp, len);
      tmp[16]=0; // EOS for human readable log
      last_BMS_CAN_activity_timeout = uwTick + TIMEOUT_LAST_ACTIVITY;
      pylontech_timeout = 0;
      forward = 0;
      switch(cid) {
        case 0x1873:
          // may reinterpret SoC depending on battery voltage instead of relying on BMS
          pylontech.voltage = U2LE(tmp,0);
          pylontech.current = S2LE(tmp,2);
          pylontech.soc = U2LE(tmp,4);
          pylontech.wattage = ((int32_t)pylontech.voltage)*((int32_t)pylontech.current)/((int32_t)100); // unit 0.1V x 0.1A
          snprintf((char*)tmp+16, sizeof(tmp)-16, "            | Vbat=%d.%dV\tIbat=%d.%dA\tWbat=%ldW\tSoC=%d\tEbat=%d.%dkWh\n", 
                   pylontech.voltage/10,pylontech.voltage%10,
                   pylontech.current/10,pylontech.current%10,
                   pylontech.wattage,
                   pylontech.soc,
                   U2LE(tmp,6)/100,U2LE(tmp,6)%100);
          if (batt_forced_soc >= 0) {
            master_log("force batt soc\n");
            tmp[4] = batt_forced_soc&0xFF;
            tmp[5] = 0;
          }
          if (batt_forced_charge >= 0) {
            tmp[4] = MIN(pylontech.soc, 75); // ensure charging when forcing charge
          }
          forward = 1;
          break;
        case 0x1872: {

          int16_t maxch = S2LE(tmp, 4);
          int16_t maxdis = S2LE(tmp, 6);

          // detect 2^31 bug (could affine with SysError detection on FlagsBMS = 0x0002)
          pylontech.fix2_31 = maxdis <= 0 && maxch <= 0 && pylontech.soc >= 10;

          snprintf((char*)tmp+16, sizeof(tmp)-16, "            | Vbatmax=%d.%dV\tVbatmin=%d.%dV\tIcmax=%d.%dA\tIdmax=%d.%dA%s\n", 
                   S2LE(tmp, 0)/10,S2LE(tmp, 0)%10,
                   S2LE(tmp, 2)/10,S2LE(tmp, 2)%10,
                   maxch/10,maxch%10,
                   maxdis/10,maxdis%10,
                   (pylontech.fix2_31)?" 2^31fix":"");

          // when soc is below 10%, then the discharge may be 0
          // if soc > 10%, then max discharge can never be 0, it's 
          // the manifestation of the 2^31 counter overflow bug in the SC0500
          // in that case, we won't tranmsit and keep the previous value sent to the inverter
          // the bug has a periodicity of 1h and a duration of a few seconds.
          // it takes to circumvent the bug on the coil level too to make this patch effective.
          if (pylontech.fix2_31) {
            break;
          }

          // ensure respecting maxcharge, taking into account the inner charge cap offset
          if (maxch) {
            maxch += SOLAX_BATTERY_CHARGE_OFFSET_DA;
            tmp[4] = maxch&0xFF;
            tmp[5] = (maxch>>8)&0xFF;
          }

          pylontech.max_charge = maxch;
          pylontech.max_discharge = maxdis;
#if 0
          // cover the inverter self consumption which is substracted from the BAT DC charge
          // this bug is kind of weird if you ask me
          if (pylontech.max_charge && pylontech.soc != 100) {
            // NOTE: when charging from solar (grid charge is cheating), then MPPT is ON
            uint32_t self_consumption_current_dA = (SOLAX_SELF_CONSUMPTION_MPPT_W
                                                    + SOLAX_SELF_CONSUMPTION_INVERTER_W)*100/*cW*/ 
                                                   / pylontech.voltage/*dV*/;
            pylontech.max_charge+=self_consumption_current_dA;
            tmp[4] = (pylontech.max_charge)&0xFF;
            tmp[5] = ((pylontech.max_charge)>>8)&0xFF;
          }
#endif
          // disallow discharge when battery is too low
          if (pylontech.soc <= 10) {
            master_log("soc < 10\n");
            // disable discharge
            tmp[6] = 0;
            tmp[7] = 0;
          }
          // max charge reached?
          if (pylontech.soc >= pylontech.max_charge_soc) {
            tmp[4] = 0;
            tmp[5] = 0;
          }
#if 1
          if (batt_forced_charge >= 0) {
            master_log("force batt charge\n");
            tmp[4] = batt_forced_charge&0xFF;
            tmp[5] = (batt_forced_charge>>8)&0xFF;
          }
#endif
          forward = 1;
          break;
        }
        case 0x1871:
          //pylontech_cache_clear();
          __attribute__((fallthrough));
        case 0x1878:
          snprintf((char*)tmp+16, sizeof(tmp)-16, "            | Format=%d\tUnk=%d\tFlagsBMS<<16=%04x\tCapacity=%dWh\n", 
                   tmp[0],
                   tmp[1],
                   U2LE(tmp, 2),
                   U4LE(tmp, 4));
          // wipe flags on 2^31 fix
          if (pylontech.fix2_31) {
            tmp[2] = tmp[3] = 0;
          }
          forward = 1;
          break;

        case 0x1875:
          pylontech.packs = U2LE(tmp,2); 
          pylontech.contactor_on = tmp[4];
          pylontech.charge_request = tmp[5];
          pylontech.cycles = U2LE(tmp, 6);
          // TODO: ensure contactor is set to 1 (tmp[4] = 1)
          snprintf((char*)tmp+16, sizeof(tmp)-16, "            | Tbms=%d.%d°C\tBatCount=%d\tContact=%d\tChargeReq=%d\tCycles=%d\n", 
                   S2LE(tmp, 0)/10,S2LE(tmp, 0)%10,
                   pylontech.packs,
                   pylontech.contactor_on,
                   pylontech.charge_request,
                   pylontech.cycles);
          forward = 1;
          break;

        case 0x1877:
          snprintf((char*)tmp+16, sizeof(tmp)-16, "            | FlagsBMS=%04x\n", 
                   U2LE(tmp, 0));
          // wipe BMS flags upon 2^31 fix
          if (pylontech.fix2_31) {
            tmp[0] = tmp[1] = 0;
            //memset(tmp, 0, 4); 
          }
          tmp[4] = BMS_KIND;
          tmp[5] = tmp[6] = tmp[7] = 0; // wipe versions
          // override message to tell the inverter of the battery configuration
          //memmove(tmp, "\x00\x00\x00\x00\x52\x00\x00\x00", 8); // OK 2H48050, OK 4 H48050p, OK 6 H48050, NOK 3/5/7/8 => seen as T58
          //memmove(tmp, "\x00\x00\x00\x00\x82\x00\x00\x00", 8); // NOK 8 H48050 // brand TP201
          //memmove(tmp, "\x00\x00\x00\x00\x83\x00\x00\x00", 8); // OK 8 H48050 // brand TP202
          len = 8;
          forward = 1;
#ifdef SOLAX_REPLY_0x0100A001_AND_0x1801
          // we've described the battery to the inverter, enable it now
          enable_battery = 1;
#endif // SOLAX_REPLY_0x0100A001_AND_0x1801
          break;

        // discarded
        case 0x1874:
          snprintf((char*)tmp+16, sizeof(tmp)-16, "            | Tcellmax=%d.%d°C\tTcellmin=%d.%d°C\tVcellmax=%d.%dV\tVcellmin=%d.%dV\n", 
                   S2LE(tmp, 0)/10,S2LE(tmp, 0)%10,
                   S2LE(tmp, 2)/10,S2LE(tmp, 2)%10,
                   S2LE(tmp, 4)/10,S2LE(tmp, 4)%10,
                   S2LE(tmp, 6)/10,S2LE(tmp, 6)%10);
          break;
        case 0x1876:
          snprintf((char*)tmp+16, sizeof(tmp)-16, "            | dlim=%d\tIdlim=%d.%dA\tclim=%d\tIclim=%d.%dA\n", 
                   U2LE(tmp, 0),
                   S2LE(tmp, 2)/10,S2LE(tmp, 2)%10,
                   U2LE(tmp, 4),
                   S2LE(tmp, 6)/10,S2LE(tmp, 6)%10);
          break;
      }
      if (forward) {
        pylontech_cache_set(cid, tmp);
        can_solax_tx_log(cid, cid_bitlen, tmp, len);
      }
      // log decoded message content
      master_log((char*)tmp+16);
    }

    tparse_finger(&tp_master, sizeof(uart_usbvcp_buffer) - DMA_Stream_USBVCP->NDTR);
    // update data from the bms usart link
    tparse_finger(&tp_bms, sizeof(uart_bms_buffer) - DMA_Stream_BMS->NDTR);
    /*
    pylon>pwr
    pwr
    @
    AverageTempr   : 22291       
    DC Volt        : 402111      
    Bat Volt       : 401777      

    Volt   Curr   Tempr  BTlow  BThigh BVlow  BVhigh UTlow  UThigh UVlow  UVhigh Base.St  Volt.St  Curr.St  Temp.St  CoulombAH                CoulombWH              Time                 B.V.St   B.T.St   U.V.St   U.T.St   Err Code
    402855 613    34000  21000  23000  3354   3360   26000  27000  50323  50390  Charge   Normal   Normal   Normal    84%          42041 mAH  83%           15960 WH 2000-01-03 23:52:40  Normal   Normal   Normal   Normal   0x0
    Command completed successfully
    $$

    #OTO: field indexes
    0      1      2      3      4      5      6      7      8      9      10     11       12       13       14        15           16    17   18            19
    */
    switch(bms_uart_state) {
      case BMS_UART_STATE_IDLE:
        if (tparse_has_line(&tp_master)) {
          size_t read = tparse_peek_line(&tp_master, (char*)tmp, sizeof(tmp));
          master_log("UARTBMSUSER >> ");
          master_log_mem(tmp, read);
          uart_select_intf(USART6);
          uart_send_mem(tmp, read);
          bms_uart_state = BMS_UART_STATE_WAIT_USER;
          tparse_token_u32(&tp_bms);
          tparse_discard_line(&tp_master);
        }
        else {
          // wait until transmit moment is reached
          if (bms_uart_next && !EXPIRED(bms_uart_next)) {
            break;
          }
          tparse_discard(&tp_bms);
          master_log("UARTBMS >> pwr\n");
          // send request to the bms
          uart_select_intf(USART6);
          uart_send_mem("pwr\n",4);
          bms_uart_state = BMS_UART_STATE_WAIT_PWR;
          bms_uart_next = uwTick + BMS_UART_NEXT_INTERVAL;
        }
        bms_uart_timeout = uwTick + BMS_UART_TIMEOUT;
        break;
      case BMS_UART_STATE_WAIT_PWR:
      case BMS_UART_STATE_WAIT_UNIT:
      case BMS_UART_STATE_WAIT_INFO:
      case BMS_UART_STATE_WAIT_USER:
        // line received?
        if (tparse_has_line(&tp_bms)) {
          size_t read = tparse_peek_line(&tp_bms, (char*)tmp, sizeof(tmp));
          master_log("UARTBMS << ");
          master_log_mem(tmp,read);
          switch(bms_uart_state) {
          case BMS_UART_STATE_WAIT_PWR: {
            uint32_t val = tparse_token_u32(&tp_bms); // Volt
            // if it's the line starting with integer and not a text line
            if (val != -1UL) {
              pylontech.precise_voltage = (int32_t)val;
              int32_t valcurr = (int32_t)tparse_token_i32(&tp_bms); // Curr

              #ifdef WIP_PRECISE_CURRENT_FROM_CAPACITY
              // keep non 0 values (captured using mAh diff)
              if (valcurr != 0) 
              #endif // WIP_PRECISE_CURRENT_FROM_CAPACITY
              {
                pylontech.precise_current = valcurr;
              }
              tparse_token(&tp_bms, (char*)&val, 4); // tempr
              tparse_token(&tp_bms, (char*)&val, 4); // btlow
              tparse_token(&tp_bms, (char*)&val, 4); // bthigh
              tparse_token(&tp_bms, (char*)&val, 4); // bvlow
              tparse_token(&tp_bms, (char*)&val, 4); // bvhigh
              tparse_token(&tp_bms, (char*)&val, 4); // utlow
              tparse_token(&tp_bms, (char*)&val, 4); // uthigh
              tparse_token(&tp_bms, (char*)&val, 4); // uvlow
              tparse_token(&tp_bms, (char*)&val, 4); // uvhigh
              tparse_token(&tp_bms, (char*)&val, 4); // base.st
              tparse_token(&tp_bms, (char*)&val, 4); // volt.st
              tparse_token(&tp_bms, (char*)&val, 4); // curr.st
              tparse_token(&tp_bms, (char*)&val, 4); // temp.st
              tparse_token(&tp_bms, (char*)&val, 4); // coulomb %
              uint32_t mah = tparse_token_u32(&tp_bms); // coulomb mAh
              if (mah != -1UL) {
                // only update value when different from previous, to better compute mean consumption
                // report current drain when 0 is notified
                if (mah != pylontech.precise_mAh) {
                  #ifdef WIP_PRECISE_CURRENT_FROM_CAPACITY
                  if (valcurr == 0) {
                    int32_t precise_current = ((int32_t)mah - (int32_t)pylontech.precise_mAh) * ((int32_t)3600000) 
                                                / ((int32_t)uwTick - (int32_t)pylontech.precise_mAh_ts);
                    // anti oups due to timestamp counter rollover
                    // over +/-100mA, the current is accounted correctly in the CAN frame
                    if (precise_current < 100 && precise_current > -100) {
                      pylontech.precise_current = precise_current;
                    }
                  }
                  #endif // WIP_PRECISE_CURRENT_FROM_CAPACITY
                  // only uptade timestamp when value changes
                  pylontech.precise_mAh_ts = uwTick;
                  pylontech.precise_mAh = mah;
                }
              }
              tparse_token(&tp_bms, (char*)&val, 4); // mAh
              val = tparse_token_u32(&tp_bms); // coulomb % mWh
              if (val != -1UL) {
                pylontech.soc_mWh = val;
              }
              val = tparse_token_u32(&tp_bms); // coulomb mWh
              if (val != -1UL) {
                pylontech.precise_mWh = val;
              }
              // conpute precise current after correction if 0 reported
              pylontech.precise_wattage = ((int32_t)pylontech.precise_current*((int32_t)pylontech.precise_voltage/(int32_t)100))/(int32_t)10000;
              snprintf((char*)tmp, sizeof(tmp), "  voltage: %ld\n  current: %ld\n  wattage: %ld\n capacity: %ld\n", pylontech.precise_voltage, pylontech.precise_current, pylontech.precise_wattage, pylontech.precise_mAh);
              master_log((char*)tmp);
              // invariant check
              if ((pylontech.precise_current < 0 && pylontech.precise_wattage > 0 )
                || (pylontech.precise_current > 0 && pylontech.precise_wattage < 0 )) {
                master_log("error: invalid precise wattage computation\n");
                pylontech.precise_wattage=0;
              }
              tparse_discard_line(&tp_bms);
            }
            // end?
            else if (read >= 3 && strstr((const char *)tmp, "\r$$") == (const char *)tmp) {
              tparse_discard(&tp_bms);
              master_log("UARTBMS >> info\n");
              // send request to the bms
              uart_select_intf(USART6);
              uart_send_mem("info\n",5);
              pylontech.bmu_idx_tmp=0;
              bms_uart_state = BMS_UART_STATE_WAIT_INFO; 
              bms_uart_timeout = uwTick + BMS_UART_TIMEOUT;
            }
            else {
              tparse_token_u32(&tp_bms);
              tparse_discard_line(&tp_bms);
            }
            break;
          } 
          case BMS_UART_STATE_WAIT_INFO: {
            /*
            pylon>info
            @
            Device address      : 0
            Manufacturer        : Pylon
            Device name         : CMU_A
            Board version       : TISP01V10R02_1
            Hard  version       : V10R9C2
            Main Soft version   : B52.28.0
            Soft  version       : V5.2
            Boot  version       : V1.4
            Comm version        : V2.0
            Release Date        : 21-06-21

            Barcode             :                 
            PCBA Barcode        : H10****************226          
            Module Barcode      : PP***********050                
            PowerSupply Barcode : H20****************058          

            Device Test Time    : 2021-07-10 10:27:10

            Specification       : 384V/50AH
            Cell Number         : 120
            Max Dischg Curr     : -40000mA
            Max Charge Curr     : 40000mA
            Shut Circuit        : Yes  
            Relay Feedback      : Yes  
            New Board           : Yes  

            BMU 7 Barcode   
            Module:   : HP***********402                
            PCBA:     : H200***************198         

            BMU 6 Barcode   
            Module:   : HP***********402                
            PCBA:     : H200***************198          
            ...
            */
            if (read >= 13 && strstr((const char *)tmp, "\rPCBA:     : ") == (const char *)tmp) {
              tparse_token(&tp_bms, (char*)tmp, sizeof(tmp)); // PCBA:
              tparse_token(&tp_bms, (char*)tmp, sizeof(tmp)); // :
              tparse_token(&tp_bms, 
                           (char*)pylontech.bmu[pylontech.bmu_idx_tmp].pcba, 
                           sizeof(pylontech.bmu[pylontech.bmu_idx_tmp].pcba)); // PCBA value
              pylontech.bmu_idx_tmp++;
              tparse_discard_line(&tp_bms);
            }
            // end?
            else if (read >= 3 && strstr((const char *)tmp, "\r$$") == (const char *)tmp) {
              tparse_discard(&tp_bms);
              master_log("UARTBMS >> unit\n");
              // send request to the bms
              uart_select_intf(USART6);
              uart_send_mem("unit\n",5);
              pylontech.bmu_idx = pylontech.bmu_idx_tmp; // validate new count, so that i2c are not desynch
              bms_uart_state = BMS_UART_STATE_WAIT_UNIT; 
              bms_uart_timeout = uwTick + BMS_UART_TIMEOUT;
            }
            else {
              tparse_token_u32(&tp_bms);
              tparse_discard_line(&tp_bms);
            }
            break;
          }
          case BMS_UART_STATE_WAIT_UNIT: {
            // unit info lines start with index value
            uint32_t idx = tparse_token_u32(&tp_bms)-1; // index
            if (idx >= 0 && idx < pylontech.bmu_idx && idx < PYLONTECH_MAX_BMUS) {
              /*
              Index  Volt   Curr   Tempr  BTlow  BThigh BVlow  BVhigh Base.St  Volt.St  Temp.St  CoulombAH                CoulombWH               Time               
              1      50682  753    28000  25000  25000  3378   3380    Charge   Normal   Normal    93%          46477 mAH  93%            2238 WH 2000-11-27 01:37:22 
              */
              tparse_token_u32(&tp_bms); // volt
              tparse_token_u32(&tp_bms); // current
              tparse_token_u32(&tp_bms); // tempr
              tparse_token_u32(&tp_bms); // btlow
              tparse_token_u32(&tp_bms); // bthigh
              pylontech.bmu[idx].vlow = tparse_token_u32(&tp_bms); // bvlow
              pylontech.bmu[idx].vhigh = tparse_token_u32(&tp_bms); // bvhigh
              tparse_token_u32(&tp_bms); // base.st
              tparse_token_u32(&tp_bms); // volt.st
              tparse_token_u32(&tp_bms); // temp.st
              pylontech.bmu[idx].soc = tparse_token_u32(&tp_bms);
              tparse_token_u32(&tp_bms); // mAh
              tparse_token_u32(&tp_bms); // 'mAh'
              pylontech.bmu[idx].soc_mWh = tparse_token_u32(&tp_bms); // CoulombWH
              tparse_discard_line(&tp_bms);
            }
            // end?
            else if (read >= 3 && strstr((const char *)tmp, "\r$$") == (const char *)tmp) {
              tparse_discard(&tp_bms);
              master_log("UARTBMS end\n");
              bms_uart_state = BMS_UART_STATE_IDLE; 
              bms_uart_timeout = 0;
            }
            else {
              tparse_token_u32(&tp_bms);
              tparse_discard_line(&tp_bms);
            }
            break;
          }
          default:
            tparse_token_u32(&tp_bms); // consume one token to ensure consumption is complete
            if (read >= 3 && strstr((const char *)tmp, "\r$$") == (const char *)tmp) {
              // last line of the command, THANKS pylontech for that delimiter
              bms_uart_next = 0;
              bms_uart_state = BMS_UART_STATE_IDLE;
            }
            tparse_discard_line(&tp_bms);
            break;
          }
        }
        // timeout waiting for the command, reset the state and prepare a new command
        if ( bms_uart_timeout && EXPIRED(bms_uart_timeout)) {
          bms_uart_timeout = 0;
          bms_uart_next = 0; // immediate next command
          bms_uart_state = BMS_UART_STATE_IDLE; // send PWR again
          pylontech.precise_wattage = 0; // avoid relaying outdated data
        }
        break;
    }

    if (timeout_pv1_switch_on && EXPIRED(timeout_pv1_switch_on)) {
      pv1_switch(1);
      master_log("pv1 switch on\n");
      timeout_pv1_switch_on = 0;
      timeout_pv1_switch_off = uwTick + SOLAX_PV_SWITCHON_TIMEOUT;
    }

    if (timeout_pv2_switch_on && EXPIRED(timeout_pv2_switch_on)) {
      pv2_switch(1); 
      master_log("pv2 switch on\n");
      timeout_pv2_switch_on = 0;
      timeout_pv2_switch_off = uwTick + SOLAX_PV_SWITCHON_TIMEOUT;
    }

    // handle solax PocketWifi port to get the solax status
    tparse_finger(&tp_solax_pw, sizeof(uart_pw_buffer) - DMA_Stream_PW->NDTR);
    switch(solax_pw_state) {
      case SOLAX_PW_IDLE:
        // is no command scheduled for sending?
        if (solax_pw_queue_free() == SOLAX_PW_QUEUE_SIZE) {
          // solax_pw_queue_push(solax_pw_cmd_get_stat_0x197, sizeof(solax_pw_cmd_get_stat_0x197), 0x197);
          solax_pw_queue_push(solax_pw_cmd_get_stat_0x25F, sizeof(solax_pw_cmd_get_stat_0x25F), 0x25F);
        }
        solax_pw_state = SOLAX_PW_SEND;
        break;

      case SOLAX_PW_SEND:
        tparse_discard(&tp_solax_pw);
        master_log("UART >> ");
        master_log_hex(solax_pw_queue[0].cmd, solax_pw_queue[0].cmd_len);
        master_log("\n");
        // send info request to solax
        uart_select_intf(UARTPW);
        uart_send_mem(solax_pw_queue[0].cmd, solax_pw_queue[0].cmd_len);
        solax_pw_state = SOLAX_PW_REQ_SENT;
        solax_pw_timeout = uwTick + SOLAX_PW_TIMEOUT;
        break;

      case SOLAX_PW_REQ_SENT:
        // if the reply is complete
        if (tparse_avail(&tp_solax_pw) >= solax_pw_queue[0].rep_len) {
          size_t read = tparse_read(&tp_solax_pw, (char*)tmp, solax_pw_queue[0].rep_len);
          if (read < solax_pw_queue[0].rep_len) {
            master_log("UART reading error ");
            master_log_hex(&read, 4);
            read = tparse_avail(&tp_solax_pw);
            master_log_hex(&read, 4);
            master_log("\n");
            goto invalid;
          }
          master_log("UART << ");
          master_log_hex(tmp, solax_pw_queue[0].rep_len);
          master_log("\n");
          // check it's the expected response
          if (tmp[0] == 0xAA && tmp[1] == 0x55 && tmp[2] == 0x5F && tmp[3] == 0x81 && tmp[4] == 0x90 ) {
            // extract fields
            solax.grid_wattage = S2LE(tmp, 9);
            solax.pv1_voltage = U2LE(tmp, 13);
            solax.pv2_voltage = U2LE(tmp, 15);
            solax.pv1_current = U2LE(tmp, 17);
            solax.pv2_current = U2LE(tmp, 19);
            solax.pv1_wattage = U2LE(tmp, 21);
            solax.pv2_wattage = U2LE(tmp, 23);
            if (tmp[25] != solax.status) {
              solax.status_count=0;
            }
            solax.status      = tmp[25];
            if (solax.status_count<255) {
              solax.status_count++;
            }
            solax.bat_wattage = S2LE(tmp, 37);
            solax.bat_temp = S2LE(tmp, 39);
            solax.bat_SoC = U2LE(tmp, 41);
            solax.output_va = U2LE(tmp, 55);
            solax.eps_power = U2LE(tmp, 61);
            solax.eps_voltage = U2LE(tmp, 63);
            solax.eps_current = U2LE(tmp, 65);
            solax.grid_meter_ct = S2LE(tmp, 69);
            solax.seconds = tmp[203];
            solax.minute = tmp[204];
            solax.hour = tmp[205];
            solax.day = tmp[206];
            solax.month = tmp[207];
            solax.year = tmp[208] + 2000;

            snprintf((char*)tmp, sizeof(tmp), "PV1: %dW (%d.%dV %d.%dA)\nPV2: %dW (%d.%dV %d.%dA)\n", solax.pv1_wattage, solax.pv1_voltage/10,solax.pv1_voltage%10, solax.pv1_current/10, solax.pv1_current%10, solax.pv2_wattage, solax.pv2_voltage/10, solax.pv2_voltage%10, solax.pv2_current/10, solax.pv2_current%10);
            master_log((char*)tmp);
            snprintf((char*)tmp, sizeof(tmp), "AC: Grid: %dW (meter %dW) EPS: %dW Output: %dVA\n", solax.grid_wattage, solax.grid_meter_ct, solax.eps_power, solax.output_va);
            master_log((char*)tmp);

            // if (!solax_checksum_verify(tmp+2,tmp[2]-2)) {
            //   batt_drain_fix_cause = 100;
            //   tparse_discard(&tp_solax_pw);
            //   solax_pw_state = SOLAX_PW_INVALID_NEXT;
            //   solax_pw_timeout = uwTick + SOLAX_PW_INVALID_RETRY_TIMEOUT;
            //   break;
            // }

            //int32_t pylontech_wattage = pylontech.precise_wattage?pylontech.precise_wattage:pylontech.wattage;
            //int32_t power_balance_w = solax.pv1_wattage + solax.pv2_wattage - (solax.grid_wattage + pylontech_wattage );
            // check for invalid data (glitch sometimes returned by the inverter)
            if (solax.pv1_voltage > SOLAX_MAX_PV_VOLTAGE_V*10 || solax.pv2_voltage > SOLAX_MAX_PV_VOLTAGE_V*10) {
              batt_drain_fix_cause = 71;
              master_log("cause 71\n");
            invalid:
              tparse_reset(&tp_solax_pw);
              solax_pw_state = SOLAX_PW_INVALID_NEXT;
              solax_pw_timeout = uwTick + SOLAX_PW_INVALID_RETRY_TIMEOUT;
              solax_pw_queue_pop();
              break;
            }
            /* this is triggered too easily when fluctuating power
            if (solax.pv1_voltage && solax.pv1_wattage > 100 && solax.pv1_voltage/10*solax.pv1_current/10 > 150*solax.pv1_wattage/100) {
              batt_drain_fix_cause = 72;
              master_log("cause 72\n");
              goto invalid;
            }
            if (solax.pv1_voltage && solax.pv1_wattage > 100 && solax.pv1_voltage/10*solax.pv1_current/10 < 50*solax.pv1_wattage/100) {
              batt_drain_fix_cause = 73;
              master_log("cause 73\n");
              goto invalid; 
            }
            if (solax.pv2_voltage && solax.pv2_wattage > 100 && solax.pv2_voltage/10*solax.pv2_current/10 > 150*solax.pv2_wattage/100) {
              batt_drain_fix_cause = 74;
              master_log("cause 74\n");
              goto invalid; 
            }
            if (solax.pv2_voltage && solax.pv2_wattage > 100 && solax.pv2_voltage/10*solax.pv2_current/10 < 50*solax.pv2_wattage/100) {
              batt_drain_fix_cause = 75;
              master_log("cause 75\n");
              goto invalid; 
            }
            */
            /*
            // check power balance is correct (with a +- variance)
            if (power_balance_w < 0 && power_balance_w < - SOLAX_SELF_CONSUMPTION_MPPT_W - SOLAX_SELF_CONSUMPTION_INVERTER_W) {
              batt_drain_fix_cause = 76;
              master_log("cause 76\n");
              goto invalid; 
            }
            if (power_balance_w > 0 && power_balance_w > SOLAX_SELF_CONSUMPTION_MPPT_W + SOLAX_SELF_CONSUMPTION_INVERTER_W) {
              batt_drain_fix_cause = 77;
              master_log("cause 77\n");
              goto invalid; 
            }
            */
            // detect invalid packet (no power flows :s)
            if (solax.grid_wattage == 0 && solax.pv1_voltage == 0 && solax.pv2_voltage == 0 && solax.bat_wattage == 0 && solax.eps_voltage == 0 && solax.output_va == 0 && solax.grid_meter_ct == 0) {
              batt_drain_fix_cause = 78;
              master_log("cause 78\n");
              goto invalid;
            }

            // only reset condition when a packet can be interpreted
            //batt_forced_charge = -1;
            //batt_forced_soc = -1;
            batt_drain_fix_cause = 0;

            // not NIGHT
            if (solax.pv1_voltage + solax.pv2_voltage > SOLAX_DAY_THRESHOLD_V*10 ) {

              // cutoff for the given duration
              if (
                (!timeout_pv1_switch_off || EXPIRED(timeout_pv1_switch_off))
                && (solax.pv1_voltage < SOLAX_PV_CUTOFF_VOLTAGE_V*10 
                   || (solax.pv1_wattage && solax.pv1_wattage < SOLAX_PV_POWER_OPT_THRESHOLD_W))) {
                master_log("pv1 switch off\n");
                pv1_switch(0);
                timeout_pv1_switch_on = uwTick + SOLAX_PV_CUTOFF_TIMEOUT;
                timeout_pv1_switch_off = 0;
              }

              // cutoff for the given duration
              if (
                (!timeout_pv2_switch_off || EXPIRED(timeout_pv2_switch_off))
                && ( solax.pv2_voltage < SOLAX_PV_CUTOFF_VOLTAGE_V*10 
                    || ( solax.pv2_wattage && solax.pv2_wattage < SOLAX_PV_POWER_OPT_THRESHOLD_W))) {
                master_log("pv2 switch off\n");
                pv2_switch(0);
                timeout_pv2_switch_on = uwTick + SOLAX_PV_CUTOFF_TIMEOUT;
                timeout_pv2_switch_off = 0;
              }

#ifdef HAVE_SOLAX_SWITCH_MODE
#ifdef HAVE_LOWLIGHT_OPT
              // if total PV voltage is below a limit, just AVOID wasting battery energy, 
              // and disable battery charging to force the solax stopping the MPPT draining 
              // energy from the battery to operate.
              // ok but when not connected to grid, could not start! // if (solax.pv1_wattage + solax.pv2_wattage < 100) 
              // OFFGRID or NIGHT could still have that condition true 
              if (timeout_pv1_switch_on == 0 
                  && solax.pv1_voltage + solax.pv2_voltage < SOLAX_PV_POWER_OPT_THRESHOLD_V*10 
                  && solax.pv1_wattage + solax.pv2_wattage < SOLAX_PV_POWER_OPT_THRESHOLD_W)  // requires some insight on the total PV array connection
              {
                batt_drain_fix_cause = 1;
                master_log("cause 1\n");
                //batt_forced_charge = 0; // deny charge (which drains battery when not enough solar)
                //batt_forced_soc = 100;
                // batt_forced_soc = SOLAX_BAT_MIN_SOC_SELFUSE;
                if (solax_pw_queue_free() >= 2 
                  && solax_forced_work_mode != SOLAX_FORCED_WORK_MODE_MANUAL_STOP
                  && solax_pw_mode_change_ready == 0
                  && self_use_auto) {
                  solax_pw_queue_push(solax_pw_cmd_mode_manual, sizeof(solax_pw_cmd_mode_manual), 7);
                  solax_pw_queue_push(solax_pw_cfg_manual_stop, sizeof(solax_pw_cfg_manual_stop), 7);
                  solax_forced_work_mode = SOLAX_FORCED_WORK_MODE_MANUAL_STOP;
                  solax_pw_mode_change_ready = uwTick + SOLAX_PW_MODE_CHANGE_MIN_INTERVAL;
                }
              }
              else if (
                timeout_pv1_switch_on == 0
                // battery is emptying in the grid, and panels aren't coping with that
                // observed than under a certain pv power, when the grid is connected, then the battery is used to power the mppt
                // grid tied
                && solax.grid_wattage /* == SELFUSE*/
                // not enough power from solar to cover solax self consumption (observed threshold)
                && solax.pv1_wattage + solax.pv2_wattage 
                   < SOLAX_PV_POWER_OPT_THRESHOLD_W
                ) {
                batt_drain_fix_cause = 4;
                master_log("cause 4\n");
                //batt_forced_charge = 0; // deny charge
                // batt_forced_soc = 100;
                // batt_forced_soc = SOLAX_BAT_MIN_SOC_SELFUSE; // deny discharge for self-use

                if (solax_pw_queue_free() >= 2 
                  && solax_forced_work_mode != SOLAX_FORCED_WORK_MODE_MANUAL_STOP
                  && solax_pw_mode_change_ready == 0
                  && self_use_auto) {
                  solax_pw_queue_push(solax_pw_cmd_mode_manual, sizeof(solax_pw_cmd_mode_manual), 7);
                  solax_pw_queue_push(solax_pw_cfg_manual_stop, sizeof(solax_pw_cfg_manual_stop), 7);
                  solax_forced_work_mode = SOLAX_FORCED_WORK_MODE_MANUAL_STOP;
                  solax_pw_mode_change_ready = uwTick + SOLAX_PW_MODE_CHANGE_MIN_INTERVAL;
                }
              }
              else 
#endif // HAVE_LOWLIGHT_OPT
              // when battery is full, then for self use
              if (pylontech.soc >= 98) {
                batt_drain_fix_cause = 3;
                master_log("cause 3\n");
                //batt_forced_charge = 0;
                // batt_forced_soc = 100;
                if (solax_forced_work_mode != SOLAX_FORCED_WORK_MODE_SELF_USE
                   && solax_pw_queue_free()
                   && solax_pw_mode_change_ready == 0
                   && self_use_auto) {
                  solax_pw_queue_push(solax_pw_cmd_mode_self_use, sizeof(solax_pw_cmd_mode_self_use), 7);
                  solax_forced_work_mode = SOLAX_FORCED_WORK_MODE_SELF_USE;
                  solax_pw_mode_change_ready = uwTick + SOLAX_PW_MODE_CHANGE_MIN_INTERVAL;
                }
              }
              // when the battery goes lower than the requested charge level, switch to charge only (no self use)
              else if (pylontech.soc < SOLAX_SELFUSE_MIN_BATTERY_SOC) {
                batt_drain_fix_cause = 8;
                master_log("cause 8\n");
                if (solax_pw_queue_free() >= 2 
                  && solax_forced_work_mode != SOLAX_FORCED_WORK_MODE_MANUAL_STOP
                  && solax_pw_mode_change_ready == 0
                  && self_use_auto) {
                  solax_pw_queue_push(solax_pw_cmd_mode_manual, sizeof(solax_pw_cmd_mode_manual), 7);
                  solax_pw_queue_push(solax_pw_cfg_manual_stop, sizeof(solax_pw_cfg_manual_stop), 7);
                  solax_forced_work_mode = SOLAX_FORCED_WORK_MODE_MANUAL_STOP;
                  solax_pw_mode_change_ready = uwTick + SOLAX_PW_MODE_CHANGE_MIN_INTERVAL;
                }
              }
              // when battery reached the requested charge level, enter self use
              else if (pylontech.soc > SOLAX_SELFUSE_MIN_BATTERY_SOC) {
                // power the grid
                batt_drain_fix_cause = 6;
                master_log("cause 6\n");
                if (solax_pw_queue_free()
                  && solax_forced_work_mode != SOLAX_FORCED_WORK_MODE_SELF_USE
                  && solax_pw_mode_change_ready == 0
                  && self_use_auto) {
                  solax_pw_queue_push(solax_pw_cmd_mode_self_use, sizeof(solax_pw_cmd_mode_self_use), 7);
                  solax_forced_work_mode = SOLAX_FORCED_WORK_MODE_SELF_USE;
                  solax_pw_mode_change_ready = uwTick + SOLAX_PW_MODE_CHANGE_MIN_INTERVAL;
                }
              }
#endif // HAVE_SOLAX_SWITCH_MODE
            }
            // NIGHT
            else {
                if (timeout_pv1_switch_on == 0) {
                batt_drain_fix_cause = 7;
                master_log("cause 7\n");
                //batt_forced_charge = 0;
                if (solax_pw_queue_free() 
                  && solax_forced_work_mode != SOLAX_FORCED_WORK_MODE_SELF_USE
                  && solax_pw_mode_change_ready == 0
                  && self_use_auto) {
                  solax_pw_queue_push(solax_pw_cmd_mode_self_use, sizeof(solax_pw_cmd_mode_self_use), 7);
                  solax_forced_work_mode = SOLAX_FORCED_WORK_MODE_SELF_USE;
                  solax_pw_mode_change_ready = uwTick + SOLAX_PW_MODE_CHANGE_MIN_INTERVAL;
                }
              }
            }
          }
#ifdef WORKAROUND_SOLAX_INJECTION_SURGE
          master_log("Solax: status=0x");
          master_log_hex(&solax.status, 1);
          master_log(" count=0x");
          master_log_hex(&solax.status_count, 1);
          master_log("\n");
          // only do this after the inverter status is stable and ready for connection
          if (solax.status_count >= GRID_SWITCH_STATE_COUNT) {
            // when max charge value is degraded, then severs the grid connection
            if (pylontech.soc >= solax.grid_disconnect_soc
              || pylontech.max_charge < pylontech.max_discharge
              || pylontech.soc >= pylontech.max_charge_soc
              ) {
#if 0
              switch(solax.status) {
              case INVERTER_STATUS_EPS_WAIT:
                // we have already severed connection with the grid, 
              case INVERTER_STATUS_WAITING:
              case INVERTER_STATUS_CHECKING:
              case INVERTER_STATUS_SELFTEST:
                // no action
                master_log("Antisurge: do nothing (1)\n");
                break;
              // failing states, must reenable grid!!
              case INVERTER_STATUS_IDLE: // already connected
              case INVERTER_STATUS_ERROR:
              case INVERTER_STATUS_FAULT:
              case INVERTER_STATUS_STANDBY:
              case INVERTER_STATUS_UPDATE:
                if (eps_mode_switch_auto) {
                  master_log("Antisurge: connect GRID (1)\n");
                  eps_mode_switch(0);
                  solax.status_count = 0; // avoid glitching too frequently
                  eps_mode_switch_timeout = uwTick + SOLAX_EPS_MODE_SWITCH_TIMEOUT_MS;
                }
                break;
              case INVERTER_STATUS_EPS:
              case INVERTER_STATUS_NORMAL:
                if (eps_mode_switch_auto) {
                  if (eps_mode_switch_timeout == 0 || EXPIRED(eps_mode_switch_timeout)) {
                    master_log("Antisurge: disconnect GRID, force EPS\n");
                    eps_mode_switch(1);
                    solax.status_count = 0; // avoid glitching too frequently
                    eps_mode_switch_timeout = uwTick + SOLAX_EPS_MODE_SWITCH_TIMEOUT_MS;
                  }
                }
                break;
              }
#else 
              if (eps_mode_switch_auto 
                /* we inject */ //&& (self_use_auto || solax_forced_work_mode != SOLAX_FORCED_WORK_MODE_MANUAL_STOP)
                && solax.status != INVERTER_STATUS_ERROR 
                && solax.status != INVERTER_STATUS_FAULT
                // COMMENTED: don't require EPS when already in EPS // avoid switching to grid when reconnecting 
                // the grid => avoid checking mode, and then a 5 seconds injection window after reaching normal mode
                //&& solax.status != INVERTER_STATUS_EPS 
                //&& solax.status != INVERTER_STATUS_EPS_WAIT // don't require EPS when already in EPS
                // 
                //&& solax.status != INVERTER_STATUS_WAITING // going to normal
                // when disabled during checking, then it oftenly fails
                //&& solax.status != INVERTER_STATUS_CHECKING // going to normal 
                ) {
                master_log("Antisurge: disconnect GRID, force EPS\n");
                eps_mode_switch(1);
                solax.status_count = 0; // avoid glitching too frequently
                eps_mode_switch_timeout = uwTick + SOLAX_EPS_MODE_SWITCH_TIMEOUT_MS;
              }
#endif // 0
            }
            // when SoC is lower than a value, then 
            else if (pylontech.soc <= solax.grid_connect_soc) {
              if (eps_mode_switch_auto
                /* we inject */ //&& (self_use_auto || solax_forced_work_mode != SOLAX_FORCED_WORK_MODE_MANUAL_STOP)
                ) {
                master_log("Antisurge: connect GRID (2)\n");
                // restablish the GRID connection, 
                eps_mode_switch(0);
                solax.status_count = 0; // avoid glitching too frequently
                eps_mode_switch_timeout = uwTick + SOLAX_EPS_MODE_SWITCH_TIMEOUT_MS;
              }
            }
            else if (solax.status_count >= GRID_SWITCH_STATE_COUNT) {
              switch(solax.status) {
              // failing states, must reenable grid!!
              case INVERTER_STATUS_IDLE:
              case INVERTER_STATUS_ERROR:
              case INVERTER_STATUS_FAULT:
              case INVERTER_STATUS_STANDBY:
              case INVERTER_STATUS_UPDATE:
                if (eps_mode_switch_auto
                  /* we inject */ //&& (self_use_auto || solax_forced_work_mode != SOLAX_FORCED_WORK_MODE_MANUAL_STOP)
                  ) {
                  master_log("Antisurge: connect GRID (3)\n");
                  eps_mode_switch(0);
                  solax.status_count = 0; // avoid glitching too frequently
                  eps_mode_switch_timeout = uwTick + SOLAX_EPS_MODE_SWITCH_TIMEOUT_MS;
                }
                break;
              }
            }
            else{
              master_log("Antisurge: do nothing (2)\n");
            }
          }
          else {
            // wait state to stabilize
          }
#endif // WORKAROUND_SOLAX_INJECTION_SURGE

          // whatever the reply, discard the data after this point
          solax_pw_state = SOLAX_PW_WAIT_NEXT; // switch state before possibly scheduling a command to send
          solax_pw_timeout = uwTick + SOLAX_PW_NEXT_TIMEOUT;
          solax_pw_queue_pop();
        }
        // timing out first entry if any
        else if (solax_pw_queue_free() != SOLAX_PW_QUEUE_SIZE && EXPIRED(solax_pw_timeout)) {
          master_log("UART TIMEOUT");
          //master_log_hex(uart_pw_buffer, sizeof(uart_pw_buffer));
          solax_pw_state = SOLAX_PW_WAIT_NEXT;
          solax_pw_timeout = uwTick + SOLAX_PW_NEXT_TIMEOUT;
          tparse_reset(&tp_solax_pw);
          solax_pw_queue_pop();
          // discard causes due to timeout of the request
          batt_drain_fix_cause = 0;
          //batt_forced_charge = -1;
          //batt_forced_soc = -1;

          Configure_UARTPW(9600);
          uart_select_intf(UARTPW);
          uart_send_mem(solax_pw_cmd_change_bitrate, sizeof(solax_pw_cmd_change_bitrate));  
          // wait until bitrate is taken into account
          LL_mDelay(250);
          Configure_UARTPW(115200);
        }
        break;

      case SOLAX_PW_INVALID_NEXT:
      case SOLAX_PW_WAIT_NEXT:
        // skip to next command sending immediately, this is not a new attempt/request
        if (EXPIRED(solax_pw_timeout) || solax_pw_queue_free() != SOLAX_PW_QUEUE_SIZE) {
          solax_pw_state = SOLAX_PW_IDLE;
        }
        break;
    }

    if (EXPIRED(solax_pw_mode_change_ready)) {
      solax_pw_mode_change_ready = 0;
    }

#ifdef USART5_HUMAN_READABLE_SUMMARY_LOG
    // display (if no erroneous data)
    if (EXPIRED(timeout_next_display) && batt_drain_fix_cause < 70) {
      // prepare next sending
      timeout_next_display = uwTick + DISPLAY_TIMEOUT;
      {
        int32_t bat_wattage_corr = solax.bat_wattage
          +(solax.bat_wattage?(solax.bat_wattage>0?-SOLAX_SELF_CONSUMPTION_INVERTER_W-SOLAX_SELF_CONSUMPTION_MPPT_W:+SOLAX_SELF_CONSUMPTION_INVERTER_W+SOLAX_SELF_CONSUMPTION_MPPT_W):0);
        // wipe screen
        if (solax.grid_meter_ct >= 0) {
          snprintf(tmp, sizeof(tmp), VERSION " EXP %dW|%s:%d %d\n", solax.grid_meter_ct, solax_forced_work_mode_str[solax_forced_work_mode], solax.status, batt_drain_fix_cause);
        }
        else {
          snprintf(tmp, sizeof(tmp), VERSION " IMP %dW|%s:%d %d\n", -solax.grid_meter_ct, solax_forced_work_mode_str[solax_forced_work_mode], solax.status, batt_drain_fix_cause);
        }
        uart_select_intf(UART5);
        uart_send_mem(tmp, strlen(tmp));
        master_log(tmp);
        if (solax.grid_wattage >= 0) {
          snprintf(tmp, sizeof(tmp), "HOUSE< %dW\n", solax.grid_wattage);
        }
        else {
          snprintf(tmp, sizeof(tmp), "HOUSE> %dW\n", -solax.grid_wattage);
        }
        uart_select_intf(UART5);
        uart_send_mem(tmp, strlen(tmp));
        master_log(tmp);
        // BEGIN BATT
        int32_t pylontech_wattage = pylontech.precise_wattage?pylontech.precise_wattage:pylontech.wattage;
        if (pylontech_wattage >= 0) {
          snprintf(tmp, sizeof(tmp), "BAT< %dW", pylontech_wattage);
        }
        else {
          snprintf(tmp, sizeof(tmp), "BAT> %dW", -pylontech_wattage);
        }
        uart_select_intf(UART5);
        uart_send_mem(tmp, strlen(tmp));
        master_log(tmp);
        #if 0
        // display current flowing through the battery
        int16_t cur = pylontech.current;
        if (cur < 0) {
          uart_send_mem("-", 1);
          cur = -cur;
        }
        snprintf(tmp, sizeof(tmp), " %d.%dA", cur/10, cur%10)
        uart_select_intf(UART5);
        uart_send_mem(tmp, strlen(tmp));
        master_log(tmp);
        #endif
        if (batt_forced_soc== -1) {
          snprintf(tmp, sizeof(tmp), " %u%% %dA\n", pylontech.soc, (batt_forced_charge==-1?pylontech.max_charge:batt_forced_charge)/10);
        }
        else {
          snprintf(tmp, sizeof(tmp), " %u%% %dA F%u%%\n", pylontech.soc, (batt_forced_charge==-1?pylontech.max_charge:batt_forced_charge)/10, batt_forced_soc);
        }
        uart_select_intf(UART5);
        uart_send_mem(tmp, strlen(tmp));
        master_log(tmp);
        // END BAT

        snprintf(tmp, sizeof(tmp), "PV %uV %dW%s%s%s\n", solax.pv1_voltage/10, solax.pv1_wattage, solax.pv1_voltage + solax.pv2_voltage <= SOLAX_DAY_THRESHOLD_V * 10 ? " N":" D", batt_drain_fix_cause==1?" LO":"", solax.pv1_switch_on?"":" x");
        uart_select_intf(UART5);
        uart_send_mem(tmp, strlen(tmp));
        master_log(tmp);
      }
    }
#endif // USART5_HUMAN_READABLE_SUMMARY_LOG

  } // end infinite loop

}

#define SLAVE_OWN_ADDRESS 0x44

uint8_t i2c_xfer_buffer[256];
uint32_t i2c_xfer_w_length;
uint32_t i2c_xfer_r_offset;
uint32_t i2c_xfer_r_length;
void I2C_Slave_Match_Callback(void) {
  i2c_xfer_w_length = 0;
}

void I2C_Slave_Reception_Callback(void) {
  if (i2c_xfer_w_length == 0) {
    i2c_xfer_r_length = 0;
    i2c_xfer_r_offset = 0;
  }
  i2c_xfer_buffer[i2c_xfer_w_length++] = LL_I2C_ReceiveData8(I2CS);

  // instruction byte interp, for single byte commands
  if (i2c_xfer_w_length == 1) {
    switch(i2c_xfer_buffer[0]) {
    case 0: // get info
      // read stats
      // don't process when an error has been detected, only ignore optimization rules (< 0x70)
      if (batt_drain_fix_cause < 70 && pylontech.soc != 0 && pylontech.soc != 255) 
      {
        i2c_xfer_r_length = 0; // wipe the previous buffer content
        // data encoding version
        i2c_xfer_r_length++; // total length, reserve space
        i2c_xfer_buffer[i2c_xfer_r_length++] = 2; 

        // solax state
        i2c_xfer_buffer[i2c_xfer_r_length++] = solax.status; 
        // solax work mode
        i2c_xfer_buffer[i2c_xfer_r_length++] = solax_forced_work_mode;

        // grid export wattage
        i2c_xfer_buffer[i2c_xfer_r_length++] = (solax.grid_meter_ct>>8)&0xFF;
        i2c_xfer_buffer[i2c_xfer_r_length++] = solax.grid_meter_ct&0xFF;
        // internal grid wattage
        i2c_xfer_buffer[i2c_xfer_r_length++] = (solax.grid_wattage>>8)&0xFF;
        i2c_xfer_buffer[i2c_xfer_r_length++] = solax.grid_wattage&0xFF;
        // pv1 wattage
        i2c_xfer_buffer[i2c_xfer_r_length++] = (solax.pv1_wattage>>8)&0xFF;
        i2c_xfer_buffer[i2c_xfer_r_length++] = solax.pv1_wattage&0xFF;
        // pv2 wattage
        i2c_xfer_buffer[i2c_xfer_r_length++] = (solax.pv2_wattage>>8)&0xFF;
        i2c_xfer_buffer[i2c_xfer_r_length++] = solax.pv2_wattage&0xFF;
        // bat wattage
        i2c_xfer_buffer[i2c_xfer_r_length++] = (solax.bat_wattage>>8)&0xFF;
        i2c_xfer_buffer[i2c_xfer_r_length++] = solax.bat_wattage&0xFF;
        // bat effective wattage (0.1A rounding if no precise wattage provided)
        int32_t pylontech_wattage = pylontech.precise_wattage?pylontech.precise_wattage:pylontech.wattage;
        i2c_xfer_buffer[i2c_xfer_r_length++] = (pylontech_wattage>>8)&0xFF;
        i2c_xfer_buffer[i2c_xfer_r_length++] = pylontech_wattage&0xFF;
        // bat soc
        i2c_xfer_buffer[i2c_xfer_r_length++] = pylontech.soc;
        // bat max charge
        i2c_xfer_buffer[i2c_xfer_r_length++] = (pylontech.max_charge>>8)&0xFF;
        i2c_xfer_buffer[i2c_xfer_r_length++] = pylontech.max_charge&0xFF;
        // bat max discharge
        i2c_xfer_buffer[i2c_xfer_r_length++] = (pylontech.max_discharge>>8)&0xFF;
        i2c_xfer_buffer[i2c_xfer_r_length++] = pylontech.max_discharge&0xFF;
        // bat forced max charge
        i2c_xfer_buffer[i2c_xfer_r_length++] = (batt_forced_charge>>8)&0xFF;
        i2c_xfer_buffer[i2c_xfer_r_length++] = batt_forced_charge&0xFF;
        // last rule executed
        i2c_xfer_buffer[i2c_xfer_r_length++] = batt_drain_fix_cause;
        // solax work_mode
        i2c_xfer_buffer[i2c_xfer_r_length++] = solax.work_mode;
        // pv1 voltage
        i2c_xfer_buffer[i2c_xfer_r_length++] = (solax.pv1_voltage>>8)&0xFF;
        i2c_xfer_buffer[i2c_xfer_r_length++] = solax.pv1_voltage&0xFF;
        // pv2 voltage
        i2c_xfer_buffer[i2c_xfer_r_length++] = (solax.pv2_voltage>>8)&0xFF;
        i2c_xfer_buffer[i2c_xfer_r_length++] = solax.pv2_voltage&0xFF;
        // eps current
        i2c_xfer_buffer[i2c_xfer_r_length++] = (solax.eps_current>>8)&0xFF;
        i2c_xfer_buffer[i2c_xfer_r_length++] = solax.eps_current&0xFF;
        // timestamp
        i2c_xfer_buffer[i2c_xfer_r_length++] = solax.year>>8;
        i2c_xfer_buffer[i2c_xfer_r_length++] = solax.year&0xFF;
        i2c_xfer_buffer[i2c_xfer_r_length++] = solax.month;
        i2c_xfer_buffer[i2c_xfer_r_length++] = solax.day;
        i2c_xfer_buffer[i2c_xfer_r_length++] = solax.hour;
        i2c_xfer_buffer[i2c_xfer_r_length++] = solax.minute;
        // state of PV switch for low voltage cutoff (transient state)
        i2c_xfer_buffer[i2c_xfer_r_length++] = solax.pv1_switch_on;
        i2c_xfer_buffer[i2c_xfer_r_length++] = solax.pv2_switch_on;
        // eps power
        i2c_xfer_buffer[i2c_xfer_r_length++] = (solax.eps_power>>8)&0xFF;
        i2c_xfer_buffer[i2c_xfer_r_length++] = solax.eps_power&0xFF;
        // eps voltage
        i2c_xfer_buffer[i2c_xfer_r_length++] = (solax.eps_voltage>>8)&0xFF;
        i2c_xfer_buffer[i2c_xfer_r_length++] = solax.eps_voltage&0xFF;
        // time since restart
        U4BE_ENCODE(i2c_xfer_buffer, i2c_xfer_r_length, uwTick);
        i2c_xfer_r_length+=4;
        // output VA
        i2c_xfer_buffer[i2c_xfer_r_length++] = (solax.output_va>>8)&0xFF;
        i2c_xfer_buffer[i2c_xfer_r_length++] = solax.output_va&0xFF;
        // output auto switches
        i2c_xfer_buffer[i2c_xfer_r_length++] = self_use_auto;
        i2c_xfer_buffer[i2c_xfer_r_length++] = eps_mode_switch_auto;
        i2c_xfer_buffer[i2c_xfer_r_length++] = eps_forced_state;
        i2c_xfer_buffer[i2c_xfer_r_length++] = solax.grid_connect_soc;
        i2c_xfer_buffer[i2c_xfer_r_length++] = solax.grid_disconnect_soc;
        i2c_xfer_buffer[i2c_xfer_r_length++] = pylontech.max_charge_soc;
        U4BE_ENCODE(i2c_xfer_buffer, i2c_xfer_r_length, pylontech.precise_mAh);
        i2c_xfer_r_length+=4;
        U4BE_ENCODE(i2c_xfer_buffer, i2c_xfer_r_length, pylontech.precise_mWh);
        i2c_xfer_r_length+=4;
        i2c_xfer_buffer[i2c_xfer_r_length++] = pylontech.soc_mWh;
        for (int i = 0; i < pylontech.bmu_idx && i2c_xfer_r_length+8 < sizeof(i2c_xfer_buffer); i++) {
          // if frame >= 128 bytes, then next frame is wrongly retrieved. this is weird
          //i2c_xfer_buffer[i2c_xfer_r_length++] = ((pylontech.bmu[i].pcba[18]-0x30)<<4)|(pylontech.bmu[i].pcba[19]-0x30);
          i2c_xfer_buffer[i2c_xfer_r_length++] = ((pylontech.bmu[i].pcba[20]-0x30)<<4)|(pylontech.bmu[i].pcba[21]-0x30);
          i2c_xfer_buffer[i2c_xfer_r_length++] = pylontech.bmu[i].soc;
          i2c_xfer_buffer[i2c_xfer_r_length++] = pylontech.bmu[i].soc_mWh;
          i2c_xfer_buffer[i2c_xfer_r_length++] = (pylontech.bmu[i].vlow>>8)&0xFF;
          i2c_xfer_buffer[i2c_xfer_r_length++] = (pylontech.bmu[i].vlow)&0xFF;
          i2c_xfer_buffer[i2c_xfer_r_length++] = (pylontech.bmu[i].vhigh>>8)&0xFF;
          i2c_xfer_buffer[i2c_xfer_r_length++] = (pylontech.bmu[i].vhigh)&0xFF;
        }

        // encode total length
        i2c_xfer_buffer[0] = i2c_xfer_r_length;
      }
      // DESIGN NOTE: TXIS is raised right when a READ transaction match occurs
      break;
    case 1:
      master_log("I2C: pv1 gmppt off\n");
      // perform gmppt wakeup
      if (solax_pw_queue_free()>=2) {
        solax_pw_queue_push(solax_pw_cfg_PIN, sizeof(solax_pw_cfg_PIN), 7);
        solax_pw_queue_push(solax_pw_cfg_gmppt_pv1_off, sizeof(solax_pw_cfg_gmppt_pv1_off), 7);
      }
      break;
    case 2:
      master_log("I2C: pv1 gmppt high\n");
      if (solax_pw_queue_free()>=2) {
        solax_pw_queue_push(solax_pw_cfg_PIN, sizeof(solax_pw_cfg_PIN), 7);
        solax_pw_queue_push(solax_pw_cfg_gmppt_pv1_high, sizeof(solax_pw_cfg_gmppt_pv1_high), 7);
      }
      break;
    case 3:
      master_log("I2C: pv2 gmppt off\n");
      // perform gmppt wakeup
      if (solax_pw_queue_free()>=2) {
        solax_pw_queue_push(solax_pw_cfg_PIN, sizeof(solax_pw_cfg_PIN), 7);
        solax_pw_queue_push(solax_pw_cfg_gmppt_pv2_off, sizeof(solax_pw_cfg_gmppt_pv2_off), 7);
      }
      break;
    case 4:
      master_log("I2C: pv2 gmppt high\n");
      if (solax_pw_queue_free()>=2) {
        solax_pw_queue_push(solax_pw_cfg_PIN, sizeof(solax_pw_cfg_PIN), 7);
        solax_pw_queue_push(solax_pw_cfg_gmppt_pv2_high, sizeof(solax_pw_cfg_gmppt_pv2_high), 7);
      }
      break;
    case 0xA:
      master_log("I2C: force offgrid\n");
      // disallow gridtie
      eps_mode_switch_auto = 0;
      eps_mode_switch(1);
      break;
    case 0xB:
      master_log("I2C: auto grid connection\n");
      // allow gridtie
      eps_mode_switch_auto = 1;
      break;
    case 0xC:
      master_log("I2C: force gridtie\n");
      // force gridtie
      eps_mode_switch_auto = 0;
      eps_mode_switch(0);
      break;
    case 0xD:
      master_log("I2C: force manual stop discharge\n");
      // force stop discharge
      self_use_auto = 0;
      if (solax_pw_queue_free() >= 2) {
        solax_pw_queue_push(solax_pw_cmd_mode_manual, sizeof(solax_pw_cmd_mode_manual), 7);
        solax_pw_queue_push(solax_pw_cfg_manual_stop, sizeof(solax_pw_cfg_manual_stop), 7);
        solax_forced_work_mode = SOLAX_FORCED_WORK_MODE_MANUAL_STOP;
      }
      break;
    case 0xE:
      master_log("I2C: force self use\n");
      // allow discharge
      self_use_auto = 0;
      if (solax_pw_queue_free()) {
        solax_pw_queue_push(solax_pw_cmd_mode_self_use, sizeof(solax_pw_cmd_mode_self_use), 7);
        solax_forced_work_mode = SOLAX_FORCED_WORK_MODE_SELF_USE;
      }
      break;
    case 0xF:
      master_log("I2C: auto self use\n");
      // automatic stop/allow discharge based on percentage
      self_use_auto = 1;
      break;

    case 0x13:
      batt_forced_charge = -1;
      break;
    }
  }
  else {
    // double bytes instructions
    if (i2c_xfer_w_length == 2) {
      switch(i2c_xfer_buffer[0]) {
      case 0x10:
        solax.grid_connect_soc = i2c_xfer_buffer[1];
        break;
      case 0x11:
        solax.grid_disconnect_soc = i2c_xfer_buffer[1];
        break;
      case 0x12:
        pylontech.max_charge_soc = i2c_xfer_buffer[1];
        break;
        // force charge (to balance batteries)
      case 0x14:
        batt_forced_charge = i2c_xfer_buffer[1]; // in dA
        break;
      }
    }
  }
}

void I2C_Slave_Ready_To_Transmit_Callback(void) {
  if (i2c_xfer_r_offset < i2c_xfer_r_length) {
    LL_I2C_TransmitData8(I2CS, i2c_xfer_buffer[i2c_xfer_r_offset++]);
  }
  else {
    // stuffing
    LL_I2C_TransmitData8(I2CS, 0xAA);
  }
}

void I2C_Slave_Complete_Callback(void) {
  
}

void I2C_Error_Callback(void) {
  i2c_xfer_r_offset = i2c_xfer_r_length = i2c_xfer_w_length = 0;
}

/**
  * Brief   This function handles I2CS (Slave) event interrupt request.
  * Param   None
  * Retval  None
  */
void I2CS_EV_IRQHandler(void)
{
  /* Check ADDR flag value in ISR register */
  if(LL_I2C_IsActiveFlag_ADDR(I2CS))
  {
    /* Verify the Address Match wI2C_Slave_Complete_Callbackith the OWN Slave address */
    if(LL_I2C_GetAddressMatchCode(I2CS) == SLAVE_OWN_ADDRESS)
    {
      I2C_Slave_Match_Callback();
      // /* Verify the transfer direction, a read direction, Slave enters transmitter mode */
      // if(LL_I2C_GetTransferDirection(I2CS) == LL_I2C_DIRECTION_READ)
      // {
      /* Clear ADDR flag value in ISR register */
      LL_I2C_ClearFlag_ADDR(I2CS);

      //   /* Enable Transmit Interrupt */
      //   LL_I2C_EnableIT_TX(I2CS);

      // }
      // else
      // {
      //   /* Clear ADDR flag value in ISR register */
      //   LL_I2C_ClearFlag_ADDR(I2CS);

      //   /* Call Error function */
      //   I2C_Error_Callback();
      // }
    }
    else
    {
      /* Clear ADDR flag value in ISR register */
      LL_I2C_ClearFlag_ADDR(I2CS);
        
      /* Call Error function */
      I2C_Error_Callback();
    }
  }
  /* Check NACK flag value in ISR register */
  else if(LL_I2C_IsActiveFlag_NACK(I2CS))
  {
    /* End of Transfer */
    LL_I2C_ClearFlag_NACK(I2CS);
  }
  /* Check RXNE flag value in ISR register */
  else if(LL_I2C_IsActiveFlag_RXNE(I2CS))
  {
    /* Call function Slave Reception Callback */
    I2C_Slave_Reception_Callback();
  }
  /* Check TXIS flag value in ISR register */
  else if(LL_I2C_IsActiveFlag_TXIS(I2CS))
  {
    /* Call function Slave Ready to Transmit Callback */
    I2C_Slave_Ready_To_Transmit_Callback();
  }
  /* Check STOP flag value in ISR register */
  else if(LL_I2C_IsActiveFlag_STOP(I2CS))
  {
    /* Clear STOP flag value in ISR register */
    LL_I2C_ClearFlag_STOP(I2CS);
    
    /* Check TXE flag value in ISR register */
    if(!LL_I2C_IsActiveFlag_TXE(I2CS))
    {
      /* Flush the TXDR register */
      LL_I2C_ClearFlag_TXE(I2CS);
    }

    /* Call function Slave Complete Callback */
    I2C_Slave_Complete_Callback();
  }
  /* Check TXE flag value in ISR register */
  else if(!LL_I2C_IsActiveFlag_TXE(I2CS))
  {
    /* Do nothing */
    /* This Flag will be set by hardware when the TXDR register is empty */
    /* If needed, use LL_I2C_ClearFlag_TXE() interface to flush the TXDR register  */
  }
  // tested in situ that nothing interesting goes here!
  // else
  // {
  //   volatile uint32_t isr = I2CS->ISR;
  //   /* Call Error function */
  //   I2C_Error_Callback();
  // }
}

/**
  * Brief   This function handles I2CS (Slave) error interrupt request.
  * Param   None
  * Retval  None
  */
void I2CS_ER_IRQHandler(void)
{
  /* Call Error function */
  I2C_Error_Callback();
}

void Configure_I2C_Slave(void)
{

  /* (1) Enables GPIO clock and configures the I2CS pins **********************/

  #ifdef BOARD_DEV
  /* Enable the peripheral clock of GPIOB */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  /* Configure SCL Pin as : Alternate function, High Speed, Open drain, Pull up */
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_10, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(GPIOB, LL_GPIO_PIN_10, LL_GPIO_AF_4);
  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_10, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_10, LL_GPIO_OUTPUT_OPENDRAIN);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_10, LL_GPIO_PULL_UP);

  /* Configure SDA Pin as : Alternate function, High Speed, Open drain, Pull up */
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_11, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(GPIOB, LL_GPIO_PIN_11, LL_GPIO_AF_4);
  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_11, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_11, LL_GPIO_OUTPUT_OPENDRAIN);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_11, LL_GPIO_PULL_UP);

  /* (2) Enable the I2CS peripheral clock and I2CS clock source ***************/

  /* Enable the peripheral clock for I2CS */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C2);
  /* Set I2C2 clock source as SYSCLK */
  LL_RCC_SetI2CClockSource(LL_RCC_I2C2_CLKSOURCE_SYSCLK);
  /* (3) Configure NVIC for I2CS **********************************************/

  /* Configure Event IT:
   *  - Set priority for I2C2_EV_IRQn
   *  - Enable I2C2_EV_IRQn
   */
  NVIC_SetPriority(I2C2_EV_IRQn, 0);  
  NVIC_EnableIRQ(I2C2_EV_IRQn);

  /* Configure Error IT:
   *  - Set priority for I2C2_ER_IRQn
   *  - Enable I2C2_ER_IRQn
   */
  NVIC_SetPriority(I2C2_ER_IRQn, 0);  
  NVIC_EnableIRQ(I2C2_ER_IRQn);
  #else
  /* Enable the peripheral clock of GPIOB */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);

  /* Configure SCL Pin as : Alternate function, High Speed, Open drain, Pull up */
  LL_GPIO_SetPinMode(GPIOF, LL_GPIO_PIN_14, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(GPIOF, LL_GPIO_PIN_14, LL_GPIO_AF_4);
  LL_GPIO_SetPinSpeed(GPIOF, LL_GPIO_PIN_14, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(GPIOF, LL_GPIO_PIN_14, LL_GPIO_OUTPUT_OPENDRAIN);
  LL_GPIO_SetPinPull(GPIOF, LL_GPIO_PIN_14, LL_GPIO_PULL_UP);

  /* Configure SDA Pin as : Alternate function, High Speed, Open drain, Pull up */
  LL_GPIO_SetPinMode(GPIOF, LL_GPIO_PIN_15, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(GPIOF, LL_GPIO_PIN_15, LL_GPIO_AF_4);
  LL_GPIO_SetPinSpeed(GPIOF, LL_GPIO_PIN_15, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(GPIOF, LL_GPIO_PIN_15, LL_GPIO_OUTPUT_OPENDRAIN);
  LL_GPIO_SetPinPull(GPIOF, LL_GPIO_PIN_15, LL_GPIO_PULL_UP);


  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C4);
  /* Set I2C4 clock source as SYSCLK */
  LL_RCC_SetI2CClockSource(LL_RCC_I2C4_CLKSOURCE_SYSCLK);
  /* (3) Configure NVIC for I2CS **********************************************/

  /* Configure Event IT:
   *  - Set priority for I2C4_EV_IRQn
   *  - Enable I2C4_EV_IRQn
   */
  NVIC_SetPriority(I2C4_EV_IRQn, 0);  
  NVIC_EnableIRQ(I2C4_EV_IRQn);

  /* Configure Error IT:
   *  - Set priority for I2C4_ER_IRQn
   *  - Enable I2C4_ER_IRQn
   */
  NVIC_SetPriority(I2C4_ER_IRQn, 0);  
  NVIC_EnableIRQ(I2C4_ER_IRQn);
  #endif

  /* (4) Configure I2CS functional parameters *********************************/

  /* Disable I2CS prior modifying configuration registers */
  LL_I2C_Disable(I2CS);

  /* Configure the SDA setup, hold time and the SCL high, low period */
  LL_I2C_SetTiming(I2CS, 0x00100105);

  /* Configure the Own Address1 :
   *  - OwnAddress1 is SLAVE_OWN_ADDRESS
   *  - OwnAddrSize is LL_I2C_OWNADDRESS1_7BIT
   *  - Own Address1 is enabled
   */
  LL_I2C_SetOwnAddress1(I2CS, SLAVE_OWN_ADDRESS, LL_I2C_OWNADDRESS1_7BIT);
  LL_I2C_EnableOwnAddress1(I2CS);

  /* Enable Clock stretching */
  /* Reset Value is Clock stretching enabled */
  //LL_I2C_EnableClockStretching(I2CS);

  /* Configure Digital Noise Filter */
  /* Reset Value is 0x00            */
  //LL_I2C_SetDigitalFilter(I2CS, 0x00);

  /* Enable Analog Noise Filter           */
  /* Reset Value is Analog Filter enabled */
  //LL_I2C_EnableAnalogFilter(I2CS);

  /* Enable General Call                  */
  /* Reset Value is General Call disabled */
  //LL_I2C_EnableGeneralCall(I2CS);

  /* Configure the 7bits Own Address2               */
  /* Reset Values of :
   *     - OwnAddress2 is 0x00
   *     - OwnAddrMask is LL_I2C_OWNADDRESS2_NOMASK
   *     - Own Address2 is disabled
   */
  //LL_I2C_SetOwnAddress2(I2CS, 0x00, LL_I2C_OWNADDRESS2_NOMASK);
  //LL_I2C_DisableOwnAddress2(I2CS);

  /* Enable Peripheral in I2C mode */
  /* Reset Value is I2C mode */
  //LL_I2C_SetMode(I2CS, LL_I2C_MODE_I2C);

  /* (5) Enable I2CS **********************************************************/
  LL_I2C_Enable(I2CS);

  /* (6) Enable I2CS address match/error interrupts:
   *  - Enable Address Match Interrupt
   *  - Enable Not acknowledge received interrupt
   *  - Enable Error interrupts
   *  - Enable Stop interrupt
   */
  LL_I2C_EnableIT_ADDR(I2CS);
  LL_I2C_EnableIT_NACK(I2CS);
  LL_I2C_EnableIT_ERR(I2CS);
  LL_I2C_EnableIT_STOP(I2CS);
  LL_I2C_EnableIT_RX(I2CS);
  LL_I2C_EnableIT_TX(I2CS);


#ifdef BOARD_DEV
  NVIC_EnableIRQ(I2C2_EV_IRQn);
  NVIC_EnableIRQ(I2C2_ER_IRQn);
#else // BOARD_DEV
  NVIC_EnableIRQ(I2C4_EV_IRQn);
  NVIC_EnableIRQ(I2C4_ER_IRQn);
#endif // BOARD_DEV
}

#endif // MODE_SOLAX_BMS
