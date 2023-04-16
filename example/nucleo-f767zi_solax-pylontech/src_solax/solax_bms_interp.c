
#include "main.h"
#include "tparse.h"
#include "stddef.h"
#include "stdio.h"

#ifdef MODE_SOLAX_BMS

#define USBVCP USART3

// required with version ARM=1.07+DSP=1.09, 
// not required with ARM=1.28+DSP=1.30 => optimization is far better with this firmware version
//#define HAVE_LOWLIGHT_OPT 

//#define BMS_PING
#define BMS_PING_INTERVAL_MS 5000
#define BMS_UART_TIMEOUT 10000
#define BMS_UART_NEXT_INTERVAL 1000

#define BMS_KIND_BLANK 0x50
#define BMS_KIND_BAK 0x51
#define BMS_KIND_REPT 0x52 // 4x H48050
#define BMS_KIND_SINOWATT 0x53
#define BMS_KIND_GOT 0x54
#define BMS_KIND_BLANK2 0x55
#define BMS_KIND_TP200 0x81
#define BMS_KIND_TP201 0x82 
#define BMS_KIND_TP202 0x83 // 8x H48050
// change depending on the battery configuration if it doesn't work out
#define BMS_KIND BMS_KIND_TP202

#define BMS_RECONNECT_DELAY 20000

#define SOLAX_PW_TIMEOUT 10000 // give few seconds for 400 bytes @ 9600bps
#define SOLAX_PW_NEXT_TIMEOUT 1000 // pocket wifi link update
#define SOLAX_PW_INVALID_RETRY_TIMEOUT 500 // 100ms before retrying in case of an error on the pocketwifi serial response
#define SOLAX_PW_MODE_CHANGE_MIN_INTERVAL 10000 // avoid changing mode constantly
#define PYLONTECH_ACTIVITY_TIMEOUT 500


#define SOLAX_PV_POWER_OPT_THRESHOLD_V 150
#define SOLAX_PV_POWER_OPT_THRESHOLD_W 25
#define SOLAX_GRID_EXPORT_OPT_THRESHOLD_W 50

#define SOLAX_MAX_PV_VOLTAGE_V 600 // from user manual

#define SOLAX_DAY_THRESHOLD_V 50 // below 50v is considered NIGHT (with high exposure nights, it has been measured as much)
#define SOLAX_SELF_CONSUMPTION_MPPT_W 40 // observed inverter consumption when MPPT is working
#define SOLAX_SELF_CONSUMPTION_INVERTER_W 40 // observed inverter consumption with only inverter enabled (not system off)

// before 80% of charge of battery, be conservative, and charge first
#define SOLAX_SELFUSE_MIN_BATTERY_SOC 40
#define SOLAX_SELFUSE_MIN_BATTERY_CHARGE_PERCENTAGE_SW_SU 30 /* at least 25% of the solar power usable must go to the battery */
#define SOLAX_SELFUSE_MIN_BATTERY_CHARGE_PERCENTAGE_SW_FS 20

#define SOLAX_PV_CUTOFF_VOLTAGE_V 90
#define SOLAX_PV_CUTOFF_TIMEOUT 2000
#define SOLAX_PV_SWITCHON_TIMEOUT 3000

#define WORKAROUND_SOLAX_INJECTION_SURGE
// not more than a switch into EPS mode per 10 minute, to avoid wearing the physical switch (not a SSR)
#define SOLAX_EPS_MODE_SWITCH_TIMEOUT_MS 600000 // 20 years durability for a 1M toggling cycles device
#define SOLAX_EPS_MODE_SWITCH_MIN_SOC 80 // won't switch EPS if not at least charged at 80% (normally)

#define DISPLAY_TIMEOUT 1000
// at least a CAN communication must have taken place within that period
#define TIMEOUT_LAST_ACTIVITY 10000

#define S2LE(buf, off) ((int16_t)((int16_t)((int16_t)((int16_t)(buf)[off+1])<<8l) | (int16_t)((int16_t)(buf)[off]&0xFFl) ))
#define U2LE(buf, off) ((((buf)[off+1]&0xFFu)<<8) | ((buf)[off]&0xFFu) )
#define U2BE(buf, off) ((((buf)[off]&0xFFu)<<8) | ((buf)[off+1]&0xFFu) )

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

enum bms_uart_state_e {
  BMS_UART_STATE_IDLE,
  BMS_UART_STATE_WAIT,
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
  solax_pw_queue[idx].cmd = cmd;
  solax_pw_queue[idx].cmd_len = cmd_len;
  solax_pw_queue[idx].rep_len = rep_len;
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
  uint8_t status;
  uint8_t work_mode;
  uint16_t bat_SoC;
  int16_t bat_temp;
  int16_t grid_wattage;
  int16_t grid_meter_ct;
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
} solax;
struct {
  uint16_t voltage;
  uint32_t precise_voltage;
  int16_t current;
  int32_t precise_current;
  uint16_t soc;
  int32_t wattage;
  int32_t precise_wattage;
  int16_t max_charge;
  int16_t max_discharge;
  uint16_t packs;
} pylontech;

void pv1_switch(uint32_t state) {
  // anode green led 
  // cathode of the optocoupler 1 = off => mosfet defaulty closed, 0 = on => mosfet open
  gpio_set(1, 0, state); // PB0 (led)
#ifndef BOARD_DEV
  gpio_set(3, 14, state); // PD14
#endif // BOARD_DEV
  solax.pv1_switch_on = state; 
}

void pv2_switch(uint32_t state) {
  // anode blue led 
  // cathode of the optocoupler 1 = off => mosfet defaulty closed, 0 = on => mosfet open
  gpio_set(1, 7, state); // PB7 (led)
#ifndef BOARD_DEV
  gpio_set(3, 15, state); // PD14
#endif // BOARD_DEV
  solax.pv2_switch_on = state;
}

void eps_mode_switch(uint32_t eps_mode_requested) {
  // switch the physical switch to force EPS mode (disconnect from the grid)
  // gpio = 1 => phototriac closed => contactor coil powered => grid contact are severed (Normally Closed)
  gpio_set(1, 14, eps_mode_requested); // PB14 led
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
  uint32_t bms_reconnect_at;
  uint32_t solax_pw_timeout=0;
#ifdef BMS_PING
  uint32_t bms_ping_timeout;
#endif // BMS_PING
  size_t len;
  uint32_t cid;
  size_t cid_bitlen;
  uint32_t enable_battery = 0;
  //uint32_t wait_bms_info = 1;
  //uint8_t bms_info[8];
#ifdef USART5_HUMAN_READABLE_SUMMARY_LOG
  uint32_t timeout_next_display = uwTick;
#endif // USART5_HUMAN_READABLE_SUMMARY_LOG
  uint32_t solax_pw_mode_change_ready;
#ifdef BMS_TIMEOUT_DETECT
  uint32_t pylontech_timeout = 0;
#endif // BMS_TIMEOUT_DETECT
  uint32_t last_CAN_activity_timeout = 0;
  uint32_t timeout_pv1_switch_on=0;
  uint32_t timeout_pv1_switch_off=0;
  pv1_switch(1);
  uint32_t timeout_pv2_switch_on=0;
  uint32_t timeout_pv2_switch_off=0;
  pv2_switch(1);
  uint32_t eps_mode_switch_timeout=0;

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

  // USART used for USBVCP communication
  Configure_USBVCP(USART_BAUDRATE_USBVCP);
  // Usart for BMS communication (UART6 PC6-TX PC7-RX)
  Configure_UARTBMS(115200);

  Configure_CAN1(500000);
  Configure_CAN3(500000);


  memset(&solax, 0, sizeof(solax));
  memset(&pylontech, 0, sizeof(pylontech));
  // init the queue
  memset(solax_pw_queue, 0, sizeof(solax_pw_queue));
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
  solax_pw_mode_change_ready = uwTick + SOLAX_PW_MODE_CHANGE_MIN_INTERVAL;

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
  tparse_init(&tp_bms, uart_bms_buffer, sizeof(uart_bms_buffer), " \n");

  master_log("Reset\n");

  // sent ping to the BMS to wake it up at reset moment
  can_bms_tx_log(0x1871, CAN_ID_EXTENDED_LEN, (uint8_t*)"\x01\x00\x01\x00\x00\x00\x00\x00", 8);
#ifdef BMS_PING
  bms_ping_timeout = uwTick + BMS_PING_INTERVAL_MS;
#endif // BMS_PING
  bms_reconnect_at = uwTick;
  last_CAN_activity_timeout = uwTick + TIMEOUT_LAST_ACTIVITY;
#ifdef BMS_TIMEOUT_DETECT
  pylontech_timeout = uwTick + PYLONTECH_ACTIVITY_TIMEOUT;
#endif // BMS_TIMEOUT_DETECT

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
    // reset in case of CAN activity
    if (uwTick - last_CAN_activity_timeout < 0x80000000UL) {
      NVIC_SystemReset();
    }

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
          last_CAN_activity_timeout = uwTick + TIMEOUT_LAST_ACTIVITY;
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

#ifdef BMS_TIMEOUT_DETECT
    // timeout the pylontech status to avoid "bad" display
    if (pylontech_timeout && uwTick - pylontech_timeout < 0x80000000UL ) {
      master_log("bms can timeout\n");
      memset(&pylontech, 0, sizeof(pylontech));
      pylontech_timeout = 0;
    }
#endif // BMS_TIMEOUT_DETECT

    if (can_fifo_avail(CAN3)) {
      len = can_fifo_rx(CAN3, &cid, &cid_bitlen, tmp, sizeof(tmp));
      master_log_can("bms >>>     | ", cid, cid_bitlen, tmp, len);
#ifdef BMS_TIMEOUT_DETECT
      pylontech_timeout = uwTick + PYLONTECH_ACTIVITY_TIMEOUT;
#endif // BMS_TIMEOUT_DETECT
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
        case 0x1878:
          forward = 1;
          break;

        case 0x1875:
          pylontech.packs = U2LE(tmp,2); 
          forward = 1;
          break;

        case 0x1877:
          // wipe BMS flags
          // memset(tmp, 0, 4); // optional, use them actually, in case of significant error to be notified to the inverter
          tmp[4] = BMS_KIND;
          tmp[5] = tmp[6] = tmp[7] = 0; // wipe versions
          // override message to tell the inverter of the battery configuration
          //memmove(tmp, "\x00\x00\x00\x00\x52\x00\x00\x00", 8); // OK 2H48050, OK 4 H48050p, OK 6 H48050, NOK 3/5/7/8 => seen as T58
          //memmove(tmp, "\x00\x00\x00\x00\x82\x00\x00\x00", 8); // NOK 8 H48050 // brand TP201
          memmove(tmp, "\x00\x00\x00\x00\x83\x00\x00\x00", 8); // OK 8 H48050 // brand TP202
          len = 8;
          forward = 1;
          // we've described the battery to the inverter, enable it now
          enable_battery = 1;
          break;

        // discarded
        case 0x1874:
        case 0x1876:
          break;
      }
      if (forward) {
        can_solax_tx_log(cid, cid_bitlen, tmp, len);
        last_CAN_activity_timeout = uwTick + TIMEOUT_LAST_ACTIVITY;
      }
    }

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
    */
    last_CAN_activity_timeout = uwTick + TIMEOUT_LAST_ACTIVITY;
    switch(bms_uart_state) {
      case BMS_UART_STATE_IDLE:
        // wait until transmit moment is reached
        if (uwTick - bms_uart_next >= 0x80000000UL) {
          break;
        }
        tparse_discard(&tp_bms);
        master_log("UARTBMS >> pwr\n");
        // send request to the bms
        uart_select_intf(USART6);
        uart_send_mem("pwr\n",4);
        bms_uart_state = BMS_UART_STATE_WAIT;
        bms_uart_next = uwTick + BMS_UART_NEXT_INTERVAL;
        bms_uart_timeout = uwTick + BMS_UART_TIMEOUT;
        break;
      case BMS_UART_STATE_WAIT:
        // line received?
        if (tparse_has_line(&tp_bms)) {
          master_log("UARTBMS line\n");

          // data line
          uint32_t val = tparse_token_u32(&tp_bms);
          // if it's the value line and not the text line
          if (val != -1UL) { 
            pylontech.precise_voltage = (int32_t)val;
            pylontech.precise_current = (int32_t)tparse_token_u32(&tp_bms);
            pylontech.precise_wattage = ((int32_t)pylontech.precise_current/(int32_t)10*(int32_t)pylontech.precise_voltage/(int32_t)10)/(int32_t)10000;
            snprintf(tmp, sizeof(tmp), "  voltage: %d\n  current: %d\n  wattage: %d\n", pylontech.precise_voltage, pylontech.precise_current, pylontech.precise_wattage);
            master_log(tmp);
            bms_uart_timeout = 0; // disable timeout
            bms_uart_state = BMS_UART_STATE_IDLE; // enable next request sending
          }
          tparse_discard_line(&tp_bms);
        }
        // timeout waiting for the command, reset the state and prepare a new command
        if ( bms_uart_timeout && (uwTick - bms_uart_timeout < 0x80000000UL)) {
          bms_uart_next = 0;
          bms_uart_state = BMS_UART_STATE_IDLE;
        }
        break;
    }

    if (timeout_pv1_switch_on && uwTick - timeout_pv1_switch_on < 0x80000000UL) {
      pv1_switch(1);
      master_log("pv1 switch on\n");
      timeout_pv1_switch_on = 0;
      timeout_pv1_switch_off = uwTick + SOLAX_PV_SWITCHON_TIMEOUT;
    }

    if (timeout_pv2_switch_on && uwTick - timeout_pv2_switch_on < 0x80000000UL) {
      pv2_switch(1); 
      master_log("pv2 switch on\n");
      timeout_pv2_switch_on = 0;
      timeout_pv2_switch_off = uwTick + SOLAX_PV_SWITCHON_TIMEOUT;
    }

#ifdef WORKAROUND_SOLAX_INJECTION_SURGE
    // likely the charge is coming to an end, avoid surges of the panels into the grid by ensuring offgrid mode
    if (pylontech.max_charge < pylontech.max_discharge 
      //&& pylontech.soc > SOLAX_EPS_MODE_SWITCH_MIN_SOC
      && uwTick - eps_mode_switch_timeout < 0x80000000UL) {
      eps_mode_switch(1);
      eps_mode_switch_timeout = uwTick + SOLAX_EPS_MODE_SWITCH_TIMEOUT_MS;
    }
    else if (uwTick - eps_mode_switch_timeout < 0x80000000UL) {
      eps_mode_switch(0);
      eps_mode_switch_timeout = uwTick + SOLAX_EPS_MODE_SWITCH_TIMEOUT_MS;
    }
#endif // WORKAROUND_SOLAX_INJECTION_SURGE

    // handle solax PocketWifi port to get the pv arrays status
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
          size_t read = tparse_read(&tp_solax_pw, tmp, solax_pw_queue[0].rep_len);
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
            solax.status      = tmp[25];
            solax.bat_wattage = S2LE(tmp, 37);
            solax.bat_temp = S2LE(tmp, 39);
            solax.bat_SoC = U2LE(tmp, 41);
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


            // if (!solax_checksum_verify(tmp+2,tmp[2]-2)) {
            //   batt_drain_fix_cause = 100;
            //   tparse_discard(&tp_solax_pw);
            //   solax_pw_state = SOLAX_PW_INVALID_NEXT;
            //   solax_pw_timeout = uwTick + SOLAX_PW_INVALID_RETRY_TIMEOUT;
            //   break;
            // }

            int32_t pylontech_wattage = pylontech.precise_wattage?pylontech.precise_wattage:pylontech.wattage;
            int32_t power_balance_w = solax.pv1_wattage + solax.pv2_wattage - (solax.grid_wattage + pylontech_wattage );
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
            if (solax.grid_wattage == 0 && solax.pv1_voltage == 0 && solax.pv2_voltage == 0 && solax.bat_wattage == 0 && solax.eps_voltage == 0) {
              batt_drain_fix_cause = 78;
              master_log("cause 78\n");
              goto invalid;
            }

            // only reset condition when a packet can be interpreted
            batt_forced_charge = -1;
            batt_forced_soc = -1;
            batt_drain_fix_cause = 0;


            // not NIGHT
            if (solax.pv1_voltage + solax.pv2_voltage > SOLAX_DAY_THRESHOLD_V*10 ) {

              // cutoff for the given duration
              if (
                (!timeout_pv1_switch_off || uwTick - timeout_pv1_switch_off < 0x80000000UL)
                && (solax.pv1_voltage < SOLAX_PV_CUTOFF_VOLTAGE_V*10 
                   || (solax.pv1_wattage && solax.pv1_wattage < SOLAX_PV_POWER_OPT_THRESHOLD_W))) {
                master_log("pv1 switch off\n");
                pv1_switch(0);
                timeout_pv1_switch_on = uwTick + SOLAX_PV_CUTOFF_TIMEOUT;
                timeout_pv1_switch_off = 0;
              }

              // cutoff for the given duration
              if (
                (!timeout_pv2_switch_off || uwTick - timeout_pv2_switch_off < 0x80000000UL)
                && ( solax.pv2_voltage < SOLAX_PV_CUTOFF_VOLTAGE_V*10 
                    || ( solax.pv2_wattage && solax.pv2_wattage < SOLAX_PV_POWER_OPT_THRESHOLD_W))) {
                master_log("pv2 switch off\n");
                pv2_switch(0);
                timeout_pv2_switch_on = uwTick + SOLAX_PV_CUTOFF_TIMEOUT;
                timeout_pv2_switch_off = 0;
              }

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
                batt_forced_charge = 0; // deny charge (which drains battery when not enough solar)
                //batt_forced_soc = 100;
                // batt_forced_soc = SOLAX_BAT_MIN_SOC_SELFUSE;
                if (solax_pw_queue_free() >= 2 
                  && solax_forced_work_mode != SOLAX_FORCED_WORK_MODE_MANUAL_STOP
                  && solax_pw_mode_change_ready == 0) {
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
                batt_forced_charge = 0; // deny charge
                // batt_forced_soc = 100;
                // batt_forced_soc = SOLAX_BAT_MIN_SOC_SELFUSE; // deny discharge for self-use

                if (solax_pw_queue_free() >= 2 
                  && solax_forced_work_mode != SOLAX_FORCED_WORK_MODE_MANUAL_STOP
                  && solax_pw_mode_change_ready == 0) {
                  solax_pw_queue_push(solax_pw_cmd_mode_manual, sizeof(solax_pw_cmd_mode_manual), 7);
                  solax_pw_queue_push(solax_pw_cfg_manual_stop, sizeof(solax_pw_cfg_manual_stop), 7);
                  solax_forced_work_mode = SOLAX_FORCED_WORK_MODE_MANUAL_STOP;
                  solax_pw_mode_change_ready = uwTick + SOLAX_PW_MODE_CHANGE_MIN_INTERVAL;
                }
              }
              else 
#endif // HAVE_LOWLIGHT_OPT
              // when battery is full, then for self use
              if (pylontech.soc >= 100) {
                batt_drain_fix_cause = 3;
                master_log("cause 3\n");
                batt_forced_charge = 0;
                // batt_forced_soc = 100;
                if (solax_forced_work_mode != SOLAX_FORCED_WORK_MODE_SELF_USE
                   && solax_pw_queue_free()
                   && solax_pw_mode_change_ready == 0) {
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
                  && solax_pw_mode_change_ready == 0) {
                  solax_pw_queue_push(solax_pw_cmd_mode_manual, sizeof(solax_pw_cmd_mode_manual), 7);
                  solax_pw_queue_push(solax_pw_cfg_manual_stop, sizeof(solax_pw_cfg_manual_stop), 7);
                  solax_forced_work_mode = SOLAX_FORCED_WORK_MODE_MANUAL_STOP;
                  solax_pw_mode_change_ready = uwTick + SOLAX_PW_MODE_CHANGE_MIN_INTERVAL;
                }
              }
              // NOTE: this is wrong when wattage total is lower (when the house consumption is not high)
#if 0
              else if ( 
                // grid tied
                solax.grid_wattage > SOLAX_GRID_EXPORT_OPT_THRESHOLD_W /* == SELFUSE*/ 
                // battery charge first stops when reading the BATT MIN SOC for selfuse
                //&& pylontech.soc <= SOLAX_SELFUSE_MIN_BATTERY_SOC
                // && solax.pv1_wattage + solax.pv2_wattage > SOLAX_SELF_CONSUMPTION_INVERTER_W
                // we're inverting everything to the self use, nothing left for battery charging :(
                // && (solax.grid_meter_ct > -SOLAX_GRID_EXPORT_OPT_THRESHOLD_W 
                //     || solax.grid_meter_ct < SOLAX_GRID_EXPORT_OPT_THRESHOLD_W )
                && ( 
                  // solax.pv1_wattage + solax.pv2_wattage 
                  //  < solax.grid_wattage
                  //    //+ SOLAX_PV_POWER_OPT_THRESHOLD_W 
                  //    //+ SOLAX_SELF_CONSUMPTION_MPPT_W + SOLAX_SELF_CONSUMPTION_INVERTER_W
                  //  || 
                  pylontech.current * pylontech.voltage / 10 / 10 
                    < SOLAX_SELFUSE_MIN_BATTERY_CHARGE_PERCENTAGE_SW_FS * (solax.pv1_wattage + solax.pv2_wattage) / 100
                   )
                // NOTE: if battery not charging fast (end of charge) then the condition is not respected, and 
                //       we don't stay in selfuse due to these conditions :(
                ) {
                // battery has nothing left to charge, discard grid 
                batt_drain_fix_cause = 5;
                master_log("cause 5\n");
                if (solax_pw_queue_free() >= 2 
                  && solax_forced_work_mode != SOLAX_FORCED_WORK_MODE_MANUAL_STOP
                  && solax_pw_mode_change_ready == 0) {
                  solax_pw_queue_push(solax_pw_cmd_mode_manual, sizeof(solax_pw_cmd_mode_manual), 7);
                  solax_pw_queue_push(solax_pw_cfg_manual_stop, sizeof(solax_pw_cfg_manual_stop), 7);
                  solax_forced_work_mode = SOLAX_FORCED_WORK_MODE_MANUAL_STOP;
                  solax_pw_mode_change_ready = uwTick + SOLAX_PW_MODE_CHANGE_MIN_INTERVAL;
                }
              }
#endif
              // when battery reached the requested charge level, enter self use
              else if (solax.pv1_wattage + solax.pv2_wattage 
                       /*> SOLAX_SELF_CONSUMPTION_MPPT_W + SOLAX_SELF_CONSUMPTION_INVERTER_W*/
                // CHARGE first strategy
                && ( (pylontech.soc > SOLAX_SELFUSE_MIN_BATTERY_SOC /* charged enough */

                     //  && -solax.grid_wattage > SOLAX_GRID_EXPORT_OPT_THRESHOLD_W /* NOT SELFUSE */
                     //  && -solax.grid_wattage < pylontech.current * pylontech.voltage / 10 / 10 // (in W)
                     //                                     - SOLAX_SELFUSE_MIN_BATTERY_CHARGE_PERCENTAGE_SW_SU * pylontech.current * pylontech.voltage / 10 / 10 / 100)
                     //     // or more power to be drained from panel (possibly) (and is needed because of grid import)
                     // || (-solax.grid_wattage > SOLAX_GRID_EXPORT_OPT_THRESHOLD_W
                     //     // if we charging above max charge minus 10%, then consider there is room for self-use
                     //     && pylontech.current >= pylontech.max_charge - 10*pylontech.max_charge/100  // (in 0.1A)
                     //     // and still power remaining for the battery (considering the import that will be compensated)
                     //     && -solax.grid_wattage < pylontech.current * pylontech.voltage / 10 / 10 // (in W)
                     //                                     - SOLAX_SELFUSE_MIN_BATTERY_CHARGE_PERCENTAGE_SW_SU * pylontech.current * pylontech.voltage / 10 / 10 / 100
                          )
                     // // || solar > 180w + (- grid export) => we have some spare to charge batteries
                       )
                     ) {
                // power the grid
                batt_drain_fix_cause = 6;
                master_log("cause 6\n");
                if (solax_pw_queue_free()
                  && solax_forced_work_mode != SOLAX_FORCED_WORK_MODE_SELF_USE
                  && solax_pw_mode_change_ready == 0) {
                  solax_pw_queue_push(solax_pw_cmd_mode_self_use, sizeof(solax_pw_cmd_mode_self_use), 7);
                  solax_forced_work_mode = SOLAX_FORCED_WORK_MODE_SELF_USE;
                  solax_pw_mode_change_ready = uwTick + SOLAX_PW_MODE_CHANGE_MIN_INTERVAL;
                }
              }
            }
            // NIGHT
            else {
                if (timeout_pv1_switch_on == 0) {
                batt_drain_fix_cause = 7;
                master_log("cause 7\n");
                //batt_forced_charge = 0;
                if (solax_pw_queue_free() 
                  && solax_forced_work_mode != SOLAX_FORCED_WORK_MODE_SELF_USE
                  && solax_pw_mode_change_ready == 0) {
                  solax_pw_queue_push(solax_pw_cmd_mode_self_use, sizeof(solax_pw_cmd_mode_self_use), 7);
                  solax_forced_work_mode = SOLAX_FORCED_WORK_MODE_SELF_USE;
                  solax_pw_mode_change_ready = uwTick + SOLAX_PW_MODE_CHANGE_MIN_INTERVAL;
                }
              }
            }
          }
          // whatever the reply, discard the data after this point
          solax_pw_state = SOLAX_PW_WAIT_NEXT; // switch state before possibly scheduling a command to send
          solax_pw_timeout = uwTick + SOLAX_PW_NEXT_TIMEOUT;
          solax_pw_queue_pop();
        }
        // timing out first entry if any
        else if (solax_pw_queue_free() != SOLAX_PW_QUEUE_SIZE && (uwTick - solax_pw_timeout < 0x80000000UL)) {
          master_log("UART TIMEOUT");
          master_log_hex(uart_pw_buffer, sizeof(uart_pw_buffer));
          solax_pw_state = SOLAX_PW_WAIT_NEXT;
          solax_pw_timeout = uwTick + SOLAX_PW_NEXT_TIMEOUT;
          tparse_reset(&tp_solax_pw);
          solax_pw_queue_pop();
          // discard causes due to timeout of the request
          batt_drain_fix_cause = 0;
          batt_forced_charge = -1;
          batt_forced_soc = -1;

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
        if (uwTick - solax_pw_timeout < 0x80000000UL || solax_pw_queue_free() != SOLAX_PW_QUEUE_SIZE) {
          solax_pw_state = SOLAX_PW_IDLE;
        }
        break;
    }

    if (uwTick - solax_pw_mode_change_ready < 0x80000000UL) {
      solax_pw_mode_change_ready = 0;
    }

#ifdef USART5_HUMAN_READABLE_SUMMARY_LOG
    // display (if no erroneous data)
    if (uwTick - timeout_next_display < 0x80000000UL && batt_drain_fix_cause < 70) {
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

uint8_t i2c_xfer_buffer[128];
uint32_t i2c_xfer_offset;
uint32_t i2c_xfer_length;
void I2C_Slave_Match_Callback(void) {
  //i2c_xfer_offset = i2c_xfer_length = 0;
}

void I2C_Slave_Reception_Callback(void) {
  i2c_xfer_offset = 0;

  switch(LL_I2C_ReceiveData8(I2CS)) {
  case 0:
    // read stats
    // don't process when an error has been detected, only ignore optimization rules (< 0x70)
    if (batt_drain_fix_cause < 70) 
    {
      i2c_xfer_length = 0; // wipe the previous buffer content
      // data encoding version
      i2c_xfer_length++; // total length, reserve space
      i2c_xfer_buffer[i2c_xfer_length++] = 1; 

      // solax state
      i2c_xfer_buffer[i2c_xfer_length++] = solax.status; 
      // solax work mode
      i2c_xfer_buffer[i2c_xfer_length++] = solax_forced_work_mode;

      // grid export wattage
      i2c_xfer_buffer[i2c_xfer_length++] = (solax.grid_meter_ct>>8)&0xFF;
      i2c_xfer_buffer[i2c_xfer_length++] = solax.grid_meter_ct&0xFF;
      // internal grid wattage
      i2c_xfer_buffer[i2c_xfer_length++] = (solax.grid_wattage>>8)&0xFF;
      i2c_xfer_buffer[i2c_xfer_length++] = solax.grid_wattage&0xFF;
      // pv1 wattage
      i2c_xfer_buffer[i2c_xfer_length++] = (solax.pv1_wattage>>8)&0xFF;
      i2c_xfer_buffer[i2c_xfer_length++] = solax.pv1_wattage&0xFF;
      // pv2 wattage
      i2c_xfer_buffer[i2c_xfer_length++] = (solax.pv2_wattage>>8)&0xFF;
      i2c_xfer_buffer[i2c_xfer_length++] = solax.pv2_wattage&0xFF;
      // bat wattage
      i2c_xfer_buffer[i2c_xfer_length++] = (solax.bat_wattage>>8)&0xFF;
      i2c_xfer_buffer[i2c_xfer_length++] = solax.bat_wattage&0xFF;
      // bat effective wattage (0.1A rounding if no precise wattage provided)
      int32_t pylontech_wattage = pylontech.precise_wattage?pylontech.precise_wattage:pylontech.wattage;
      i2c_xfer_buffer[i2c_xfer_length++] = (pylontech_wattage>>8)&0xFF;
      i2c_xfer_buffer[i2c_xfer_length++] = pylontech_wattage&0xFF;
      // bat soc
      i2c_xfer_buffer[i2c_xfer_length++] = (pylontech.soc>>8)&0xFF;
      i2c_xfer_buffer[i2c_xfer_length++] = pylontech.soc&0xFF;
      // bat max charge
      i2c_xfer_buffer[i2c_xfer_length++] = (pylontech.max_charge>>8)&0xFF;
      i2c_xfer_buffer[i2c_xfer_length++] = pylontech.max_charge&0xFF;
      // bat max discharge
      i2c_xfer_buffer[i2c_xfer_length++] = (pylontech.max_discharge>>8)&0xFF;
      i2c_xfer_buffer[i2c_xfer_length++] = pylontech.max_discharge&0xFF;
      // bat forced max charge
      i2c_xfer_buffer[i2c_xfer_length++] = (batt_forced_charge>>8)&0xFF;
      i2c_xfer_buffer[i2c_xfer_length++] = batt_forced_charge&0xFF;
      // last rule executed
      i2c_xfer_buffer[i2c_xfer_length++] = batt_drain_fix_cause;
      // solax work_mode
      i2c_xfer_buffer[i2c_xfer_length++] = solax.work_mode;
      // pv1 voltage
      i2c_xfer_buffer[i2c_xfer_length++] = (solax.pv1_voltage>>8)&0xFF;
      i2c_xfer_buffer[i2c_xfer_length++] = solax.pv1_voltage&0xFF;
      // pv2 voltage
      i2c_xfer_buffer[i2c_xfer_length++] = (solax.pv2_voltage>>8)&0xFF;
      i2c_xfer_buffer[i2c_xfer_length++] = solax.pv2_voltage&0xFF;
      // eps current
      i2c_xfer_buffer[i2c_xfer_length++] = (solax.eps_current>>8)&0xFF;
      i2c_xfer_buffer[i2c_xfer_length++] = solax.eps_current&0xFF;
      // timestamp
      i2c_xfer_buffer[i2c_xfer_length++] = solax.year>>8;
      i2c_xfer_buffer[i2c_xfer_length++] = solax.year&0xFF;
      i2c_xfer_buffer[i2c_xfer_length++] = solax.month;
      i2c_xfer_buffer[i2c_xfer_length++] = solax.day;
      i2c_xfer_buffer[i2c_xfer_length++] = solax.hour;
      i2c_xfer_buffer[i2c_xfer_length++] = solax.minute;
      // state of PV switch for low voltage cutoff (transient state)
      i2c_xfer_buffer[i2c_xfer_length++] = solax.pv1_switch_on;
      i2c_xfer_buffer[i2c_xfer_length++] = solax.pv2_switch_on;
      // eps power
      i2c_xfer_buffer[i2c_xfer_length++] = (solax.eps_power>>8)&0xFF;
      i2c_xfer_buffer[i2c_xfer_length++] = solax.eps_power&0xFF;
      // eps voltage
      i2c_xfer_buffer[i2c_xfer_length++] = (solax.eps_voltage>>8)&0xFF;
      i2c_xfer_buffer[i2c_xfer_length++] = solax.eps_voltage&0xFF;
      // encode total length
      i2c_xfer_buffer[0] = i2c_xfer_length;
    }
    // DESIGN NOTE: TXIS is raised right when a READ transaction match occurs
    break;
  // case 1:
  //   break;
  // case 2:
  //   break;
  }
}

void I2C_Slave_Ready_To_Transmit_Callback(void) {
  if (i2c_xfer_offset < i2c_xfer_length) {
    LL_I2C_TransmitData8(I2CS, i2c_xfer_buffer[i2c_xfer_offset++]);
  }
  else {
    // stuffing
    LL_I2C_TransmitData8(I2CS, 0xAA);
  }
}

void I2C_Slave_Complete_Callback(void) {
  //i2c_xfer_offset = i2c_xfer_length = 0;
}

void I2C_Error_Callback(void) {
  i2c_xfer_offset = i2c_xfer_length = 0;
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
  uint32_t timing = 0;

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
