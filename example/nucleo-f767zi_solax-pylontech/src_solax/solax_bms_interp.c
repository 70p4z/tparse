
#include "main.h"
#include "tparse.h"
#include "stddef.h"
#include "stdio.h"
#include "stdbool.h"
#include "globals.h"
#include "consts.h"

/*
TODO
[ ] global charge enable / disable to avoid charging during wrong hours => stay in tempo, that switch would have a timeout to reset itself and go back to auto enable upon low batt

*/

void Configure_I2C_Slave(void);
void solax_compute_maxcharge(void);

void update_external_charger(void);

// for snprintf to work as expected
void _sbrk(void) {

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

void can_inv_tx_log(uint32_t cid, size_t cid_bitlen, uint8_t* canmsg, size_t canmsg_len) {
  master_log_can("    >>> inv | ", cid, cid_bitlen, canmsg, canmsg_len);
  can_tx(CAN1, cid, cid_bitlen, canmsg, canmsg_len);
}

void can_bms_tx_log(uint32_t cid, size_t cid_bitlen, uint8_t* canmsg, size_t canmsg_len) {
  master_log_can("    >>> bms | ", cid, cid_bitlen, canmsg, canmsg_len);
  can_tx(CAN3, cid, cid_bitlen, canmsg, canmsg_len);
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

/*
GPIO=1 + EPS=230V -> (TRIAC=CLOSED) + EPS_relay(NO=COM) -> GRID_relay(OPENED) -> INV=|=GRID
GPIO=* + EPS=0V   -> (TRIAC=*) + EPS_relay(NC=COM) -> GRID_relay(CLOSED) -> INV===GRID
*/
void offgrid_switch(uint32_t eps_mode_requested) {
  // switch the physical switch to force EPS mode (disconnect from the grid)
  // gpio = 1 => phototriac closed => contactor coil powered => grid contact are severed (Normally Closed)
  gpio_set(1, 14, eps_mode_requested); // PB14 (red led)
#ifndef BOARD_DEV
  gpio_set(5, 12, eps_mode_requested); // PF12
#endif // BOARD_DEV
}

/**
 * Take care as bmu index pakc are reversed in the pylontech storage versus this gpios mapping.
 * Thanks pylontech for the reverse listing from farther of the link.
 */
struct {
  uint8_t gpio_port;
  uint8_t gpio_pin;
} const transcharge_gpios[] = {
  /* SW1 */ {/*GPIOB*/ 1, 13},
  /* SW2 */ {/*GPIOB*/ 1, 12},
  /* SW3 */ {/*GPIOA*/ 0, 15},
  /* SW4 */ {/*GPIOA*/ 0, 5},
  /* SW5 */ {/*GPIOA*/ 0, 6},
  /* SW6 */ {/*GPIOB*/ 1, 5},
  /* SW7 */ {/*GPIOA*/ 0, 4},
  /* SW8 */ {/*GPIOB*/ 1, 1},
  /* SW9 */ {/*GPIOC*/ 2, 2},
  /* SW10 */ {/*GPIOA*/ 0, 2},
  /* SW11 */ {/*GPIOA*/ 0, 7},
  /* SW12 */ {/*GPIOF*/ 6, 13},
  /* SW13 */ {/*GPIOE*/ 5, 9},
  /* SW14 */ {/*GPIOE*/ 5, 11},
  /* SW15 */ {/*GPIOE*/ 5, 13},
  /* SW16 */ {/*GPIOG*/ 6, 14},
};

#define TRANSCHARGE_MANUAL_TIMEOUT_MS (1800*1000) // half an hour max manual charge
#define TRANSCHARGE_AUTO_TIMEOUT_MS (900*1000) // 15 minutes timeout for auto
#define TRANSCHARGE_BALANCING_START_MV 10
#define TRANSCHARGE_BALANCING_MIN_GAP_MV 5 // when to charge another pack
#define TRANSCHARGE_BALANCING_START_MAX_WATTAGE 400
#define TRANSCHARGE_INTERVAL_MS (30*1000) // not too often to avoid starting/stopping charge too often
struct {
  uint32_t auto_enable;
  uint32_t enabled;
  uint32_t timeout;
  uint32_t auto_next_run;
  uint32_t auto_last_idx;
} transcharge;

void transcharge_enable(uint32_t index, uint32_t enabled) {
  // invalid channel
  if (index >= sizeof(transcharge_gpios) / sizeof(transcharge_gpios[0])) {
    return;
  }
  if (enabled) {
    transcharge.enabled |= (1<<index);
  }
  else {
    transcharge.enabled &= ~(1<<index);
  }
  gpio_set(transcharge_gpios[index].gpio_port, transcharge_gpios[index].gpio_pin, enabled);
}

void transcharge_disable_all(void) {
  for (int i = 0 ; i < sizeof(transcharge_gpios) / sizeof(transcharge_gpios[0]); i++) {
    transcharge_enable(i, 0);
  }
  transcharge.timeout = 0;
  transcharge.auto_last_idx = -1;
}

// naive balancing by charge algorithm
// must be run only when pylontech data are valid
void transcharge_auto_run(void) {
  if (transcharge.auto_enable && EXPIRED(transcharge.auto_next_run)) {
    uint32_t vcellmin_mv = -1;
    uint32_t vcellmin_mv_idx=pylontech.bmu_idx;
    uint32_t vcellmax_mv = 0;
    //uint32_t vcellmax_mv_idx=pylontech.bmu_idx;
    if (pylontech.bmu_idx > 0) {
      int32_t pylontech_wattage = pylontech.precise_wattage?pylontech.precise_wattage:pylontech.wattage;
      // don't start charge balancing when wattage is too high, this will not be working well
      if (pylontech_wattage > TRANSCHARGE_BALANCING_START_MAX_WATTAGE 
        || pylontech_wattage < -TRANSCHARGE_BALANCING_START_MAX_WATTAGE) {
        master_log("BALANCE: skipped, too much charge wattage already\n");
        transcharge.auto_next_run = EXPIRE_IN(TRANSCHARGE_INTERVAL_MS);
        transcharge_disable_all();
        return;
      }
      // retrieve min and max voltage along the cell voltage readings
      uint32_t pylon_idx = pylontech.bmu_idx;
      uint32_t transcharge_idx = 0; // convert index into transcharge index
      while (pylon_idx-- > 0) {
        // only perform balancing based on lowest voltage cell of each pack
        uint32_t v = pylontech.bmu[pylon_idx].vlow;
        if (vcellmin_mv > v
          && VCELL_VALID(v)
          && VCELL_VALID(pylontech.bmu[pylon_idx].vhigh)
          // don't select this pack when its top cell is already too high, wait for internal balancing
          && pylontech.bmu[pylon_idx].vhigh < PYLONTECH_BALANCING_MAX_MV) {
          vcellmin_mv = v;
          vcellmin_mv_idx = transcharge_idx;
        }
        if (vcellmax_mv < v
          && VCELL_VALID(v)) {
          vcellmax_mv = v;
          //vcellmax_mv_idx = transcharge_idx;
        }
        transcharge_idx++;
      }

      transcharge.auto_next_run = EXPIRE_IN(TRANSCHARGE_INTERVAL_MS);

      // Safety, disable all charge balancing due to over charge
      if (transcharge.auto_last_idx < pylontech.bmu_idx
        && VCELL_VALID(pylontech.bmu[transcharge.auto_last_idx].vhigh)
        && pylontech.bmu[transcharge.auto_last_idx].vhigh >= PYLONTECH_BALANCING_MAX_MV) {
        master_log("BALANCE: stop overcharge");
        transcharge_disable_all();
      }

      // if there's at least one pack requiring balancing, then charge it
      if (vcellmin_mv + TRANSCHARGE_BALANCING_START_MV < vcellmax_mv
        // a pack has been selected. else disable possibly started balancing
        && vcellmin_mv_idx != pylontech.bmu_idx ) {

        // Continue charging the same pack until its vcellmin is XmV above the current cellmin
        if (transcharge.auto_last_idx < pylontech.bmu_idx
          && vcellmin_mv_idx != transcharge.auto_last_idx
          && VCELL_VALID(pylontech.bmu[transcharge.auto_last_idx].vlow)
          && vcellmin_mv + TRANSCHARGE_BALANCING_MIN_GAP_MV < pylontech.bmu[transcharge.auto_last_idx].vlow) {
          transcharge.timeout = EXPIRE_IN(TRANSCHARGE_AUTO_TIMEOUT_MS);
          master_log("BALANCE: not charged enough, continue charging the same pack\n");
        }

        // is this pack already being balanced? or not?
        else if ((transcharge.enabled & (1<<vcellmin_mv_idx)) == 0) {
          // disable all other pack balancing (only one at a time)
          transcharge_disable_all();
          // balance the pack with the lowest cell voltage only
          transcharge_enable(vcellmin_mv_idx, 1);
          transcharge.auto_last_idx = vcellmin_mv_idx;
          transcharge.timeout = EXPIRE_IN(TRANSCHARGE_AUTO_TIMEOUT_MS);
          master_log("BALANCE: charge pack 0x");
          master_log_hex(&vcellmin_mv_idx, 1);
          master_log("\n");
        }
        else {
          transcharge.timeout = EXPIRE_IN(TRANSCHARGE_AUTO_TIMEOUT_MS);
          master_log("BALANCE: no change\n");
        }
      }
      // no more pack requiring balancing, disable all
      else {
        master_log("BALANCE: disable all pack charging\n");
        transcharge_disable_all();
      }
    }
  }
}


void interp(void) {
  uint32_t forward;
  uint8_t candata[8];

#ifdef SUPPORT_PYLONTECH_RECONNECT
  uint32_t bms_reconnect_at;
#endif // SUPPORT_PYLONTECH_RECONNECT  
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
  uint32_t pylontech_timeout = 0;
  uint32_t last_INV_CAN_activity_timeout = 0;
  uint32_t last_BMS_CAN_activity_timeout = 0;

  memset(&knobs, 0, sizeof(knobs));
  memset(&inverter, 0, sizeof(inverter));
  memset(&pylontech, 0, sizeof(pylontech));
#ifdef HAVE_EXT_CHARGER
  uint32_t charger_com_next_ms = 1;
  memset(&charger, 0, sizeof(charger));
#endif // HAVE_EXT_CHARGER
  memset(&pylontech_pid, 0, sizeof(pylontech_pid));
  pylontech_pid.kp_x100 = 30;
  pylontech_pid.ki_up_x100 = 2;
  pylontech_pid.ki_down_x100 = 2;
  pylontech_pid.kd_x100 = 5;
  pylontech_pid.max_step_up_dA = 10; // max change per cycle
  pylontech_pid.max_step_down_dA = 20; // max change per cycle (faster on drops)
  pylontech_pid.max_energy_step_dA = 20;
  pylontech_pid.energy_deadband_dA = 2;
  pylontech_pid.min_current_offset_dA = 1;
  pylontech_pid.inverter_offset_dA = SOLAX_BATTERY_CHARGE_OFFSET_DA;

  memset(&charger_pid, 0, sizeof(charger_pid));
  charger_pid.kp_x100 = 100;
  charger_pid.ki_up_x100 = 10;
  charger_pid.ki_down_x100 = 20;
  charger_pid.kd_x100 = 10;
  charger_pid.max_step_up_dA = 10; // max change per cycle
  charger_pid.max_step_down_dA = 20; // max change per cycle (faster on drops)
  charger_pid.max_energy_step_dA = 20;
  charger_pid.energy_deadband_dA = 2;
    // allow to respect the target as a measured value, not as a returned max allowed value
  charger_pid.compensate_measure = 1;


  // automatic state switching and eps disconnect mode by default
  auto_self_use_from_bat = 1;
  auto_grid_connection = 1;
  auto_bat_charge = 1;
  inverter.grid_connect_soc = GRID_CONNECT_SOC;
  inverter.grid_disconnect_soc = GRID_DISCONNECT_SOC;
  knobs.max_charge_voltage = BMS_MAX_CELL_VOLTAGE_FOR_CURRENT_CHG_DV;
  knobs.max_pylontech_charge_drive = BMS_MAX_CELL_VOLTAGE_FOR_PYLONTECH_DRIVE_DV;
  knobs.cell_voltage_limited_charge = BMS_CELL_VOLTAGE_FOR_LIMITED_CHARGE_DV;
  knobs.limited_charge_wattage = -1;
  knobs.max_charge_temperature = BMS_MAX_CHARGE_TEMPERATURE_DC;
#ifdef HAVE_EXT_CHARGER
  knobs.charger_stop_soc = 50;
  knobs.charger_start_soc = 25;
  knobs.allowed_charge_wattage = 1000;
#endif //HAVE_EXT_CHARGER
  // default is transcharge disabled
  transcharge.auto_enable = 0;
  transcharge.auto_next_run = EXPIRE_IN(0); // ensure immediate run
  transcharge.enabled = 0;
  transcharge.timeout = 0;
  transcharge_disable_all();

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

  // init the queue
  inverter_uart_init();
  bms_uart_init();
  pylontech_cache_clear();

  // USART used for USBVCP communication
  Configure_USBVCP(USART_BAUDRATE_USBVCP);
  // Usart for BMS communication (UART6 PC6-TX PC7-RX)
  Configure_UARTBMS(115200);

  Configure_CAN1(500000);
  Configure_CAN3(500000);

  //solax_pw_queue_push(solax_pw_cmd_change_bitrate, sizeof(solax_pw_cmd_change_bitrate), 7);
  inverter_uart_force_bitrate();

  // ensure starting with SELF USE mode
  solax_pw_mode_self_use();

  Configure_I2C_Slave();

#ifdef MODE_FAKE_SOLAX
  while (1) {
    can_inv_tx_log(0x1871, CAN_ID_EXTENDED_LEN, (uint8_t*)"\x01\x00\x01\x00\x00\x00\x00\x00", 8);
    LL_mDelay(BMS_PING_INTERVAL_MS);
  }
#endif // MODE_FAKE_SOLAX

  master_log("Reset\n");

  // // sent ping to the BMS to wake it up at reset moment
  // can_bms_tx_log(0x1871, CAN_ID_EXTENDED_LEN, (uint8_t*)"\x01\x00\x01\x00\x00\x00\x00\x00", 8);

  //can_inv_tx_log(0x0100A001, CAN_ID_EXTENDED_LEN, NULL, 0);
  //can_inv_tx_log(0x1801, CAN_ID_EXTENDED_LEN, (uint8_t*)"\x01\x00\x01\x00\x00\x00\x00\x00", 8);

#ifdef BMS_PING
  bms_ping_timeout = EXPIRE_IN(BMS_PING_INTERVAL_MS);
#endif // BMS_PING
#ifdef SUPPORT_PYLONTECH_RECONNECT  
  bms_reconnect_at = EXPIRE_IN(0);
#endif // SUPPORT_PYLONTECH_RECONNECT
  last_INV_CAN_activity_timeout = EXPIRE_IN(TIMEOUT_LAST_ACTIVITY);
  last_BMS_CAN_activity_timeout = EXPIRE_IN(TIMEOUT_LAST_ACTIVITY);
  pylontech_timeout = EXPIRE_IN(PYLONTECH_REPLY_TIMEOUT);
#ifdef USART5_HUMAN_READABLE_SUMMARY_LOG
  timeout_next_display = EXPIRE_IN(0);
#endif // USART5_HUMAN_READABLE_SUMMARY_LOG

  // startup the watchdog
  /* Enable the peripheral clock of DBG register (uncomment for debug purpose) */
  /* ------------------------------------------------------------------------- */
  /*  LL_DBGMCU_APB1_GRP1_FreezePeriph(LL_DBGMCU_APB1_GRP1_IWDG_STOP); */
  
#ifdef HAVE_WATCHDOG
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
#endif // HAVE_WATCHDOG

  while (1) {

#ifdef HAVE_WATCHDOG
    /* Refresh IWDG down-counter to default value */
    LL_IWDG_ReloadCounter(IWDG);
#endif // HAVE_WATCHDOG
    // reset in case of CAN activity timeout
    if (last_INV_CAN_activity_timeout && EXPIRED(last_INV_CAN_activity_timeout)) {
      master_log("INV ACTIVITY TIMEOUT\n");
      NVIC_SystemReset();
    }
    if (last_BMS_CAN_activity_timeout && EXPIRED(last_BMS_CAN_activity_timeout)) {
      master_log("CAN ACTIVITY TIMEOUT\n");
      NVIC_SystemReset();
    }

    // check for messages from the inverter
    if (can_fifo_avail(CAN1)) {
      cid = 0;
      forward = 0;
      last_INV_CAN_activity_timeout = EXPIRE_IN(TIMEOUT_LAST_ACTIVITY); // we've received something
      len = can_fifo_rx(CAN1, &cid, &cid_bitlen, candata, sizeof(candata));
      master_log_can("inv >>>     | ", cid, cid_bitlen, candata, len);

      forward = can_inv_interp(cid, cid_bitlen, candata, len);

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
          pylontech_timeout = EXPIRE_IN(PYLONTECH_REPLY_TIMEOUT);
        }
#ifdef SUPPORT_PYLONTECH_RECONNECT
        // make sure to avoid overflow when no reconnection request for a while
        bms_reconnect_at = EXPIRE_IN(0); 
#endif // SUPPORT_PYLONTECH_RECONNECT
        can_bms_tx_log(cid, cid_bitlen, candata, len);
      }
    }

#ifdef BMS_PING
    // check for messages from the bms
     // time for a ping
    if (EXPIRED(bms_ping_timeout)) {
      can_bms_tx_log(0x1871, CAN_ID_EXTENDED_LEN, (uint8_t*)"\x01\x00\x01\x00\x00\x00\x00\x00", 8);
      //can_inv_tx_log(0x1801, CAN_ID_EXTENDED_LEN, (uint8_t*)"\x01\x00\x01\x00\x00\x00\x00\x00", 8);
      bms_ping_timeout = EXPIRE_IN(BMS_PING_INTERVAL_MS);
    }
#endif // BMS_PING

#ifdef HAVE_EXT_CHARGER
    if (charger_com_next_ms && EXPIRED(charger_com_next_ms)) {
      // Message 1 (0x1806E5F4) /!\ BIG ENDIAN
      memset(candata, 0, 8);
      // only put values when charger is enabled, we're never too precautious
      if (charger.charge_enabled) {
        candata[0] = charger.max_charge_voltage>>8;
        candata[1] = charger.max_charge_voltage&0xFF;
        candata[2] = charger.max_charge_current>>8;
        candata[3] = charger.max_charge_current&0xFF;
      }
      candata[4] = charger.charge_enabled?0:1; // inverted charge request
      can_bms_tx_log(0x1806E5F4, CAN_ID_EXTENDED_LEN, candata, 8);
      charger_com_next_ms = EXPIRE_IN(CHARGER_COM_INTERVAL_MS);
    }
#endif // HAVE_EXT_CHARGER

    // pylontech timeout, serve the cached infos
    if (pylontech_timeout && EXPIRED(pylontech_timeout)) {
      master_log("BMS CAN TIMEOUT\n");
      // avoid retriggering timeout
      pylontech_timeout = 0;
      // reply with cache infos
      for (uint8_t i=0; i< PYLONTECH_CACHE_COUNT; i++) {
        if (pylontech_cached_infos[i].used) {
          can_inv_tx_log(pylontech_cached_infos[i].can_id, CAN_ID_EXTENDED_LEN, 
                           pylontech_cached_infos[i].can_msg, 8);
        }
      }
    }

    // take into account transcharge timeout
    if (transcharge.timeout && EXPIRED(transcharge.timeout)) {
      master_log("BALANCE TIMEOUT\n");
      transcharge_disable_all();
    }

    // perform transcharge auto algorithm only when all data from BMS are valid
    if (pylontech.bmu_idx > 0 && transcharge.auto_enable && pylontech.tcellmax < knobs.max_charge_temperature) {
      transcharge_auto_run();
    }

    if (can_fifo_avail(CAN3)) {
      cid = 0;
      forward = 0;
      len = can_fifo_rx(CAN3, &cid, &cid_bitlen, candata, sizeof(candata));
      master_log_can("bms >>>     | ", cid, cid_bitlen, candata, len);
      tmp[16]=0; // EOS for human readable log

#ifdef HAVE_EXT_CHARGER
      // regular communication from the charger
      if (cid == 0x18FF50E5) {
        charger.out_voltage = U2BE(candata, 0); // in dV
        charger.out_current = U2BE(candata, 2); // in dA
        charger.status = candata[4];

        // bms >>>     | 0x18ff50e5 e 00040000000f0036

        snprintf((char*)tmp+16, sizeof(tmp)-16, "            | OBC Vbat=%d.%dV\tIbat=%d.%dA\tstatus=%s%s%s%s%s\n", 
                 charger.out_voltage/10,charger.out_voltage%10,
                 charger.out_current/10,charger.out_current%10,
                 charger.status&1?"HWERR,":"",
                 charger.status&2?"OVTEMP,":"",
                 charger.status&4?"ACERR,":"",
                 charger.status&8?"SWERR,":"",
                 charger.status&16?"TIMEOUT,":"");
        break;
      }
      else 
#endif // HAVE_EXT_CHARGER
      // the BMS replied
      {
        pylontech_timeout = 0;
        last_BMS_CAN_activity_timeout = EXPIRE_IN(TIMEOUT_LAST_ACTIVITY);
      }

      forward = can_bms_interp(cid, cid_bitlen, candata, len);

      if (forward) {
        pylontech_cache_set(cid, candata);
        can_inv_tx_log(cid, cid_bitlen, candata, len);
      }
      // log decoded message content
      master_log((char*)tmp+16);
    }

    bms_uart_update();

    inverter_uart_update();

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
      if (inverter.valid_data && pylontech.soc != 0 && pylontech.soc != 255) 
      {
        i2c_xfer_r_length = 0; // wipe the previous buffer content
        // data encoding version
        i2c_xfer_r_length++; // total length, reserve space

        // schema version for incompatibility checks on the host side
        i2c_xfer_buffer[i2c_xfer_r_length++] = 7; 

        // solax state
        i2c_xfer_buffer[i2c_xfer_r_length++] = inverter.status; 
        // grid export wattage
        i2c_xfer_buffer[i2c_xfer_r_length++] = (inverter.grid_meter_ct>>8)&0xFF;
        i2c_xfer_buffer[i2c_xfer_r_length++] = inverter.grid_meter_ct&0xFF;
        // internal grid wattage
        i2c_xfer_buffer[i2c_xfer_r_length++] = (inverter.grid_wattage>>8)&0xFF;
        i2c_xfer_buffer[i2c_xfer_r_length++] = inverter.grid_wattage&0xFF;
        // eps power
        i2c_xfer_buffer[i2c_xfer_r_length++] = (inverter.eps_power>>8)&0xFF;
        i2c_xfer_buffer[i2c_xfer_r_length++] = inverter.eps_power&0xFF;
        /*
        // eps current
        i2c_xfer_buffer[i2c_xfer_r_length++] = (inverter.eps_current>>8)&0xFF;
        i2c_xfer_buffer[i2c_xfer_r_length++] = inverter.eps_current&0xFF;
        // eps voltage
        i2c_xfer_buffer[i2c_xfer_r_length++] = (inverter.eps_voltage>>8)&0xFF;
        i2c_xfer_buffer[i2c_xfer_r_length++] = inverter.eps_voltage&0xFF;
        */
        // pv1 wattage
        i2c_xfer_buffer[i2c_xfer_r_length++] = (inverter.pv1_wattage>>8)&0xFF;
        i2c_xfer_buffer[i2c_xfer_r_length++] = inverter.pv1_wattage&0xFF;
        // pv2 wattage
        i2c_xfer_buffer[i2c_xfer_r_length++] = (inverter.pv2_wattage>>8)&0xFF;
        i2c_xfer_buffer[i2c_xfer_r_length++] = inverter.pv2_wattage&0xFF;
        // pv1 voltage
        i2c_xfer_buffer[i2c_xfer_r_length++] = (inverter.pv1_voltage>>8)&0xFF;
        i2c_xfer_buffer[i2c_xfer_r_length++] = inverter.pv1_voltage&0xFF;
        // pv2 voltage
        i2c_xfer_buffer[i2c_xfer_r_length++] = (inverter.pv2_voltage>>8)&0xFF;
        i2c_xfer_buffer[i2c_xfer_r_length++] = inverter.pv2_voltage&0xFF;
        // bat wattage
        i2c_xfer_buffer[i2c_xfer_r_length++] = (inverter.bat_wattage>>8)&0xFF;
        i2c_xfer_buffer[i2c_xfer_r_length++] = inverter.bat_wattage&0xFF;
        // bat effective wattage (0.1A rounding if no precise wattage provided)
        int32_t pylontech_wattage = pylontech.precise_wattage?pylontech.precise_wattage:pylontech.wattage;
        i2c_xfer_buffer[i2c_xfer_r_length++] = (pylontech_wattage>>8)&0xFF;
        i2c_xfer_buffer[i2c_xfer_r_length++] = pylontech_wattage&0xFF;
        // bat soc
        i2c_xfer_buffer[i2c_xfer_r_length++] = pylontech.soc;
        // bat max charge
        i2c_xfer_buffer[i2c_xfer_r_length++] = (pylontech.max_charge_dA>>8)&0xFF;
        i2c_xfer_buffer[i2c_xfer_r_length++] = pylontech.max_charge_dA&0xFF;
        // bat max discharge
        i2c_xfer_buffer[i2c_xfer_r_length++] = (pylontech.max_discharge_dA>>8)&0xFF;
        i2c_xfer_buffer[i2c_xfer_r_length++] = pylontech.max_discharge_dA&0xFF;
        // effective max charge (taking into account forced charge and workaround for battery full)
        i2c_xfer_buffer[i2c_xfer_r_length++] = (pylontech.effective_charge_dA>>8)&0xFF;
        i2c_xfer_buffer[i2c_xfer_r_length++] = pylontech.effective_charge_dA&0xFF;
        // timestamp
        i2c_xfer_buffer[i2c_xfer_r_length++] = inverter.year>>8;
        i2c_xfer_buffer[i2c_xfer_r_length++] = inverter.year&0xFF;
        i2c_xfer_buffer[i2c_xfer_r_length++] = inverter.month;
        i2c_xfer_buffer[i2c_xfer_r_length++] = inverter.day;
        i2c_xfer_buffer[i2c_xfer_r_length++] = inverter.hour;
        i2c_xfer_buffer[i2c_xfer_r_length++] = inverter.minute;

        // time since restart
        U4BE_ENCODE(i2c_xfer_buffer, i2c_xfer_r_length, uwTick);
        i2c_xfer_r_length+=4;
        // output VA
        i2c_xfer_buffer[i2c_xfer_r_length++] = (inverter.output_va>>8)&0xFF;
        i2c_xfer_buffer[i2c_xfer_r_length++] = inverter.output_va&0xFF;
        // output auto switches
        i2c_xfer_buffer[i2c_xfer_r_length++] = auto_self_use_from_bat;
        i2c_xfer_buffer[i2c_xfer_r_length++] = auto_grid_connection;
        i2c_xfer_buffer[i2c_xfer_r_length++] = pylontech.apparent_soc;
        U4BE_ENCODE(i2c_xfer_buffer, i2c_xfer_r_length, pylontech.precise_mAh);
        i2c_xfer_r_length+=4;
        U4BE_ENCODE(i2c_xfer_buffer, i2c_xfer_r_length, pylontech.precise_mWh);
        i2c_xfer_r_length+=4;
        i2c_xfer_buffer[i2c_xfer_r_length++] = pylontech.soc_mWh;
        i2c_xfer_buffer[i2c_xfer_r_length++] = knobs.max_charge_voltage;
        i2c_xfer_buffer[i2c_xfer_r_length++] = knobs.forced_soc;
        // in deca Watt, to allow for 0-1.2kW span precision is not at the watt :)
        i2c_xfer_buffer[i2c_xfer_r_length++] = (knobs.forced_wattage/10)&0xFF;
        i2c_xfer_buffer[i2c_xfer_r_length++] = auto_bat_charge;
#ifdef HAVE_EXT_CHARGER
        i2c_xfer_buffer[i2c_xfer_r_length++] = knobs.allowed_charge_wattage>>8;
        i2c_xfer_buffer[i2c_xfer_r_length++] = knobs.allowed_charge_wattage&0xFF;
#else // HAVE_EXT_CHARGER
        i2c_xfer_buffer[i2c_xfer_r_length++] = 0;
        i2c_xfer_buffer[i2c_xfer_r_length++] = 0;
#endif // HAVE_EXT_CHARGER
        i2c_xfer_buffer[i2c_xfer_r_length++] = (transcharge.auto_enable?1:0);
        i2c_xfer_buffer[i2c_xfer_r_length++] = transcharge.enabled;
        i2c_xfer_buffer[i2c_xfer_r_length++] = (knobs.cell_voltage_limited_charge)&0xFF;
        i2c_xfer_buffer[i2c_xfer_r_length++] = (knobs.limited_charge_wattage/10)&0xFF;
        i2c_xfer_buffer[i2c_xfer_r_length++] = charger.out_current; // active current on external charger link
        i2c_xfer_buffer[i2c_xfer_r_length++] = charger.max_charge_current;
        
        // BMS units are listed backward (for the farther Link to the closest link)
        int i = pylontech.bmu_idx;
        while (i-- && i2c_xfer_r_length+8 < sizeof(i2c_xfer_buffer)) {
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
      solax_pw_gmppt1_off();
      break;
    case 2:
      master_log("I2C: pv1 gmppt high\n");
      solax_pw_gmppt1_high();
      break;
    case 3:
      master_log("I2C: pv2 gmppt off\n");
      // perform gmppt wakeup
      solax_pw_gmppt2_off();
      break;
    case 4:
      master_log("I2C: pv2 gmppt high\n");
      solax_pw_gmppt2_high();
      break;

    case 0xA:
      master_log("I2C: force offgrid\n");
      // disallow gridtie
      auto_grid_connection = 0;
      offgrid_switch(1);
      break;
    case 0xB:
      master_log("I2C: auto grid connection\n");
      // allow gridtie
      auto_grid_connection = 1;
      break;
    case 0xC:
      master_log("I2C: force gridtie\n");
      // force gridtie
      auto_grid_connection = 2;
      offgrid_switch(0);
      break;

    case 0xD:
      master_log("I2C: stop self use from batt\n");
      auto_self_use_from_bat = 0;
      // TODO: SOC to be read from Solax parameters instead
      pylontech.apparent_soc = MAX(0,20-1); 
      break;
    case 0xE:
      master_log("I2C: force self use from batt\n");
      auto_self_use_from_bat = 2;
      pylontech.apparent_soc = 75; /*random okish soc to ensure charging is possible*/
      break;
    case 0xF:
      master_log("I2C: auto self use from batt\n");
      auto_self_use_from_bat = 1;
      break;

      // change mode SELFUSE/STOP/FORCECHARGE

#ifdef HAVE_EXT_CHARGER
    case 0x20:
      master_log("I2C: auto bat charge\n");
      auto_bat_charge = 1;
      break;

    case 0x21:
      master_log("I2C: stop bat charge\n");
      auto_bat_charge = 0;
      charger.charge_enabled = 0;
      charger.max_charge_voltage = 0;
      charger.max_charge_current = 0;
      break;

    case 0x22:
      master_log("I2C: force bat charge\n");
      auto_bat_charge = 0;
      charger.charge_enabled = 1;
      break;
#endif // HAVE_EXT_CHARGER

    case 0x30:
      transcharge.auto_enable = 0;
      transcharge_disable_all();
      break;

    case 0x31:
      transcharge.auto_enable = 1;
      transcharge.auto_next_run = uwTick;
      break;
    }
  }
  else {
    // double bytes instructions
    if (i2c_xfer_w_length == 2) {
      switch(i2c_xfer_buffer[0]) {
      /*
      case 0x10:
        inverter.grid_connect_soc = i2c_xfer_buffer[1];
        break;
      case 0x11:
        inverter.grid_disconnect_soc = i2c_xfer_buffer[1];
        break;
      case 0x12:
        pylontech.max_charge_soc = i2c_xfer_buffer[1];
        break;
        // force charge (to balance batteries)
      case 0x14:
        batt_forced_charge = i2c_xfer_buffer[1]; // in dA
        break;
        */
      case 0x17:
        knobs.max_charge_voltage = i2c_xfer_buffer[1]; // id dV
        break;
      case 0x18:
        // if out of [0:100] then disabled the forced soc
        knobs.forced_soc = i2c_xfer_buffer[1]>100?0:i2c_xfer_buffer[1]; // reported SoC
        break;
      case 0x1A:
        knobs.forced_wattage = i2c_xfer_buffer[1]*10; // in daW (deca watt)
        break;
      case 0x1B:
        knobs.cell_voltage_limited_charge = i2c_xfer_buffer[1]; // in dV
        break;
      case 0x1C:
        knobs.limited_charge_wattage = i2c_xfer_buffer[1]*10; // in daW (deca watt)
        break;
#ifdef HAVE_EXT_CHARGER
      case 0x23:
        knobs.allowed_charge_wattage = i2c_xfer_buffer[1]*100; // in hecto watt
        break;
#endif // HAVE_EXT_CHARGER

      case 0x32: // transcharge manual enable
        // disable pack transcharge balancing
        transcharge.auto_enable = 0;
        transcharge_enable(i2c_xfer_buffer[1], 1);
        transcharge.timeout = EXPIRE_IN(TRANSCHARGE_MANUAL_TIMEOUT_MS);
        break; 
      case 0x33: // transchagre manual disable
        // disable pack transcharge balancing
        transcharge.auto_enable = 0;
        transcharge_enable(i2c_xfer_buffer[1], 0);
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
  LL_I2C_ClearFlag_ARLO(I2CS);
  LL_I2C_ClearFlag_BERR(I2CS);
  LL_I2C_ClearFlag_OVR(I2CS);
  LL_I2C_ClearSMBusFlag_TIMEOUT(I2CS);
  LL_I2C_ClearSMBusFlag_ALERT(I2CS);
  LL_I2C_ClearSMBusFlag_PECERR(I2CS);

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
  //LL_I2C_EnableIT_ERR(I2CS);
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

void solax_process_data(void) {
  if (!inverter.valid_data) {
    return;
  }

////////////////////////////////////////////////////////////////////////////////////////////////////*/
///                                                                                                 */
///        ▄▄▄▄   ▄▄▄▄▄▄     ▄▄▄▄▄▄   ▄▄▄▄▄                  ▄▄▄▄     ▄▄▄▄    ▄▄▄   ▄▄  ▄▄▄   ▄▄    */
///      ██▀▀▀▀█  ██▀▀▀▀██   ▀▀██▀▀   ██▀▀▀██              ██▀▀▀▀█   ██▀▀██   ███   ██  ███   ██    */
///     ██        ██    ██     ██     ██    ██            ██▀       ██    ██  ██▀█  ██  ██▀█  ██    */
///     ██  ▄▄▄▄  ███████      ██     ██    ██            ██        ██    ██  ██ ██ ██  ██ ██ ██    */
///     ██  ▀▀██  ██  ▀██▄     ██     ██    ██            ██▄       ██    ██  ██  █▄██  ██  █▄██    */
///      ██▄▄▄██  ██    ██   ▄▄██▄▄   ██▄▄▄██              ██▄▄▄▄█   ██▄▄██   ██   ███  ██   ███    */
///        ▀▀▀▀   ▀▀    ▀▀▀  ▀▀▀▀▀▀   ▀▀▀▀▀                  ▀▀▀▀     ▀▀▀▀    ▀▀   ▀▀▀  ▀▀   ▀▀▀    */
///                                                                                                 */
///                                                                                                 */
////////////////////////////////////////////////////////////////////////////////////////////////////*/
  master_log("Solax: status=0x");
  master_log_hex(&inverter.status, 1);
  master_log(" count=0x");
  master_log_hex(&inverter.status_count, 1);
  master_log("\n");
  // only do this after the inverter status is stable and ready for connection
  if (inverter.status_count >= GRID_SWITCH_STATE_COUNT ) {
    // when max charge value is degraded, then severs the grid connection

    if (auto_grid_connection == 1) {
      if (pylontech.soc > inverter.grid_disconnect_soc
        // when battery does not accept the full power for charging, it means it's either dead, or full. 
        // therefore sever the grid connection to avoid injection
        || pylontech.max_charge_dA < pylontech.max_discharge_dA
        ) {
        if (
          // only perform disconnection when we're in sync with the grid and in self use mode, else
          // no disconnection
          // at boot, when in EPS, must stay in EPS!, therefore activate the relay to stay in EPS
          (inverter.status == INVERTER_STATUS_NORMAL 
            || inverter.status == INVERTER_STATUS_EPS
            /* /!\ don't switch while waiting or EPS wait, that triggers power outage locally
            || inverter.status == INVERTER_STATUS_WAITING
            || inverter.status == INVERTER_STATUS_CHECKING
            || inverter.status == INVERTER_STATUS_EPS_WAIT
            */
            )
          ) {
          master_log("Antisurge: disconnect GRID, force EPS\n");
          offgrid_switch(1);
          inverter.status_count = 0; // avoid glitching too frequently
        }
      }
      // when SoC is lower than a value, then 
      else if (pylontech.soc <= inverter.grid_connect_soc) {
        master_log("Antisurge: connect GRID (2)\n");
        // restablish the GRID connection, 
        offgrid_switch(0);
        inverter.status_count = 0; // avoid glitching too frequently
      }
      else {
        switch(inverter.status) {
        // failing states, must reenable grid!!
        case INVERTER_STATUS_IDLE:
        case INVERTER_STATUS_ERROR:
        case INVERTER_STATUS_FAULT:
        case INVERTER_STATUS_STANDBY:
        case INVERTER_STATUS_UPDATE:
          master_log("Antisurge: connect GRID (3)\n");
          offgrid_switch(0);
          inverter.status_count = 0; // avoid glitching too frequently
          break;
        }
      }
    }
  }
  else {
    // wait state to stabilize
  }

#ifdef HAVE_EXT_CHARGER
  update_external_charger();
#endif // HAVE_EXT_CHARGER

#if 0
  // when battery has a charge request, then process it (dunno if solax executes it)
  if (pylontech.charge_request) {

  }
#endif 
}

/*








*/