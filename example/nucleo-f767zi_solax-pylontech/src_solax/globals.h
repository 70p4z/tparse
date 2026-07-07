#pragma once
#include "stdlib.h"
#include "stdint.h"
#include "stddef.h"
#include "stdbool.h"
#include "bms_charge_pid.h"

struct knobs_s {
  int32_t forced_soc; // unit %
  int32_t forced_wattage; // unit: W
  uint8_t max_charge_voltage; // unit dV
  uint8_t max_pylontech_charge_drive; // unit dV // when voltage over this value, then we are in 
                                                 // control of the charge current (redondant with 
                                                 // limited charge)
  uint8_t cell_voltage_limited_charge; // unit dV (0 means disabled limited_wattage)
  int32_t limited_charge_wattage; // unit: W (0 means disabled limited wattage)
  int16_t max_charge_temperature; // unit 0.1°C

  uint16_t allowed_charge_wattage; // in W
  uint8_t charger_start_soc;
  uint8_t charger_stop_soc;
};

enum solax_forced_work_mode_e {
  SOLAX_FORCED_WORK_MODE_NONE,
  SOLAX_FORCED_WORK_MODE_SELF_USE,
  SOLAX_FORCED_WORK_MODE_BACKUP,
  SOLAX_FORCED_WORK_MODE_MANUAL_STOP,
  SOLAX_FORCED_WORK_MODE_MANUAL_CHARGE,
  SOLAX_FORCED_WORK_MODE_MANUAL_DISCHARGE,
};


struct inverter_s {
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

  uint8_t grid_connect_soc;
  uint8_t grid_disconnect_soc;
  uint8_t valid_data;

  uint8_t self_use_discharge_enabled;
};

#define PYLONTECH_MAX_BMUS 20
struct pylontech_s {
  uint16_t voltage_dV;
  uint32_t precise_voltage_mV; // mV
  int16_t current_dA;
  int32_t precise_current_mA;  // mA
  uint8_t soc;
  uint8_t apparent_soc;
  uint8_t soc_mWh;
  uint32_t precise_mAh;
  uint32_t precise_mAh_ts; // to compute approx current drive, when 0 is reported
  uint32_t precise_mWh;
  int32_t wattage;
  int32_t precise_wattage;
  int16_t max_charge_dA;
  int16_t cap_max_charge_dA;
  int16_t max_discharge_dA;
  int16_t effective_charge_dA;
  uint16_t packs;
  uint8_t max_charge_soc;
  uint8_t contactor_on;
  uint8_t charge_request;
  uint16_t cycles;
  uint8_t fix2_31;
  uint32_t total_capacity_mAh;
  uint8_t vcellmax;
  uint8_t vcellmin;
  uint8_t tcellmax;
  uint8_t tcellmin;

  // cached values
  uint16_t vcell_highest;
  uint16_t vcell_lowest;

  uint8_t bmu_idx_tmp;
  uint8_t bmu_idx;
  // ONLY VALID WHEN BMU8IDX != 0
  // {
  struct {
    uint8_t soc;
    uint8_t soc_mWh;
    uint16_t vlow;
    uint16_t vhigh;
    uint8_t pcba[32+1];
  } bmu[PYLONTECH_MAX_BMUS];
  uint16_t vcell_highest_tmp;
  uint16_t vcell_lowest_tmp;
  bool charge_disabled;
  // }
};

extern struct inverter_s inverter;
extern struct pylontech_s pylontech;
extern current_controller_pv_t pylontech_pid;

extern current_controller_pv_t charger_pid;

extern struct knobs_s knobs;

extern uint32_t auto_self_use_from_bat;
extern uint32_t auto_grid_connection;
extern uint32_t auto_bat_charge;
extern enum solax_forced_work_mode_e solax_forced_work_mode;

#define TMP_BUFFER_SIZE_B 1024
extern uint8_t tmp[TMP_BUFFER_SIZE_B];


#define CHARGER_COM_INTERVAL_MS 2500 // different period from the inverter call to try avoiding collision on CAN
struct charger_s {
  uint8_t charge_enabled;
  uint16_t max_charge_voltage; // in 0.1V
  uint16_t max_charge_current; // in 0.1A
  uint16_t out_voltage; // in 0.1V
  uint16_t out_current; // in 0.1A
  union {
    uint8_t status;
    struct {
      uint8_t hw_failure:1;
      uint8_t over_temp:1;
      uint8_t ac_voltage_fault:1;
      uint8_t batt_disconnected:1;
      uint8_t comm_failure:1;
    };
  };
};
extern struct charger_s charger;



#define S2LE(buf, off) ((int16_t)((int16_t)((int16_t)((int16_t)(buf)[off+1])<<8l) | (int16_t)((int16_t)(buf)[off]&0xFFl) ))
#define U2LE(buf, off) ((((buf)[off+1]&0xFFu)<<8) | ((buf)[off]&0xFFu) )
#define U4LE(buf, off) ((U2LE(buf, off+2)<<16) | (U2LE(buf, off)&0xFFFFu))
#define U2BE(buf, off) ((((buf)[off]&0xFFu)<<8) | ((buf)[off+1]&0xFFu) )
#define U4BE_ENCODE(buf, off, value) {(buf)[(off)+0] = ((value)>>24)&0xFF;(buf)[(off)+1] = ((value)>>16)&0xFF;(buf)[(off)+2] = ((value)>>8)&0xFF;(buf)[(off)+3] = ((value))&0xFF;}

#define VCELL_VALID(v) ((v)>=2000 && (v)<=4000)

#define EXPIRED(time) (((uint32_t)uwTick - (uint32_t)(time)) < (uint32_t)0x80000000)
#define EXPIRE_IN(interval) expire_in(interval)
uint32_t expire_in(uint32_t interval);
void inverter_usart_queue_pop(void);
uint32_t inverter_usart_queue_free(void);
void inverter_usart_queue_push(const uint8_t* cmd, uint32_t cmd_len, uint32_t rep_len);
void inverter_uart_init(void);
void inverter_uart_update(void);

void bms_uart_init(void);
void bms_uart_update(void);

void master_log(char* buffer);
void master_log_mem(void* _buffer, size_t length);
void master_log_hex(void* _buffer, size_t length);

void solax_process_data(void);
void solax_pw_gmppt1_off(void);
void solax_pw_gmppt1_high(void);
void solax_pw_gmppt2_off(void);
void solax_pw_gmppt2_high(void);
void solax_pw_mode_self_use(void);
void inverter_uart_force_bitrate(void);

void master_log(char* buffer);
void master_log_mem(void* _buffer, size_t length);
void master_log_hex(void* _buffer, size_t length);
void master_log_can(char* prefix, uint32_t cid, size_t cid_bitlen, uint8_t* canmsg, size_t canmsg_len);
void can_inv_tx_log(uint32_t cid, size_t cid_bitlen, uint8_t* canmsg, size_t canmsg_len);
void can_bms_tx_log(uint32_t cid, size_t cid_bitlen, uint8_t* canmsg, size_t canmsg_len);

// return 1 when message has to be forwarded to the BMS CAN bus
uint32_t can_inv_interp(uint32_t cid, size_t cid_bitlen, uint8_t* candata, size_t candata_len);
// return 1 when message has to be forwarded to the inverter CAN bus
uint32_t can_bms_interp(uint32_t cid, size_t cid_bitlen, uint8_t* candata, size_t candata_len);

void offgrid_switch(uint32_t eps_mode_requested);
void transcharge_disable_all(void);
int16_t update_charge(int16_t maxch);

#define CAN_ID_STANDARD_LEN 11
#define CAN_ID_EXTENDED_LEN 29

#ifndef MAX
#define MAX(x,y) ((x)>(y)?(x):(y))
#endif // MAX
#ifndef MIN
#define MIN(x,y) ((x)<(y)?(x):(y))
#endif // MIN
