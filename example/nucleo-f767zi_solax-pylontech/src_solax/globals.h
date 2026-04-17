#pragma once
#include "stdlib.h"
#include "stdint.h"
#include "stddef.h"
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
};

enum solax_forced_work_mode_e {
  SOLAX_FORCED_WORK_MODE_NONE,
  SOLAX_FORCED_WORK_MODE_SELF_USE,
  SOLAX_FORCED_WORK_MODE_BACKUP,
  SOLAX_FORCED_WORK_MODE_MANUAL_STOP,
  SOLAX_FORCED_WORK_MODE_MANUAL_CHARGE,
  SOLAX_FORCED_WORK_MODE_MANUAL_DISCHARGE,
};


struct solax_s {
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
  uint8_t enable_self_use_soc;
  uint8_t disable_self_use_soc;
  uint8_t stop_discharge_soc;
  uint8_t valid_data;

  uint8_t self_use_discharge_enabled;
};

#define PYLONTECH_MAX_BMUS 20
struct pylontech_s {
  uint16_t voltage;
  uint32_t precise_voltage; // mV
  int16_t current;
  int32_t precise_current;  // mA
  uint8_t soc;
  uint8_t apparent_soc;
  uint8_t soc_mWh;
  uint32_t precise_mAh;
  uint32_t precise_mAh_ts; // to compute approx current drive, when 0 is reported
  uint32_t precise_mWh;
  int32_t wattage;
  int32_t precise_wattage;
  int16_t max_charge;
  int16_t cap_max_charge;
  int16_t max_discharge;
  int16_t effective_charge;
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
  // }
};

extern struct solax_s solax;
extern struct pylontech_s pylontech;
extern current_controller_pv_t pylontech_pid;

extern struct knobs_s knobs;

extern uint32_t auto_self_use_from_bat;
extern uint32_t auto_grid_connection;
extern uint32_t auto_bat_charge;
extern enum solax_forced_work_mode_e solax_forced_work_mode;

#define TMP_BUFFER_SIZE_B 1024
extern uint8_t tmp[TMP_BUFFER_SIZE_B];

#define VCELL_VALID(v) ((v)>=2000 && (v)<=4000)
