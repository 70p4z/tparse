#include "globals.h"
#include "stdio.h"

void master_log(char*);

#define MV_NO_BOUNDARY 0
#define CHARGE_NO_BOUNDARY 255
struct {
  uint16_t min_mV; // if 0 => no boundary
  uint16_t max_mV; // if 0 => no boundary
  uint16_t charge_dA;  // if 255 => no boundary
} const bms_max_charge_constraint[] = {
 { .min_mV = 0,    .max_mV = 3370, .charge_dA = 255 },
 { .min_mV = 3375, .max_mV = 3395, .charge_dA = 100 },
 { .min_mV = 3400, .max_mV = 3425, .charge_dA = 60 },
 { .min_mV = 3430, .max_mV = 3460, .charge_dA = 20 },
 { .min_mV = 3465, .max_mV = 3475, .charge_dA = 10 },
 { .min_mV = 3480, .max_mV = 3495, .charge_dA = 5 },
 { .min_mV = 3500, .max_mV = 3550, .charge_dA = 3 },
 { .min_mV = 3560, .max_mV = 0,    .charge_dA = 1 }, // make sure controlled charge takes over
 //{ .min_mV = 3600, .max_mV = 0, .charge_dA = 0 },
};

// Hysterisis are NOT respected! when cell passes 3.5 then 0 applies, then when it jumps back to 3.499, then
// 2.0 applies! => must respect the fact the top value has crossed

void bms_cap_charge_update(uint16_t max_cell_mV) {
  int i;

  // initialize the cap max charge if not initiliazed yet
  if (pylontech.cap_max_charge == 0) {
    snprintf((char*)tmp+128, sizeof(tmp)-128, "cap max charge = use BMS %d (init) (was %d)\n", pylontech.max_charge, pylontech.cap_max_charge);
    master_log((char*)tmp+128);
    pylontech.cap_max_charge = pylontech.max_charge;
  }

  // avoid fully charged weirdness when requesting power
  if (pylontech.soc >= 100) {
    master_log("full battery, limit charge current\n");
    pylontech.cap_max_charge = 1;
    return;
  }

  // no default cap max harge value to allow for hysterisis gap to respect the last set value
  for (i = 0; i<sizeof(bms_max_charge_constraint) / sizeof(bms_max_charge_constraint[0]) ; i++) {
    if ((bms_max_charge_constraint[i].min_mV == MV_NO_BOUNDARY || bms_max_charge_constraint[i].min_mV <= max_cell_mV)
        && (bms_max_charge_constraint[i].max_mV == MV_NO_BOUNDARY || bms_max_charge_constraint[i].max_mV >= max_cell_mV)) {
      if (bms_max_charge_constraint[i].charge_dA == CHARGE_NO_BOUNDARY) {
        snprintf((char*)tmp+128, sizeof(tmp)-128, "cap max charge = use BMS %d (uncapped) (was %d)\n", pylontech.max_charge, pylontech.cap_max_charge);
        master_log((char*)tmp+128);
        pylontech.cap_max_charge = pylontech.max_charge;
      }
      else {
        //uint16_t offset_dA = bms_max_charge_constraint[i].charge_dA>0?SOLAX_BATTERY_CHARGE_OFFSET_DA:0;
        uint16_t cap_dA = bms_max_charge_constraint[i].charge_dA;
        //snprintf((char*)tmp+128, sizeof(tmp)-128, "cap max charge = %d + %d (was %d)\n", cap_dA, offset_dA, pylontech.cap_max_charge);
        snprintf((char*)tmp+128, sizeof(tmp)-128, "cap max charge = %d  (was %d)\n", cap_dA, pylontech.cap_max_charge);
        master_log((char*)tmp+128);
        pylontech.cap_max_charge = cap_dA /*+ (cap_dA>1?offset_dA:0)*/;
      }
      // no more match SHOULD occur
      break;
    }
  }

  master_log("cap max charge = unchanged\n");
}

uint16_t update_charge(uint16_t maxch) {

  // update highest/lowest cell values
  snprintf((char*)tmp+128, sizeof(tmp)-128, "update cap for Vcell minV: %dV, maxV: %dV\n", pylontech.vcell_lowest, pylontech.vcell_highest);
  master_log((char*)tmp+128);

  // update capped max charge depending on battery voltage
  bms_cap_charge_update(pylontech.vcell_highest);

  // computed battery measured charge current
  int16_t bat_current_dA = pylontech.current;
  if (pylontech.precise_wattage) {
    // mA / 100 => dA
    bat_current_dA = pylontech.precise_current/100;
  }

  // cap for limited current when voltage reaches limited voltage value
  uint32_t target_dA = pylontech.cap_max_charge;
  if (pylontech.soc >= 100 || pylontech.vcellmax >= knobs.cell_voltage_limited_charge) {
    // W *100 / mV => dA
    target_dA = knobs.limited_charge_wattage*100/pylontech.voltage;
    snprintf((char*)tmp+128, sizeof(tmp)-128, "limited charge enabled W=%d V=%d dA=%d\n", 
      knobs.limited_charge_wattage,
      pylontech.voltage,
      target_dA);
    master_log((char*)tmp+128);
  }

  if (knobs.forced_wattage != 0) {
    // forced!
    pylontech_pid.charge_allowed = true;
    maxch = bms_charge_pid(
      bat_current_dA, 
      knobs.forced_wattage*100/pylontech.voltage,
      pylontech.vcell_highest, 
      &pylontech_pid);

    snprintf((char*)tmp+128, sizeof(tmp)-128, "PID: FORCED cur=%ddA tgt=%ddA chg=%ddA\n", 
             knobs.forced_wattage,
             pylontech.voltage,
             bat_current_dA,
             knobs.forced_wattage*100/pylontech.voltage,
             maxch
             );
  }
  else {
    maxch = bms_charge_pid(
      bat_current_dA, 
      target_dA,
      pylontech.vcell_highest, 
      &pylontech_pid);


    snprintf((char*)tmp+128, sizeof(tmp)-128, "PID: cur=%ddA tgt=%ddA chg=%ddA\n", 
             bat_current_dA,
             target_dA,
             maxch
             );
  }
  master_log((char*)tmp+128);

  // avoid too high result (shall be taken care of in the PID instead!, this is security harness)
  if (maxch > target_dA + 8) {
    maxch = target_dA + 8;
    master_log("cap max charge for target respect\n");
  }

  return maxch;
}