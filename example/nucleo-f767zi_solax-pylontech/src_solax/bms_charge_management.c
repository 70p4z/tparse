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
 { .min_mV = 3400, .max_mV = 3425, .charge_dA = 90 },
 { .min_mV = 3430, .max_mV = 3460, .charge_dA = 70 },
 { .min_mV = 3465, .max_mV = 3475, .charge_dA = 50 },
 { .min_mV = 3480, .max_mV = 3495, .charge_dA = 30 },
 { .min_mV = 3500, .max_mV = 3520, .charge_dA = 10 },
 { .min_mV = 3525, .max_mV = 3550, .charge_dA = 3 },
 { .min_mV = 3560, .max_mV = 0,    .charge_dA = 1 }, // make sure controlled charge takes over
 //{ .min_mV = 3600, .max_mV = 0, .charge_dA = 0 },
};

// Hysterisis are NOT respected! when cell passes 3.5 then 0 applies, then when it jumps back to 3.499, then
// 2.0 applies! => must respect the fact the top value has crossed

//////////////////////////////////////////////////////////////////////////////////////////*/
///                                                                                       */
///               ▄▄    ▄▄            ▄▄▄▄▄▄▄▄    ▄▄▄▄                    ▄▄     ▄▄       */
///               ▀██  ██▀            ▀▀▀██▀▀▀   ██▀▀██                   ██    ████      */
///     ████▄██▄   ██  ██                ██     ██    ██             ▄███▄██    ████      */
///     ██ ██ ██   ██  ██                ██     ██    ██            ██▀  ▀██   ██  ██     */
///     ██ ██ ██    ████                 ██     ██    ██            ██    ██   ██████     */
///     ██ ██ ██    ████                 ██      ██▄▄██             ▀██▄▄███  ▄██  ██▄    */
///     ▀▀ ▀▀ ▀▀    ▀▀▀▀                 ▀▀       ▀▀▀▀                ▀▀▀ ▀▀  ▀▀    ▀▀    */
///                                                                                       */
///                                                                                       */
//////////////////////////////////////////////////////////////////////////////////////////*/

void bms_cap_charge_update(uint16_t max_cell_mV) {
  int i;

  // initialize the cap max charge if not initiliazed yet
  if (pylontech.cap_max_charge_dA == 0) {
    snprintf((char*)tmp+128, sizeof(tmp)-128, "cap max charge = use BMS %d (init) (was %d)\n", pylontech.max_charge_dA, pylontech.cap_max_charge_dA);
    master_log((char*)tmp+128);
    pylontech.cap_max_charge_dA = pylontech.max_charge_dA;
  }

  // avoid fully charged weirdness when requesting power
  if (pylontech.soc >= 100) {
    master_log("full battery, limit charge current\n");
    pylontech.cap_max_charge_dA = 1;
    return;
  }

  // no default cap max harge value to allow for hysterisis gap to respect the last set value
  for (i = 0; i<sizeof(bms_max_charge_constraint) / sizeof(bms_max_charge_constraint[0]) ; i++) {
    if ((bms_max_charge_constraint[i].min_mV == MV_NO_BOUNDARY || bms_max_charge_constraint[i].min_mV <= max_cell_mV)
        && (bms_max_charge_constraint[i].max_mV == MV_NO_BOUNDARY || bms_max_charge_constraint[i].max_mV >= max_cell_mV)) {
      if (bms_max_charge_constraint[i].charge_dA == CHARGE_NO_BOUNDARY) {
        snprintf((char*)tmp+128, sizeof(tmp)-128, "cap max charge = use BMS %d (uncapped) (was %d)\n", pylontech.max_charge_dA, pylontech.cap_max_charge_dA);
        master_log((char*)tmp+128);
        pylontech.cap_max_charge_dA = pylontech.max_charge_dA;
      }
      else {
        //uint16_t offset_dA = bms_max_charge_constraint[i].charge_dA>0?SOLAX_BATTERY_CHARGE_OFFSET_DA:0;
        uint16_t cap_dA = bms_max_charge_constraint[i].charge_dA;
        //snprintf((char*)tmp+128, sizeof(tmp)-128, "cap max charge = %d + %d (was %d)\n", cap_dA, offset_dA, pylontech.cap_max_charge);
        snprintf((char*)tmp+128, sizeof(tmp)-128, "cap max charge = %d  (was %d)\n", cap_dA, pylontech.cap_max_charge_dA);
        master_log((char*)tmp+128);
        pylontech.cap_max_charge_dA = cap_dA /*+ (cap_dA>1?offset_dA:0)*/;
      }
      // no more match SHOULD occur
      break;
    }
  }

  master_log("cap max charge = unchanged\n");
}

////////////////////////////////////////////////////////////////////////////////*/
///                                                                             */
///     ▄▄▄▄▄▄    ▄▄▄  ▄▄▄    ▄▄▄▄              ▄▄▄▄▄▄     ▄▄▄▄▄▄   ▄▄▄▄▄       */
///     ██▀▀▀▀██  ███  ███  ▄█▀▀▀▀█             ██▀▀▀▀█▄   ▀▀██▀▀   ██▀▀▀██     */
///     ██    ██  ████████  ██▄                 ██    ██     ██     ██    ██    */
///     ███████   ██ ██ ██   ▀████▄             ██████▀      ██     ██    ██    */
///     ██    ██  ██ ▀▀ ██       ▀██            ██           ██     ██    ██    */
///     ██▄▄▄▄██  ██    ██  █▄▄▄▄▄█▀            ██         ▄▄██▄▄   ██▄▄▄██     */
///     ▀▀▀▀▀▀▀   ▀▀    ▀▀   ▀▀▀▀▀              ▀▀         ▀▀▀▀▀▀   ▀▀▀▀▀       */
///                                                                             */
///                                                                             */
////////////////////////////////////////////////////////////////////////////////*/

int16_t update_charge(int16_t maxch) {

  // update highest/lowest cell values
  snprintf((char*)tmp+128, sizeof(tmp)-128, "update cap for Vcell minV: %dV, maxV: %dV\n", pylontech.vcell_lowest, pylontech.vcell_highest);
  master_log((char*)tmp+128);

  // update capped max charge depending on battery voltage
  bms_cap_charge_update(pylontech.vcell_highest);

  // computed battery measured charge current
  int16_t bat_current_dA = pylontech.current_dA;
  if (pylontech.precise_wattage) {
    // mA / 100 => dA
    bat_current_dA = pylontech.precise_current_mA/100;
  }

  // cap for limited current when voltage reaches limited voltage value
  uint32_t target_dA = pylontech.cap_max_charge_dA;
  if (pylontech.soc >= 100 || pylontech.vcellmax >= knobs.cell_voltage_limited_charge) {
    // W *100 / mV => dA
    target_dA = knobs.limited_charge_wattage*100/pylontech.voltage_dV;
    snprintf((char*)tmp+128, sizeof(tmp)-128, "limited charge enabled W=%d V=%d.%d A=%d.%d\n", 
      knobs.limited_charge_wattage,
      pylontech.voltage_dV/10,pylontech.voltage_dV%10,
      target_dA/10,target_dA%10);
    master_log((char*)tmp+128);
  }

  if (knobs.forced_wattage != 0) {
    // forced!
    maxch = bms_charge_pid(
      bat_current_dA, 
      knobs.forced_wattage*100/pylontech.voltage_dV,
      &pylontech_pid);

    snprintf((char*)tmp+128, sizeof(tmp)-128, "PID: FORCED cur=%ddA tgt=%ddA chg=%ddA\n", 
             knobs.forced_wattage,
             pylontech.voltage_dV,
             bat_current_dA,
             knobs.forced_wattage*100/pylontech.voltage_dV,
             maxch
             );
    // skip maxch cap!
  }
  else {

    // ============================================================
    // LAYER 1 — SAFETY (HYSTERESIS + CHARGE ENABLE)
    // ============================================================

    if (VCELL_VALID(pylontech.vcell_highest)) {
      if (pylontech.vcell_highest >= 3550) {
          pylontech.charge_disabled = true;
      }
      else if (pylontech.vcell_highest < 3450) {
          pylontech.charge_disabled = false;
      }
    }

    if (pylontech.charge_disabled) {
      master_log("min current offset dA, charge not allowed\n");
      maxch = pylontech_pid.min_current_offset_dA;
    }
    else {
      // use regular PID management
      maxch = bms_charge_pid(
        bat_current_dA, 
        target_dA,
        &pylontech_pid);
    }
    snprintf((char*)tmp+128, sizeof(tmp)-128, "PID: cur=%ddA tgt=%ddA chg=%ddA\n", 
             bat_current_dA,
             target_dA,
             maxch
             );

    // avoid too high result (shall be taken care of in the PID instead!, this is security harness)
    if (maxch > target_dA + 8) {
      maxch = target_dA + 8;
      master_log("cap max charge for target respect\n");
    }
  }
  master_log((char*)tmp+128);


  return maxch;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////*/
///                                                                                                  */
///     ▄▄▄▄▄▄       ▄▄     ▄▄▄▄▄▄▄▄               ▄▄▄▄   ▄▄    ▄▄  ▄▄▄▄▄▄       ▄▄▄▄   ▄▄▄▄▄▄       */
///     ██▀▀▀▀██    ████    ▀▀▀██▀▀▀             ██▀▀▀▀█  ██    ██  ██▀▀▀▀██   ██▀▀▀▀█  ██▀▀▀▀██     */
///     ██    ██    ████       ██               ██▀       ██    ██  ██    ██  ██        ██    ██     */
///     ███████    ██  ██      ██               ██        ████████  ███████   ██  ▄▄▄▄  ███████      */
///     ██    ██   ██████      ██               ██▄       ██    ██  ██  ▀██▄  ██  ▀▀██  ██  ▀██▄     */
///     ██▄▄▄▄██  ▄██  ██▄     ██                ██▄▄▄▄█  ██    ██  ██    ██   ██▄▄▄██  ██    ██     */
///     ▀▀▀▀▀▀▀   ▀▀    ▀▀     ▀▀                  ▀▀▀▀   ▀▀    ▀▀  ▀▀    ▀▀▀    ▀▀▀▀   ▀▀    ▀▀▀    */
///                                                                                                  */
///                                                                                                  */
/////////////////////////////////////////////////////////////////////////////////////////////////////*/
void update_external_charger(void) {

  if (pylontech.tcellmax <= knobs.max_charge_temperature) {
    if (auto_bat_charge) {
      // we've charged enough, stop compensation from grid
      if (charger.charge_enabled
        && (pylontech.soc >= knobs.charger_stop_soc
        || (VCELL_VALID(pylontech.vcell_highest) && pylontech.vcell_highest >= 3550))) {
        charger.charge_enabled = 0;
        // ensure no trouble
        charger.max_charge_current = 0; 
        master_log("charger: disable charge\n");

      }
      // when to enable auto charge to compensate 
      else if (!charger.charge_enabled
        && pylontech.soc <= knobs.charger_start_soc
        && VCELL_VALID(pylontech.vcell_highest) && pylontech.vcell_highest < 3450) {
        charger.charge_enabled = 1;
        master_log("charger: enable charge\n");
      }

      // modulate charge current to compensate EPS load
      if (charger.charge_enabled) {
        int target_charge_current = knobs.allowed_charge_wattage * 100 / pylontech.voltage_dV;

        // compensate the current of the battery (being drawn or already charging)
        int allowed_dA = bms_charge_pid(
            pylontech.current_dA,
            target_charge_current,
            &charger_pid
        );

        // update v/a from targeted wattage
        charger.max_charge_voltage = charger.out_voltage + 50; // in dV
        // ONLY CHARGE ! else discard
        charger.max_charge_current = allowed_dA>0?allowed_dA:0;
      }
    }
    // apply max allowed charge current
    if (!auto_bat_charge) {
      charger.max_charge_voltage = charger.out_voltage + 50; // in dV
      charger.max_charge_current = knobs.allowed_charge_wattage * 10 * 10 / charger.out_voltage; // in dA
    }
  }
  else {
    // disable charging
    charger.charge_enabled = 0;
  }
}