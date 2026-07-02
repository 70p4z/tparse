#include "globals.h"
#include "consts.h"
#include "stdio.h"
#include "string.h"

uint32_t can_bms_interp(uint32_t cid, size_t cid_bitlen, uint8_t* candata, size_t candata_len) {
  uint32_t forward = 0;
  switch(cid) {

//////////////////////////////////////////////////////////////////////////////////////////*/
///                                                                                       */
///       ▄▄▄▄      ▄▄▄▄    ▄▄           ▄▄     ▄▄▄  ▄▄▄                                  */
///     ▄█▀▀▀▀█    ██▀▀██   ██          ████     ██▄▄██                                   */
///     ██▄       ██    ██  ██          ████      ████                                    */
///      ▀████▄   ██    ██  ██         ██  ██      ██                                     */
///          ▀██  ██    ██  ██         ██████     ████                                    */
///     █▄▄▄▄▄█▀   ██▄▄██   ██▄▄▄▄▄▄  ▄██  ██▄   ██  ██                                   */
///      ▀▀▀▀▀      ▀▀▀▀    ▀▀▀▀▀▀▀▀  ▀▀    ▀▀  ▀▀▀  ▀▀▀                                  */
///                                                                                       */
///                                                                                       */
///                                                                                       */
///     ▄▄▄▄▄▄    ▄▄▄▄▄▄      ▄▄▄▄    ▄▄▄▄▄▄▄▄    ▄▄▄▄       ▄▄▄▄     ▄▄▄▄    ▄▄          */
///     ██▀▀▀▀█▄  ██▀▀▀▀██   ██▀▀██   ▀▀▀██▀▀▀   ██▀▀██    ██▀▀▀▀█   ██▀▀██   ██          */
///     ██    ██  ██    ██  ██    ██     ██     ██    ██  ██▀       ██    ██  ██          */
///     ██████▀   ███████   ██    ██     ██     ██    ██  ██        ██    ██  ██          */
///     ██        ██  ▀██▄  ██    ██     ██     ██    ██  ██▄       ██    ██  ██          */
///     ██        ██    ██   ██▄▄██      ██      ██▄▄██    ██▄▄▄▄█   ██▄▄██   ██▄▄▄▄▄▄    */
///     ▀▀        ▀▀    ▀▀▀   ▀▀▀▀       ▀▀       ▀▀▀▀       ▀▀▀▀     ▀▀▀▀    ▀▀▀▀▀▀▀▀    */
///                                                                                       */
///                                                                                       */
//////////////////////////////////////////////////////////////////////////////////////////*/
    case 0x1873:
      // may reinterpret SoC depending on battery voltage instead of relying on BMS
      pylontech.voltage_dV = U2LE(candata,0);
      pylontech.current_dA = S2LE(candata,2);
      pylontech.soc = U2LE(candata,4);
      pylontech.wattage = ((int32_t)pylontech.voltage_dV)*((int32_t)pylontech.current_dA)/((int32_t)100); // unit 0.1V x 0.1A
      snprintf((char*)tmp+16, sizeof(tmp)-16, "            | Vbat=%d.%dV\tIbat=%d.%dA\tWbat=%ldW\tSoC=%d\tEbat=%d.%dkWh\n", 
               pylontech.voltage_dV/10,pylontech.voltage_dV%10,
               pylontech.current_dA/10,pylontech.current_dA%10,
               pylontech.wattage,
               pylontech.soc,
               U2LE(candata,6)/100,U2LE(candata,6)%100);
      master_log((char*)tmp+16);
      tmp[16] = 0;

      // always apply a SoC that allosw the inverter to charge. to ensure battery full workaround is effective
      // ensure charging when forcing charge (invert will not deny charge at that value)
      candata[4] = MIN(pylontech.soc, pylontech.apparent_soc?pylontech.apparent_soc:75);
      candata[5] = 0;

      if (knobs.forced_soc > 0) {
        master_log("force batt soc\n");
        candata[4] = knobs.forced_soc&0xFF;
        candata[5] = 0;
      }
      forward = 1;
      break;
    case 0x1872: {

      int16_t maxch = S2LE(candata, 4);
      int16_t maxdis = S2LE(candata, 6);

      // detect 2^31 bug (could affine with SysError detection on FlagsBMS = 0x0002)
      pylontech.fix2_31 = maxdis <= 0 && maxch <= 0;

      snprintf((char*)tmp+16, sizeof(tmp)-16, "            | Vbatmax=%d.%dV\tVbatmin=%d.%dV\tIcmax=%d.%dA\tIdmax=%d.%dA%s\n", 
               S2LE(candata, 0)/10,S2LE(candata, 0)%10,
               S2LE(candata, 2)/10,S2LE(candata, 2)%10,
               maxch/10,maxch%10,
               maxdis/10,maxdis%10,
               (pylontech.fix2_31)?" 2^31fix":"");
      master_log((char*)tmp+16);
      tmp[16] = 0;

      // when soc is below 10%, then the discharge may be 0
      // if soc > 10%, then max discharge can never be 0, it's 
      // the manifestation of the 2^31 counter overflow bug in the SC0500
      // in that case, we won't tranmsit and keep the previous value sent to the inverter
      // the bug has a periodicity of 1h and a duration of a few seconds.
      // it takes to circumvent the bug on the coil level too to make this patch effective.
      if (pylontech.fix2_31 
        // when max charge is reported below zero (hence discharging?) => send the last good value
        // if 0, then probably battery full or out of voltage bound, DON'T reapply previous value
        || maxch < 0) {
        // use previous max charge value to avoid charge disruption (and ihccups on the grid export side)
        maxch = pylontech.max_charge_dA;
        candata[4] = maxch&0xFF;
        candata[5] = (maxch>>8)&0xFF;
        // use previous value for max discharge, to avoid service disruption
        maxdis = pylontech.max_discharge_dA;
        candata[6] = maxdis&0xFF;
        candata[7] = (maxdis>>8)&0xFF;
      }

      /* taken into account in the PID directly now, to compensate low charge to approach 0
      // ensure respecting maxcharge, taking into account the inner charge cap offset
      if (maxch) {
        maxch += SOLAX_BATTERY_CHARGE_OFFSET_DA;
      }
      */

      pylontech.max_charge_dA = maxch;
      pylontech.max_discharge_dA = maxdis;

      // disallow discharge when battery is too low
      if (pylontech.soc <= 10) {
        master_log("soc < 10\n");
        // disable discharge
        candata[6] = 0;
        candata[7] = 0;
      }

      maxch = update_charge(maxch);

      candata[4] = maxch&0xFF;
      candata[5] = (maxch>>8)&0xFF;
      pylontech.effective_charge_dA = maxch;

      // absolute max rating to avoid chemistry degradation
      // has not worked when maxch was negative
      if (pylontech.vcellmax >= knobs.max_charge_voltage
        // avoid charging when battery are too hot!
        || pylontech.tcellmax >= knobs.max_charge_temperature) {
        master_log("batt voltage dangerous (3.6V), or temperature too high, stop forced charge to avoid wearing\n");
        candata[4] = 0;
        candata[5] = 0;
        pylontech.effective_charge_dA = 0;

        // TODO, switch to automatic offgrid, and force offgrid! => solax has a bug continuing 
        // to charge when full and still connected on grid even if charge current is set to 0
        // ignore auto switch here. this is a measure for battery safety!
        offgrid_switch(1);
        // force auto switch mode. to avoid forcing
        auto_grid_connection = 1;
        // battery voltage too high, stop transcharge balancing
        transcharge_disable_all();
#ifdef HAVE_EXT_CHARGER
        // disable all forms of charge!
        charger.charge_enabled = 0;
#endif // HAVE_EXT_CHARGER
      }
      forward = 1;
      break;
    }
    case 0x1878:
      snprintf((char*)tmp+16, sizeof(tmp)-16, "            | Format=%d\tUnk=%d\tFlagsBMS<<16=%04x\tCapacity=%dWh\n", 
               candata[0],
               candata[1],
               U2LE(candata, 2),
               U4LE(candata, 4));
      master_log((char*)tmp+16);
      tmp[16] = 0;
      // wipe flags on 2^31 fix
      if (pylontech.fix2_31) {
        candata[2] = candata[3] = 0;
      }
      forward = 1;
      break;

    case 0x1875:
      // note: if packs number is invalid, then the BMS_KIND is faulty => inverter goes into fault mode
      pylontech.packs = U2LE(candata,2); 
      // initialize the limited charge wattage, depending on connected pack count
      // TODO, use a current instead, to make it static, and avoid that runtime initialization
      if (knobs.limited_charge_wattage == -1) {
        knobs.limited_charge_wattage = BMS_LIMITED_CHARGE_WATTAGE_PER_PACK * pylontech.packs;
      }
      pylontech.contactor_on = candata[4];
      pylontech.charge_request = candata[5];
      pylontech.cycles = U2LE(candata, 6);
      // TODO: ensure contactor is set to 1 (tmp[4] = 1)
      snprintf((char*)tmp+16, sizeof(tmp)-16, "            | Tbms=%d.%d°C\tBatCount=%d\tContact=%d\tChargeReq=%d\tCycles=%d\n", 
               S2LE(candata, 0)/10,S2LE(candata, 0)%10,
               pylontech.packs,
               pylontech.contactor_on,
               pylontech.charge_request,
               pylontech.cycles);
      master_log((char*)tmp+16);
      tmp[16] = 0;
      forward = 1;
      break;

    case 0x1877:
      snprintf((char*)tmp+16, sizeof(tmp)-16, "            | FlagsBMS=%04x\n", 
               U2LE(candata, 0));
      master_log((char*)tmp+16);
      tmp[16] = 0;
      // wipe BMS flags upon 2^31 fix
      if (pylontech.fix2_31) {
        candata[0] = candata[1] = 0;
        //memset(tmp, 0, 4); 
      }
      candata[4] = BMS_KIND;
      candata[5] = candata[6] = candata[7] = 0; // wipe versions
      // override message to tell the inverter of the battery configuration
      //memmove(candata, "\x00\x00\x00\x00\x52\x00\x00\x00", 8); // OK 2H48050, OK 4 H48050p, OK 6 H48050, NOK 3/5/7/8 => seen as T58
      //memmove(candata, "\x00\x00\x00\x00\x82\x00\x00\x00", 8); // NOK 8 H48050 // brand TP201
      //memmove(candata, "\x00\x00\x00\x00\x83\x00\x00\x00", 8); // OK 8 H48050 // brand TP202
      forward = 1;
      break;

    // discarded
    case 0x1874:
      pylontech.tcellmax=S2LE(candata, 0);
      pylontech.tcellmin=S2LE(candata, 2);
      pylontech.vcellmax=S2LE(candata, 4);
      pylontech.vcellmin=S2LE(candata, 6);
      snprintf((char*)tmp+16, sizeof(tmp)-16, "            | Tcellmax=%d.%d°C\tTcellmin=%d.%d°C\tVcellmax=%d.%dV\tVcellmin=%d.%dV\n", 
               S2LE(candata, 0)/10,S2LE(candata, 0)%10,
               S2LE(candata, 2)/10,S2LE(candata, 2)%10,
               S2LE(candata, 4)/10,S2LE(candata, 4)%10,
               S2LE(candata, 6)/10,S2LE(candata, 6)%10);
      master_log((char*)tmp+16);
      tmp[16] = 0;
      break;
    case 0x1876:
      snprintf((char*)tmp+16, sizeof(tmp)-16, "            | dlim=%d\tIdlim=%d.%dA\tclim=%d\tIclim=%d.%dA\n", 
               U2LE(candata, 0),
               S2LE(candata, 2)/10,S2LE(candata, 2)%10,
               U2LE(candata, 4),
               S2LE(candata, 6)/10,S2LE(candata, 6)%10);
      master_log((char*)tmp+16);
      tmp[16] = 0;
      break;

////////////////////////////////////////////////////////////////////////////////////////////////////*/
///                                                                                                 */
///       ▄▄▄▄       ▄▄▄▄     ▄▄▄▄    ▄▄▄▄▄▄▄     ▄▄▄▄      ▄▄▄▄              ▄▄    ▄▄  ▄▄    ▄▄    */
///     ▄█▀▀▀▀█    ██▀▀▀▀█   ██▀▀██   ██▀▀▀▀▀    ██▀▀██    ██▀▀██             ██    ██  ▀██  ██▀    */
///     ██▄       ██▀       ██    ██  ██▄▄▄▄    ██    ██  ██    ██            ██    ██   ██  ██     */
///      ▀████▄   ██        ██ ██ ██  █▀▀▀▀██▄  ██ ██ ██  ██ ██ ██            ████████   ██  ██     */
///          ▀██  ██▄       ██    ██        ██  ██    ██  ██    ██            ██    ██    ████      */
///     █▄▄▄▄▄█▀   ██▄▄▄▄█   ██▄▄██   █▄▄▄▄██▀   ██▄▄██    ██▄▄██             ██    ██    ████      */
///      ▀▀▀▀▀       ▀▀▀▀     ▀▀▀▀     ▀▀▀▀▀      ▀▀▀▀      ▀▀▀▀              ▀▀    ▀▀    ▀▀▀▀      */
///                                                                                                 */
///                                                                                                 */
///                                                                                                 */
///     ▄▄▄▄▄▄    ▄▄▄▄▄▄      ▄▄▄▄    ▄▄▄▄▄▄▄▄    ▄▄▄▄       ▄▄▄▄     ▄▄▄▄    ▄▄                    */
///     ██▀▀▀▀█▄  ██▀▀▀▀██   ██▀▀██   ▀▀▀██▀▀▀   ██▀▀██    ██▀▀▀▀█   ██▀▀██   ██                    */
///     ██    ██  ██    ██  ██    ██     ██     ██    ██  ██▀       ██    ██  ██                    */
///     ██████▀   ███████   ██    ██     ██     ██    ██  ██        ██    ██  ██                    */
///     ██        ██  ▀██▄  ██    ██     ██     ██    ██  ██▄       ██    ██  ██                    */
///     ██        ██    ██   ██▄▄██      ██      ██▄▄██    ██▄▄▄▄█   ██▄▄██   ██▄▄▄▄▄▄              */
///     ▀▀        ▀▀    ▀▀▀   ▀▀▀▀       ▀▀       ▀▀▀▀       ▀▀▀▀     ▀▀▀▀    ▀▀▀▀▀▀▀▀              */
///                                                                                                 */
///                                                                                                 */
////////////////////////////////////////////////////////////////////////////////////////////////////*/
    // Stack status
    case 0x4210:
      // may reinterpret SoC depending on battery voltage instead of relying on BMS
      pylontech.voltage_dV = U2LE(candata,0);
      pylontech.current_dA = S2LE(candata,2) - 30000;
      pylontech.soc = candata[6];
      pylontech.wattage = ((int32_t)pylontech.voltage_dV)*((int32_t)pylontech.current_dA)/((int32_t)100); // unit 0.1V x 0.1A
      snprintf((char*)tmp+16, sizeof(tmp)-16, "            | Vbat=%d.%dV\tIbat=%d.%dA\tWbat=%ldW\tSoC=%d%%\tSoH=%d%%\n", 
               pylontech.voltage_dV/10,pylontech.voltage_dV%10,
               pylontech.current_dA/10,pylontech.current_dA%10,
               pylontech.wattage,
               pylontech.soc,
               candata[7]);
      master_log((char*)tmp+16);
      tmp[16] = 0;

      // always apply a SoC that allosw the inverter to charge. to ensure battery full workaround is effective
      // ensure charging when forcing charge (invert will not deny charge at that value)
      candata[6] = MIN(pylontech.soc, pylontech.apparent_soc?pylontech.apparent_soc:75);

      if (knobs.forced_soc > 0) {
        master_log("force batt soc\n");
        candata[6] = knobs.forced_soc&0xFF;
      }
      forward = 1;
      break;

    // Charge/Discharge limits
    case 0x4220:
    {
      int16_t corr;
      int16_t maxch = S2LE(candata, 4) - 30000;
      int16_t maxdis = - (S2LE(candata, 6) - 30000); // use positive max discharge value

      // detect 2^31 bug (could affine with SysError detection on FlagsBMS = 0x0002)
      pylontech.fix2_31 = maxdis <= 0 && maxch <= 0;

      snprintf((char*)tmp+16, sizeof(tmp)-16, "            | Icmax=%d.%dA\tIdmax=%d.%dA%s\n", 
               maxch/10,maxch%10,
               maxdis/10,maxdis%10,
               (pylontech.fix2_31)?" 2^31fix":"");
      master_log((char*)tmp+16);
      tmp[16] = 0;

      // when soc is below 10%, then the discharge may be 0
      // if soc > 10%, then max discharge can never be 0, it's 
      // the manifestation of the 2^31 counter overflow bug in the SC0500
      // in that case, we won't tranmsit and keep the previous value sent to the inverter
      // the bug has a periodicity of 1h and a duration of a few seconds.
      // it takes to circumvent the bug on the coil level too to make this patch effective.
      if (pylontech.fix2_31 
        // when max charge is reported below zero (hence discharging?) => send the last good value
        // if 0, then probably battery full or out of voltage bound, DON'T reapply previous value
        || maxch < 0) {
        // use previous max charge value to avoid charge disruption (and ihccups on the grid export side)
        maxch = pylontech.max_charge_dA;
        corr = maxch + 30000;
        candata[4] = corr&0xFF;
        candata[5] = (corr>>8)&0xFF;
        // use previous value for max discharge, to avoid service disruption
        maxdis = pylontech.max_discharge_dA;
        corr = -maxdis + 30000;
        candata[6] = corr&0xFF;
        candata[7] = (corr>>8)&0xFF;
      }

      if (!pylontech.fix2_31 && maxdis <= 0) {
        master_log("fix max discharge invalid value\n");
        // reuse previous value
        maxdis = pylontech.max_discharge_dA;
        corr = -maxdis + 30000;
        candata[6] = corr&0xFF;
        candata[7] = (corr>>8)&0xFF;
      }

      pylontech.max_charge_dA = maxch;
      pylontech.max_discharge_dA = maxdis;

      // disallow discharge when battery is too low
      if (pylontech.soc <= 10) {
        master_log("soc < 10\n");
        // disable discharge
        corr = 0 + 30000;
        candata[6] = corr&0xFF;
        candata[7] = (corr>>8)&0xFF;
      }

      // compute max charge current, and adjust the value
      maxch = update_charge(maxch);
      corr = maxch + 30000;
      candata[4] = corr&0xFF;
      candata[5] = (corr>>8)&0xFF;
      pylontech.effective_charge_dA = maxch;

      // absolute max rating to avoid chemistry degradation
      // has not worked when maxch was negative
      if (pylontech.vcellmax >= knobs.max_charge_voltage
        // avoid charging when battery are too hot!
        || pylontech.tcellmax >= knobs.max_charge_temperature) {
        master_log("batt voltage dangerous (3.6V), or temperature too high, stop forced charge to avoid wearing\n");
        corr = 0 + 30000;
        candata[4] = corr&0xFF;
        candata[5] = (corr>>8)&0xFF;
        pylontech.effective_charge_dA = 0;

        // TODO, switch to automatic offgrid, and force offgrid! => solax has a bug continuing 
        // to charge when full and still connected on grid even if charge current is set to 0
        // ignore auto switch here. this is a measure for battery safety!
        offgrid_switch(1);
        // force auto switch mode. to avoid forcing
        auto_grid_connection = 1;
        // battery voltage too high, stop transcharge balancing
        transcharge_disable_all();
#ifdef HAVE_EXT_CHARGER
        // disable all forms of charge!
        charger.charge_enabled = 0;
#endif // HAVE_EXT_CHARGER
      }
      forward = 1;
      break;
    }

    case 0x4230:
      pylontech.vcellmax=S2LE(candata, 0)/100;
      pylontech.vcellmin=S2LE(candata, 2)/100;
      forward = 1;
      break;

    case 0x4240:
      pylontech.tcellmax=S2LE(candata, 0)-100;
      pylontech.tcellmin=S2LE(candata, 2)-100;
      forward = 1;
      break;

    case 0x4250:
      // pylontech.state      = candata[0]&0x7
      // pylontech.charge_req = candata[0]&0x8
      // pylontech.balance_req = candata[0]&0x10
      // pylontech.cycle      = U2LE(candata,1);
      // pylontech.error      = candata[3]
      // pylontech.alarm      = U2LE(candata,4);
      // pylontech.protection = U2LE(candata,6);
      snprintf((char*)tmp+16, sizeof(tmp)-16, "            | State=%d ChgReq=%d BalReq=%d Cycles=%d Error=%02x Alarm=%04x Prot=%04x\n", 
               candata[0]&0x3, (candata[0]&0x8)>>3,(candata[0]&0x10)>>4,
               U2LE(candata,1),
               candata[3],
               U2LE(candata,4),
               U2LE(candata,6),
               (pylontech.fix2_31)?" 2^31fix":"");
      master_log((char*)tmp+16);
      tmp[16] = 0;

      // wipe error status when fix 2^31 bug is detected
      if (pylontech.fix2_31) {
        memset(candata+3, 0, 5);
      }
      forward = 1;
      break;

    // Equipment information
    case 0x7320:
      pylontech.packs = candata[2];
      forward = 1;
      break;
      // no break, we forward it too
    case 0x7310:
    case 0x7330:
    case 0x7340:
      strcpy(tmp+16, "no process, forwarding\n");
      master_log((char*)tmp+16);
      tmp[16] = 0;
      forward=1;
      break;
  }
  return forward;
}