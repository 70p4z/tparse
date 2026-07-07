#include "globals.h"

extern uint32_t uwTick;

uint32_t can_inv_interp(uint32_t cid, size_t cid_bitlen, uint8_t* candata, size_t candata_len) {  
  uint32_t forward = 0;
  // other request from the inverter are discarded
  switch (cid) {
  case 0x1871:
    switch(candata[0]) {
      case 1:
        {
          uint8_t b[4];
          U4BE_ENCODE(b, 0, uwTick);
          master_log("            | Timestamp: ");
          master_log_hex(b, 4);
          master_log("\n");
        }
        inverter.powered_on = candata[2];
        // get data
        forward = 1;
        break;
#ifdef SUPPORT_PYLONTECH_RECONNECT
      case 2:
        // disconnect request (fault seen from the inverter's side)
        // if sent to the BMS, the SC0500 goes to slumber and no command can wake it up, 
        // have it disconnect after period of inactivity from the inverter instead.
        bms_reconnect_at = EXPIRE_IN(BMS_RECONNECT_DELAY);
        break;

      case 3:
        // not timestamp
        if (candata[1] != 6) {
          bms_reconnect_at = EXPIRE_IN(BMS_RECONNECT_DELAY);
        }
        break;

      case 5:
        // ping?
        break;
#endif // SUPPORT_PYLONTECH_RECONNECT
    }
    break;
  case 0x4200:
    forward=1;
    // ensure to send an equipment information request too
    if (candata[0] != 0x02) { 
      // request equipment info in case it's not updated everytime (to allow rewire of packs for balancing)
      can_bms_tx_log(0x4200, CAN_ID_EXTENDED_LEN, (uint8_t*)"\x02\x00\x00\x00\x00\x00\x00\x00", 8);
    }
  }

  return forward;
}