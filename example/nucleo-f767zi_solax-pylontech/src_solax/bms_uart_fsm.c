#include "main.h"
#include "tparse.h"
#include "stddef.h"
#include "stdio.h"
#include "stdbool.h"
#include "globals.h"


tparse_ctx_t tp_bms;
tparse_ctx_t tp_master;

#define BMS_UART_TIMEOUT 10000
#define BMS_UART_NEXT_INTERVAL 1000

enum bms_uart_state_e {
  BMS_UART_STATE_IDLE,
  BMS_UART_STATE_WAIT_USER,
  BMS_UART_STATE_WAIT_PWR,
  BMS_UART_STATE_WAIT_INFO,
  BMS_UART_STATE_WAIT_UNIT,
} bms_uart_state;

uint32_t bms_uart_next;
uint32_t bms_uart_timeout;

void bms_uart_init(void) {
  bms_uart_next = 0;
  bms_uart_timeout = 0;
  bms_uart_state = BMS_UART_STATE_IDLE;
  tparse_init(&tp_bms, uart_bms_buffer, sizeof(uart_bms_buffer), " %\n"); // add % as delim to parse percentage easier
  tparse_init(&tp_master, uart_usbvcp_buffer, sizeof(uart_usbvcp_buffer), "\n");
}

void bms_uart_update(void) {
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
  tparse_finger(&tp_master, sizeof(uart_usbvcp_buffer) - DMA_Stream_USBVCP->NDTR);
  // update data from the bms usart link
  tparse_finger(&tp_bms, sizeof(uart_bms_buffer) - DMA_Stream_BMS->NDTR);
  switch(bms_uart_state) {
    case BMS_UART_STATE_IDLE:
      if (tparse_has_line(&tp_master)) {
        size_t read = tparse_peek_line(&tp_master, (char*)tmp, sizeof(tmp));
        master_log("UARTBMSUSER >> ");
        master_log_mem(tmp, read);
        uart_select_intf(USART6);
        uart_send_mem(tmp, read);
        bms_uart_state = BMS_UART_STATE_WAIT_USER;
        // ensure discarding the line, as a read occured
        tparse_token_u32(&tp_master);
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
        // invalidate bmu values
        pylontech.bmu_idx = 0;
        bms_uart_state = BMS_UART_STATE_WAIT_PWR;
        bms_uart_next = EXPIRE_IN( BMS_UART_NEXT_INTERVAL);
      }
      bms_uart_timeout = EXPIRE_IN(BMS_UART_TIMEOUT);
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
            pylontech.precise_voltage_mV = (int32_t)val;
            int32_t valcurr = (int32_t)tparse_token_i32(&tp_bms); // Curr

            #ifdef WIP_PRECISE_CURRENT_FROM_CAPACITY
            // keep non 0 values (captured using mAh diff)
            if (valcurr != 0) 
            #endif // WIP_PRECISE_CURRENT_FROM_CAPACITY
            {
              pylontech.precise_current_mA = valcurr;
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
                  int32_t precise_current_mA = ((int32_t)mah - (int32_t)pylontech.precise_mAh) * ((int32_t)3600000) 
                                              / ((int32_t)uwTick - (int32_t)pylontech.precise_mAh_ts);
                  // anti oups due to timestamp counter rollover
                  // over +/-100mA, the current is accounted correctly in the CAN frame
                  if (precise_current_mA < 100 && precise_current_mA > -100) {
                    pylontech.precise_current_mA = precise_current_mA;
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
            pylontech.precise_wattage = ((int32_t)pylontech.precise_current_mA*((int32_t)pylontech.precise_voltage_mV/(int32_t)100))/(int32_t)10000;
            snprintf((char*)tmp, sizeof(tmp), "  voltage: %ld\n  current: %ld\n  wattage: %ld\n capacity: %ld\n", pylontech.precise_voltage_mV, pylontech.precise_current_mA, pylontech.precise_wattage, pylontech.precise_mAh);
            master_log((char*)tmp);
            // invariant check
            if ((pylontech.precise_current_mA < 0 && pylontech.precise_wattage > 0 )
              || (pylontech.precise_current_mA > 0 && pylontech.precise_wattage < 0 )) {
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
            bms_uart_timeout = EXPIRE_IN(BMS_UART_TIMEOUT);
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
          // farthest pack returned first
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
            bms_uart_state = BMS_UART_STATE_WAIT_UNIT; 
            bms_uart_timeout = EXPIRE_IN(BMS_UART_TIMEOUT);
            // init unit's min/max voltages
            pylontech.vcell_highest_tmp=0;
            pylontech.vcell_lowest_tmp=-1;
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
          if (idx >= 0 && idx < pylontech.bmu_idx_tmp && idx < PYLONTECH_MAX_BMUS) {
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

            if (VCELL_VALID(pylontech.bmu[idx].vlow) && VCELL_VALID(pylontech.bmu[idx].vhigh)) {
              // add absolute limit to filter out invlaid values
              if (pylontech.bmu[idx].vlow < pylontech.vcell_lowest_tmp) {
                pylontech.vcell_lowest_tmp = pylontech.bmu[idx].vlow;
              }
              // add absolute limit to filter out invlaid values
              if (pylontech.bmu[idx].vhigh > pylontech.vcell_highest_tmp) {
                pylontech.vcell_highest_tmp = pylontech.bmu[idx].vhigh;
              }
            }
          }
          // end?
          else if (read >= 3 && strstr((const char *)tmp, "\r$$") == (const char *)tmp) {
            tparse_discard(&tp_bms);
            master_log("UARTBMS end\n");
            // validate new count, so that i2c are not desynch
            pylontech.bmu_idx = pylontech.bmu_idx_tmp;
            // cache new values
            pylontech.vcell_lowest = pylontech.vcell_lowest_tmp;
            pylontech.vcell_highest = pylontech.vcell_highest_tmp;
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
        master_log("UARTBMS TIMEOUT\n");
        bms_uart_timeout = 0;
        bms_uart_next = 0; // immediate next command
        bms_uart_state = BMS_UART_STATE_IDLE; // send PWR again
        pylontech.precise_wattage = 0; // avoid relaying outdated data
        pylontech.bmu_idx = 0;
      }
      break;
  }
}