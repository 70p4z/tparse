//#define HAVE_FORCE_CHARGE_PCT_SOC
#define HAVE_WATCHDOG
#define HAVE_EXT_CHARGER
#define USBVCP USART3

// ensure 5 seconds steady state before making up a decision
#define WORKAROUND_SOLAX_INJECTION_SURGE // avoid too much charged battery to force inverter injecting surplus with clouds' surges
#define GRID_SWITCH_STATE_COUNT 3
#define GRID_DISCONNECT_SOC 75 // disconnect grid when over or equal
// best if equals to the value as the self use end of injection, so that in the end, the inverter is offgrid most of the time
#define GRID_CONNECT_SOC    20 

#define SOLAX_DISABLE_SELF_USE_PV_POWER 150
#define SOLAX_ENABLE_SELF_USE_PV_POWER 200

#define BMS_MAX_CELL_VOLTAGE_FOR_CURRENT_CHG_DV 36
#define BMS_MAX_CELL_VOLTAGE_FOR_PYLONTECH_DRIVE_DV 35
#define BMS_CELL_VOLTAGE_FOR_LIMITED_CHARGE_DV 35
// (~0.7A) => the dissipation of internal pack cell balancing (0.6*3.5 = 2 Watts) 
#define BMS_LIMITED_CHARGE_WATTAGE_PER_PACK (3500*15*5/10/1000) 
#define BMS_MAX_CHARGE_TEMPERATURE_DC 400 // in 0.1°C

// min difference to enable balancing charge
#define PYLONTECH_BALANCING_STOP_DIFF_MV 5
// max voltage to continue balancing
#define PYLONTECH_BALANCING_MAX_MV 3550
#define PYLONTECH_BALANCING_MIN_MV 3450
// optimal max wattage
#define PYLONTECH_BALANCING_OPTIMAL_WATTAGE 160

#define SOLAX_MAX_CHARGE_SOC 100 // limit battery wearing

// Solax X1G4 has an offset when respecting battery max charge current (maybe some inside DC bus consumption)
// 50W / 200 = 
// X1G4 for type 0x83 1.8A => 1.0A
// X1G4 for type 0x83 2.0A => 1.2A
// X1G4 for type 0x83 2.0A => 1.6A (when not output EPS power)
// X1G4 for type 0x83 1.0A => 0.1A
//#define SOLAX_BATTERY_CHARGE_OFFSET_DA 4 WITH NO EPS POWERING
//#define SOLAX_BATTERY_CHARGE_OFFSET_DA 8
#define SOLAX_BATTERY_CHARGE_OFFSET_DA 5 // reduce a bit to avoid 4.4kwh instead of 4kwh

// When charge is not possible anymore, use this value to make the inverter thinks it can charge and 
// avoid draining the battery when PV power is still available
#define SOLAX_BATTERY_CHARGE_DA_WORKAROUND_BATTERY_DRAIN 3 // 0.5A => more charging than discharging
// average offset accounted in the inverter
//#define SOLAX_BATT_FULL_BATTERY_WORKAROUND_WATTAGE 400 // 320 (rounded to 400 to ensure 1A compensation of the DC)
// allow to compensate charge up to a given value when value is full and ouse load is higher than currently balanced
#define SOLAX_BATT_FULL_BATTERY_WORKAROUND_WATTAGE 1000
#define COMPUTED_WATTAGE_AVG_COUNT 10
#define SOLAX_BATT_FULL_BATTERY_WORKAROUND_DELAY_MS (COMPUTED_WATTAGE_AVG_COUNT*2/4*1000)

//#define SUPPORT_PYLONTECH_RECONNECT // don't support reconnect to avoid loss of power in EPS, and no conflict with the pylotnech caching stuff
// #define SOLAX_REPLY_0x0100A001_AND_0x1801 # not needed on Solax X1G4

// required with version ARM=1.07+DSP=1.09, 
// not required with ARM=1.28+DSP=1.30 => optimization is far better with this firmware version
//#define HAVE_LOWLIGHT_OPT 

//#define BMS_PING
#define BMS_PING_INTERVAL_MS 5000

#define BMS_KIND_BLANK 0x50
#define BMS_KIND_BAK 0x51
#define BMS_KIND_REPT 0x52 // ok 4x H48050
#define BMS_KIND_SINOWATT 0x53
#define BMS_KIND_GOT 0x54
#define BMS_KIND_BLANK2 0x55
#define BMS_KIND_TP200 0x81
#define BMS_KIND_TP201 0x82 
#define BMS_KIND_TP202 0x83 // ok from 1 to 8 H48050 connected
// change depending on the battery configuration if it doesn't work out
#define BMS_KIND BMS_KIND_TP202

#define BMS_RECONNECT_DELAY 20000

#define SOLAX_PW_MODE_CHANGE_MIN_INTERVAL 10000 // avoid changing mode constantly
#define PYLONTECH_REPLY_TIMEOUT 1000 // auto reply when pylontech timeoutsn (reset or what not), to avoid the inverter to disconnect


#define SOLAX_PV_POWER_OPT_THRESHOLD_V 150
#define SOLAX_PV_POWER_OPT_THRESHOLD_W 25
#define SOLAX_GRID_EXPORT_OPT_THRESHOLD_W 50

#define SOLAX_DAY_THRESHOLD_V 50 // below 50v is considered NIGHT (with high exposure nights, it has been measured as much)
#define SOLAX_SELF_CONSUMPTION_MPPT_W 40 // observed inverter consumption when MPPT is working
#define SOLAX_SELF_CONSUMPTION_INVERTER_W 40 // observed inverter consumption with only inverter enabled (not system off)

#if 0
// before 80% of charge of battery, be conservative, and charge first
#define SOLAX_SELFUSE_MIN_BATTERY_SOC (GRID_DISCONNECT_SOC-1) // ensure starting selfuse before disconecting the grid to avoid glitch and mini outtage
#define SOLAX_SELFUSE_START_SOC (GRID_DISCONNECT_SOC-1)
#define SOLAX_SELFUSE_STOP_SOC (GRID_CONNECT_SOC+1)
#define HAVE_SOLAX_SWITCH_MODE
#define SOLAX_PV_CUTOFF_VOLTAGE_V 90
#define SOLAX_PV_CUTOFF_TIMEOUT 2000
#define SOLAX_PV_SWITCHON_TIMEOUT 3000
#endif // 0


#define DISPLAY_TIMEOUT 1000
// at least a CAN communication must have taken place within that period
#define TIMEOUT_LAST_ACTIVITY 180000
