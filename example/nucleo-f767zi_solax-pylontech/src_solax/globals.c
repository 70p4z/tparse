
#include "globals.h"


struct solax_s solax;
struct pylontech_s pylontech;
struct charger_s charger;
current_controller_pv_t pylontech_pid;
current_controller_pv_t charger_pid;

struct knobs_s knobs;

uint32_t auto_self_use_from_bat;
uint32_t auto_grid_connection;
uint32_t auto_bat_charge;
enum solax_forced_work_mode_e solax_forced_work_mode;


extern uint32_t uwTick;
uint32_t expire_in(uint32_t interval) {
  uint32_t timing = uwTick + interval;
  if (!timing) {timing++;}
  return timing;
}
