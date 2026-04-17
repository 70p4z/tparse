
#include "globals.h"


struct solax_s solax;
struct pylontech_s pylontech;
current_controller_pv_t pylontech_pid;

struct knobs_s knobs;

uint32_t auto_self_use_from_bat;
uint32_t auto_grid_connection;
uint32_t auto_bat_charge;
enum solax_forced_work_mode_e solax_forced_work_mode;
