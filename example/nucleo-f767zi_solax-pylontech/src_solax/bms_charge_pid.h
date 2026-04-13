#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

typedef struct
{
    // ============================================================
    // LAYER 2 — ENERGY CONTROLLER (slow dynamics)
    // ============================================================

    // energy correction limits
    int16_t  max_energy_step_dA;
    int16_t energy_deadband_dA;

    // ============================================================
    // LAYER 2b — PLANT GAIN ADAPTATION (VERY SLOW)
    // ============================================================

    // estimated plant gain:
    // measured ≈ gain * allowed
    // stored in x1000 scale
    int32_t gain_x1000;
    bool gain_initialized;

    // ============================================================
    // LAYER 3 — FAST DYNAMICS (PID)
    // ============================================================

    int16_t kp_x100;
    int16_t kd_x100;
    // energy correction gain asymmetry
    int16_t ki_up_x100;     // deficit correction strength
    int16_t ki_down_x100;   // recovery damping strength

    int16_t  max_step_up_dA;
    int16_t  max_step_down_dA;

    // ============================================================
    // INVERTER OFFSET (hardware workaround)
    // ============================================================

    int16_t inverter_offset_dA;
    int16_t bias_correction_dA;

    // minimum enforced current due to inverter bug
    int16_t min_current_offset_dA;

    // ============================================================
    // LAYER 1 — PHYSICAL / SYSTEM LIMITS
    // ============================================================

    uint16_t v_start_hyst_mV;
    uint16_t v_stop_hyst_mV;

    // INTERNALS
    // energy average of measured current (fixed-point x100)
    int32_t avg_current_dA_x100;
    int32_t  integral_x100;
    int16_t  prev_error_dA;
    int16_t  last_allowed_dA;
    bool charge_allowed;

} current_controller_pv_t;

int16_t bms_charge_pid(
    int16_t measured_current_avg_dA,   // measured battery current
    int16_t target_current_dA,         // target battery current
    uint16_t cell_max_voltage_mV,
    bool soc_full,
    current_controller_pv_t *ctrl
);
