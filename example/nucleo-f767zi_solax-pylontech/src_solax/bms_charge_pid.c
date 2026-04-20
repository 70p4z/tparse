#include "stddef.h"
#include "stdio.h"
#include "stdbool.h"
#include "stdint.h"

#include "bms_charge_pid.h"

#define MAX(x,y) ((x)>(y)?(x):(y))
#define MIN(x,y) ((x)<(y)?(x):(y))

#if 1
uint16_t bms_charge_pid(
    int16_t measured_current_dA,
    int16_t target_current_dA,
    uint16_t cell_max_voltage_mV,
    current_controller_pv_t *ctrl
)
{
    // ============================================================
    // LAYER 1 — SAFETY (HYSTERESIS + CHARGE ENABLE)
    // ============================================================

    if (cell_max_voltage_mV >= ctrl->v_stop_hyst_mV)
    {
        ctrl->charge_allowed = false;
    }
    else if (cell_max_voltage_mV < ctrl->v_start_hyst_mV)
    {
        ctrl->charge_allowed = true;
    }

    if (!ctrl->charge_allowed)
    {
        ctrl->last_allowed_dA = ctrl->min_current_offset_dA;

#ifdef X86
        printf("min current offset dA, charge not allowed\n");
#endif // X86
        return ctrl->last_allowed_dA;
    }

    // ============================================================
    // LAYER 2 — ENERGY LOOP (slow correction on real measurement)
    // ============================================================

    ctrl->avg_current_dA_x100 +=
        ((int32_t)measured_current_dA * 100
         - ctrl->avg_current_dA_x100) / 64;

    int16_t avg_current_dA =
        (int16_t)(ctrl->avg_current_dA_x100 / 100);

    int16_t energy_error_dA =
        target_current_dA - avg_current_dA;

    int16_t energy_correction_dA = 0;

    if (energy_error_dA > ctrl->energy_deadband_dA)
    {
        energy_correction_dA =
            (energy_error_dA * ctrl->ki_up_x100) / 100;
    }
    else if (energy_error_dA < -ctrl->energy_deadband_dA)
    {
        energy_correction_dA =
            (energy_error_dA * ctrl->ki_down_x100) / 100;
    }

    if (energy_correction_dA > ctrl->max_energy_step_dA)
        energy_correction_dA = ctrl->max_energy_step_dA;

    if (energy_correction_dA < -ctrl->max_energy_step_dA)
        energy_correction_dA = -ctrl->max_energy_step_dA;

    // ============================================================
    // LAYER 2.5 — BIAS LEARNER
    // ============================================================

    int16_t bias_error_dA =
        target_current_dA - avg_current_dA;

    ctrl->bias_correction_dA += bias_error_dA / 32;

    // if we are overshooting → reduce bias faster
    if (measured_current_dA > target_current_dA)
    {
        ctrl->bias_correction_dA -=
            (measured_current_dA - target_current_dA) / 8;
    }

    ctrl->bias_correction_dA = (ctrl->bias_correction_dA * 255) / 256;

    // ============================================================
    // LAYER 3 — FAST PID
    // ============================================================

    int16_t effective_target_dA = target_current_dA;

    int32_t hard_max_allowed =
        (int32_t)target_current_dA + ctrl->inverter_offset_dA;

    int16_t error_dA =
        effective_target_dA - measured_current_dA;

    // ============================================================
    // ANTI-WINDUP INTEGRATOR
    // ============================================================

    int32_t new_integral =
        (ctrl->integral_x100 * 31) / 32
        + (int32_t)error_dA * 100;

    int16_t max_allowed =
        effective_target_dA
        + ctrl->inverter_offset_dA
        + ctrl->bias_correction_dA;

    bool saturating_high =
        (ctrl->last_allowed_dA >= max_allowed);

    bool saturating_low =
        (ctrl->last_allowed_dA <= ctrl->min_current_offset_dA);

    // conditional integration
    if (!((saturating_high && error_dA > 0) ||
          (saturating_low && error_dA < 0)))
    {
        ctrl->integral_x100 = new_integral;
    }

    // fast unwind when sign flips
    if ((error_dA > 0 && ctrl->integral_x100 < 0) ||
        (error_dA < 0 && ctrl->integral_x100 > 0))
    {
        ctrl->integral_x100 /= 2;
    }

    // hard clamp (safety)
    int32_t integral_max = 200 * 100;

    if (ctrl->integral_x100 > integral_max)
        ctrl->integral_x100 = integral_max;

    if (ctrl->integral_x100 < -integral_max)
        ctrl->integral_x100 = -integral_max;

    int32_t p = ((int32_t)ctrl->kp_x100 * error_dA) / 100;
    int32_t i = ctrl->integral_x100 / 100;
    int32_t d = ((int32_t)ctrl->kd_x100 *
                 (error_dA - ctrl->prev_error_dA)) / 100;

    ctrl->prev_error_dA = error_dA;

    int32_t dynamic_dA = p + i + d;

    // ============================================================
    // CORE FIX — CAUSAL CONTROL (DO NOT USE MEASURED AS BASELINE)
    // ============================================================

    int32_t allowed_dA =
        //(int32_t)ctrl->last_allowed_dA
        measured_current_dA
        + dynamic_dA
        + energy_correction_dA
        + ctrl->bias_correction_dA;

    int16_t allowed_pre = allowed_dA;

    // ============================================================
    // FIX: HARD GLOBAL CEILING PROJECTION (CRITICAL)
    // ============================================================

    if (allowed_dA > hard_max_allowed)
    {
        allowed_dA = hard_max_allowed;

        // CRITICAL: stop integrator from “believing” overshoot is valid
        if (ctrl->integral_x100 > 0)
            ctrl->integral_x100 /= 2;
    }


    // ============================================================
    // ANTI-COLLAPSE PROTECTION
    // ============================================================

    // detect inverter collapse / invalid state
    if (measured_current_dA < 5 && ctrl->last_allowed_dA > 100)
    {
        // prevent integrator runaway during collapse
        ctrl->integral_x100 /= 2;

        // gently re-seed output instead of freezing at 1
        allowed_dA = ctrl->last_allowed_dA / 2;
    }

    // ============================================================
    // RATE LIMITING
    // ============================================================

    int32_t delta = allowed_dA - ctrl->last_allowed_dA;

    if (delta > ctrl->max_step_up_dA)
        delta = ctrl->max_step_up_dA;

    if (delta < -ctrl->max_step_down_dA)
        delta = -ctrl->max_step_down_dA;

    allowed_dA = ctrl->last_allowed_dA + delta;

    // ============================================================
    // MIN CURRENT SAFETY (NO HARD ZERO LOCK)
    // ============================================================

    if (allowed_dA < ctrl->min_current_offset_dA)
    {
        allowed_dA = ctrl->min_current_offset_dA;

        // prevent integrator wind-down into dead zone
        ctrl->integral_x100 /= 2;
    }

    // ============================================================
    // Avoid too high overshoots when panels/offset are too strict
    // ============================================================

    if (allowed_dA > max_allowed) {
        allowed_dA = max_allowed;

        // 🔥 ANTI-WINDUP: prevent integrator from pushing further up
        if (error_dA > 0)
        {
            ctrl->integral_x100 -= error_dA * 100;
        }
    }

    // ============================================================
    // SMART SAFETY: detect TRUE overshoot (not bias)
    // ============================================================

    int16_t predicted_dA = avg_current_dA;

    // only react if we are ABOVE both:
    // - target
    // - and what system "normally produces"
    if (measured_current_dA > target_current_dA &&
        measured_current_dA > predicted_dA + 1)
    {
        int16_t excess = measured_current_dA - target_current_dA;

        allowed_dA -= excess;   // moderate correction

        // prevent integrator from fighting this
        if (ctrl->integral_x100 > 0)
            ctrl->integral_x100 /= 2;
    }

    // ============================================================
    // SAFETY LOCK — MONOTONIC WHEN ABOVE TARGET
    // ============================================================

    if (measured_current_dA > target_current_dA)
    {
        if (allowed_dA > ctrl->last_allowed_dA)
        {
            allowed_dA = ctrl->last_allowed_dA;
        }
    }

    if (allowed_dA < ctrl->min_current_offset_dA)
    {
        allowed_dA = ctrl->min_current_offset_dA;
    }

    // ============================================================
    // STATE UPDATE
    // ============================================================

    ctrl->last_allowed_dA = (int16_t)allowed_dA;

#ifdef X86
    printf(
      "cellV=%d P=%d I=%d D=%d dyn=%d corr=%d last=%d meas=%d allowed_pre=%d allowed_post=%d bias=%d kp=%d ki_up=%d ki_down=%d kd=%d maxstpup=%d maxstepdown=%d maxenergystep=%d,  \n",
      cell_max_voltage_mV,
      p, i, d,
      dynamic_dA,
      energy_correction_dA,
      ctrl->last_allowed_dA,
      measured_current_dA,
      allowed_pre,
      allowed_dA,
      ctrl->bias_correction_dA,
      ctrl->kp_x100,
      ctrl->ki_up_x100,
      ctrl->ki_down_x100,
      ctrl->kd_x100,
      ctrl->max_step_up_dA,
      ctrl->max_step_down_dA,
      ctrl->max_energy_step_dA
    );
#endif // X86

    return (uint16_t)allowed_dA>0?allowed_dA:0;
}
#endif
