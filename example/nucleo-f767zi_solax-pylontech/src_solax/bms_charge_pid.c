#include "stddef.h"
#include "stdio.h"
#include "stdbool.h"
#include "stdint.h"

#include "bms_charge_pid.h"

#if 1
int16_t bms_charge_pid(
    int16_t measured_current_dA,
    int16_t target_current_dA,
    uint16_t cell_max_voltage_mV,
    bool soc_full,
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
        (int32_t)ctrl->last_allowed_dA
        + dynamic_dA
        + energy_correction_dA
        + ctrl->bias_correction_dA;

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

    int16_t allowed_before_clamp = allowed_dA;

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

    int16_t allowed_before_rate_limit = allowed_dA;

    // ============================================================
    // RATE LIMITING
    // ============================================================

    int32_t delta = allowed_dA - ctrl->last_allowed_dA;

    if (delta > ctrl->max_step_up_dA)
        delta = ctrl->max_step_up_dA;

    if (delta < -ctrl->max_step_down_dA)
        delta = -ctrl->max_step_down_dA;

    allowed_dA = ctrl->last_allowed_dA + delta;

    int16_t allowed_before_zero_antilock = allowed_dA;

    // ============================================================
    // MIN CURRENT SAFETY (NO HARD ZERO LOCK)
    // ============================================================

    if (allowed_dA < ctrl->min_current_offset_dA)
    {
        allowed_dA = ctrl->min_current_offset_dA;

        // prevent integrator wind-down into dead zone
        ctrl->integral_x100 /= 2;
    }

    int16_t allowed_before_clamping = allowed_dA;

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

    // ============================================================
    // STATE UPDATE
    // ============================================================

    ctrl->last_allowed_dA = (int16_t)allowed_dA;

#ifdef X86
    printf(
      "cellV=%d P=%d I=%d D=%d dyn=%d corr=%d last=%d meas=%d allowed_pre=%d allowed_pre_rl=%d allowed_pre_zal=%d allowed_pre_clamp=%d allowed_post=%d bias=%d\n",
      cell_max_voltage_mV,
      p, i, d,
      dynamic_dA,
      energy_correction_dA,
      ctrl->last_allowed_dA,
      measured_current_dA,
      allowed_before_clamp,
      allowed_before_rate_limit,
      allowed_before_zero_antilock,
      allowed_before_clamping,
      allowed_dA,
      ctrl->bias_correction_dA
    );
#endif // X86

    return (int16_t)allowed_dA;
}
#endif

/*
ChatGPT "explain the architecture of charge PID function":
Here is a **concise but maintainable architecture description** of your controller as it stands now. It’s written so someone coming later (or future you) can quickly understand *what each part is supposed to do and what must NOT be broken*.

---

# 🧠 Overall Control Philosophy

This controller regulates **battery charge current** using a **3-layer architecture**:

```text
Target (from battery logic)
        ↓
[Layer 3] Fast PID → reacts immediately
        ↓
[Layer 2] Energy loop → corrects long-term drift
        ↓
[Layer 2.5] Bias learner → compensates inverter/system offsets
        ↓
[Layer 1] Safety → enforces hard physical limits
        ↓
Allowed current (actuator command to inverter)
```

---

# ⚠️ Critical Design Rule (DO NOT BREAK)

> **Target applies to measured current, NOT allowed current.**
> Offsets (bias, inverter) must ONLY affect the actuator, never the target.

Breaking this rule leads to:

* deadlocks (cannot reach target)
* permanent offset errors
* unstable regulation

---

# 🔷 Layer 1 — Safety / Physical Constraints

### Role

Guarantee **battery protection and system validity**.

### Mechanisms

* Voltage hysteresis:

  * `v_stop_hyst_mV` → stop charging
  * `v_start_hyst_mV` → resume charging
* Forces minimum current when disabled:

  ```c
  allowed = min_current_offset_dA;
  ```

### Invariant

* Safety always overrides control loops.

---

# 🔷 Layer 2 — Energy Controller (Slow Loop)

### Role

Correct **long-term energy mismatch** (PV variability, slow drift).

### Mechanisms

* Low-pass filtered current:

  ```c
  avg_current_dA_x100 (τ ≈ 64 cycles)
  ```
* Deadband to avoid noise reaction
* Asymmetric gains:

  * `ki_up_x100` → react faster to deficit
  * `ki_down_x100` → slower recovery

### Output

```c
energy_correction_dA (bounded by max_energy_step_dA)
```

### Invariant

* Must remain **slow** (never destabilize PID)
* Acts as a **bias on top of PID**, not a replacement

---

# 🔷 Layer 2.5 — Bias Learner (Adaptive Offset)

### Role

Compensate **unknown and time-varying system offsets**:

* inverter nonlinearities
* hidden load interactions
* slow drift (~minutes scale)

### Mechanism

```c
bias += (target - avg_measured) / 32;
```

### Properties

* Very slow integrator
* Clamped (±100 dA)

### Invariant

* Must **always be allowed to evolve**
* Must **not be blocked by clamps**

---

# 🔷 Layer 3 — Fast PID (Dynamic Control)

### Role

Track **instantaneous target current**

### Input

```c
error = target - measured
```

⚠️ **No offsets here** (by design)

### Components

* P: immediate reaction
* I: removes steady-state error (with leak)
* D: dampens fast transitions

### Integral form

```c
I = leak + accumulation
```

### Invariant

* Must be **signed correctly**
* Must be the **primary fast actuator driver**

---

# 🔷 Actuator Composition (Core Control Law)

```c
allowed =
    last_allowed
  + dynamic (PID)
  + energy_correction
  + bias_correction
  + inverter_offset;
```

### Key property

> This is **causal control** (incremental), NOT absolute control.

---

# 🔷 Protections & Stabilizers

## Anti-collapse

Detects inverter failure / PV collapse:

```c
if (measured very low && allowed previously high)
```

→ reduces integral and halves output

---

## Rate limiting

Limits actuator speed:

```c
delta ≤ max_step_up/down
```

Prevents:

* oscillations
* inverter instability

---

## Minimum current clamp

Avoids inverter dead-zone:

```c
allowed ≥ min_current_offset_dA
```

---

## Overshoot clamp (IMPORTANT)

```c
allowed ≤ target + inverter_offset + bias
```

### Meaning

* Limits **excess actuator command**
* Still allows compensation for offsets

### Invariant

> Must NEVER clamp to `target` alone
> Must include offsets, otherwise system deadlocks

---

# 🔷 Internal State Summary

| Variable              | Role                          |
| --------------------- | ----------------------------- |
| `last_allowed_dA`     | actuator memory (causal base) |
| `integral_x100`       | fast error accumulation       |
| `avg_current_dA_x100` | slow measurement filter       |
| `bias_correction_dA`  | learned system offset         |
| `charge_allowed`      | safety state                  |

---

# 🧭 System Behavior Summary

### Normal operation

* PID tracks target
* Bias compensates inverter quirks
* Energy loop smooths long-term mismatch

### With load disturbance

* Measured drops
* PID increases allowed above target
* Bias slowly adapts

### With PV drop

* Measured collapses
* Anti-collapse prevents runaway

### Near full battery

* Voltage hysteresis stops charging
* Resume occurs cleanly

---

# 🚨 Known Sensitive Points (for maintenance)

1. **Do not clamp `allowed` to `target`**
2. **Do not inject offsets into PID target**
3. **Do not disable bias learning during saturation**
4. **Keep energy loop slower than PID**
5. **Preserve causal form (`last_allowed` based)**

---

# ✅ Mental Model (keep this)

```text
PID → fast brain
Energy loop → long-term balance
Bias → reality correction
Safety → hard guardrails
```

---

# 📊 1. Architecture Diagram (Control Flow)

```text
                 ┌──────────────────────────────┐
                 │   Battery / System Logic     │
                 │  (target_current_dA)         │
                 └──────────────┬───────────────┘
                                │
                                ▼
                     ┌────────────────────┐
                     │   LAYER 3 — PID    │
                     │ error = tgt - meas │
                     │ P + I + D          │
                     └─────────┬──────────┘
                               │ dynamic_dA
                               ▼
        ┌────────────────────────────────────────────┐
        │        ACTUATOR COMPOSITION (CORE)         │
        │                                            │
        │ allowed = last_allowed                     │
        │         + dynamic_dA       (fast)          │
        │         + energy_corr      (slow)          │
        │         + bias_corr        (adaptive)      │
        │         + inverter_offset  (static)        │
        └──────────────┬─────────────────────────────┘
                       │
                       ▼
        ┌────────────────────────────────────────────┐
        │         PROTECTIONS / LIMITS               │
        │  - anti-collapse                          │
        │  - rate limiting                          │
        │  - min current (no dead zone)             │
        │  - overshoot clamp (WITH offsets)         │
        └──────────────┬─────────────────────────────┘
                       │
                       ▼
                allowed_current_dA
                       │
                       ▼
                ┌──────────────┐
                │   INVERTER   │
                │ (non-ideal)  │
                └──────┬───────┘
                       │
                       ▼
                measured_current_dA
                       │
         ┌─────────────┼────────────────┐
         │             │                │
         ▼             ▼                ▼

 ┌──────────────┐  ┌──────────────┐  ┌──────────────┐
 │ LAYER 2      │  │ LAYER 2.5    │  │ LAYER 1      │
 │ Energy loop  │  │ Bias learner │  │ Safety       │
 │ (avg filter) │  │ (slow adapt) │  │ (voltage)    │
 └──────────────┘  └──────────────┘  └──────────────┘
```

---

# 🧠 Key structural insight (from diagram)

* **PID does NOT know about offsets**
* **Offsets are injected AFTER PID**
* **Measured feeds ALL loops (fast + slow + adaptive)**

---

# ⏱️ 2. Timing Model (Very Important for Your System)

Your system actually runs **4 different time scales simultaneously**:

---

## ⚡ Fast loop — PID (every cycle)

```text
Timescale: 10–100 ms (or your control tick)

Reacts to:
- instantaneous error (target - measured)

Effect:
- immediate correction (dynamic_dA)

Risk:
- oscillations if too aggressive
```

---

## 🟡 Medium loop — Rate limiting

```text
Timescale: per step constraint

Limits:
- how fast allowed_dA can change

Effect:
- stabilizes inverter behavior
- prevents brutal jumps

Important:
- defines system “smoothness”
```

---

## 🔵 Slow loop — Energy controller

```text
Timescale: ~64 cycles (low-pass filter)

React to:
- average mismatch

Effect:
- compensates PV variability
- prevents long-term drift

Behavior:
- asymmetric (faster up than down)
```

---

## 🟣 Very slow loop — Bias learner

```text
Timescale: ~32 cycles per small step
→ minutes in real system (your 5-min drift)

React to:
- persistent error

Effect:
- compensates:
  - inverter nonlinearities
  - hidden loads
  - slow drift

Critical:
- must NOT be blocked by clamps
```

---

# 🧭 Combined Timing Behavior

```text
Time →
──────────────────────────────────────────────

PID            █ █ █ █ █ █ █ █ █ █ █ (instant)

Rate limit     ▒ ▒ ▒ ▒ ▒ ▒ ▒ ▒ ▒ ▒ ▒ (step smoothing)

Energy loop    ░░░░░░░░░░░░░░░░░░░ (slow drift correction)

Bias learner   ▓───────▓───────▓──── (very slow adaptation)
```

---

# 🔥 What this explains in your real system

## 🟠 Your “5-minute inverter drift”

Now clearly explained:

* bias learner slowly adapts → increases allowed
* after a while → overshoots → needs reduction
* cycle repeats

👉 This is **normal**, but can be improved with:

* better bias time constant
* or slight damping

---

## 🟢 Cloud / PV fluctuation

* PID reacts instantly
* energy loop stabilizes average
* no long-term drift

---

## 🔴 Load disturbance (your tricky case)

* measured goes negative
* PID pushes allowed ABOVE target
* bias learns required offset
* system stabilizes

---

# 🧠 Golden Mental Model (with time)

```text
PID         = reflexes
Energy loop = metabolism
Bias        = learning
Safety      = survival instinct
```

---

# ⚠️ Maintenance Warnings (Timing-related)

### 1. If PID too strong

→ oscillations / inverter instability

### 2. If energy loop too fast

→ fights PID (bad interaction)

### 3. If bias too fast

→ oscillatory “offset chasing”

### 4. If bias too slow

→ 5-min drift becomes worse

*/
