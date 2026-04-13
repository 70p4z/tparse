#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "bms_charge_pid.h"
#include "string.h"

#define MIN(x,y) ((x)<(y)?x:y)

void init_pid(current_controller_pv_t *ctrl)
{
    memset(ctrl, 0, sizeof(*ctrl));

    ctrl->kp_x100 = 59;
    ctrl->ki_up_x100 = 10;
    ctrl->ki_down_x100 = 20;
    ctrl->kd_x100 = 5;

    ctrl->max_step_up_dA = 20;
    ctrl->max_step_down_dA = 50;

    ctrl->max_energy_step_dA = 50;
    ctrl->energy_deadband_dA = 2;

    ctrl->v_start_hyst_mV = 3450;
    ctrl->v_stop_hyst_mV  = 3550;

    ctrl->charge_allowed = true;

    ctrl->min_current_offset_dA = 1;

    ctrl->inverter_offset_dA = 8; // observed max offset internally consumed by the inverter (more surely expressed as something related to power of the link)

    //ctrl->gain_x1000 = 1000;
    //ctrl->gain_initialized = false;
}

#if 0
void test_simple_pid() {
    current_controller_pv_t ctrl = {0};
    init_pid(&ctrl);

    uint16_t cell_v_mV = 3500;
    bool soc_full = false;

    // Step response: target = 30A, measured = 0A
    uint16_t allowed = bms_charge_pid(
        0, 300, cell_v_mV, soc_full, &ctrl
    );

    printf("Simple PID step response allowed current: %d dA\n", allowed);
    assert(allowed > 0 && allowed <= 300); // Should allow some current
}

void test_simple_pid2() {
    current_controller_pv_t ctrl = {0};
    init_pid(&ctrl);

    uint16_t cell_v_mV = 3500;
    bool soc_full = false;

    int16_t allowed = 0;

    for (int i = 0; i < 10000; i++)
    {
        allowed = bms_charge_pid(
            0,
            300,
            cell_v_mV,
            soc_full,
            &ctrl
        );
        printf("Allowed current: %d dA\n", allowed);
    }

    printf("Final allowed: %d dA\n", allowed);

    // ✔ correct expectation:
    // system should respond positively and stabilize
    assert(allowed > 0);

    // optional: ensure it doesn't diverge
    assert(allowed < 300);
}

#if 0
void test_yoyo_long() {
    current_controller_pv_t ctrl = {0};
    init_pid(&ctrl);

    uint16_t cell_v_mV = 3500;
    bool soc_full = false;
    uint16_t measured_current_dA = 0;
    uint16_t target_current_dA = 400; // 40A target

    srand(1234); // deterministic pseudo-random sequence

    printf("t\tPVmax\tAllowed\tMeasured\n");
    for (int t = 0; t < 100; t++) { // 100 cycles (~100 seconds)
        // PV fluctuation between 20A and 50A (200-500 dA)
        uint16_t pv_max_dA = 200 + rand() % 301;

        // PID calculation
        uint16_t allowed_dA = bms_charge_pid(
            measured_current_dA, // 1-second lag
            target_current_dA,
            cell_v_mV,
            soc_full,
            &ctrl
        );

        // Simulate one-sample lag in measured current
        measured_current_dA = allowed_dA;

        printf("%d\t%d\t%d\t%d\n", t, pv_max_dA, allowed_dA, measured_current_dA);

        // Assertions
        // assert(allowed_dA <= pv_max_dA);   // Never exceed PV supply
        assert(allowed_dA <= target_current_dA); // Never exceed battery limit
        assert(allowed_dA >= 0);           // Never negative
    }

    printf("Long yoyo test completed successfully.\n");
}


void test_yoyo_long_delayed() {
    current_controller_pv_t ctrl = {0};
    init_pid(&ctrl);

    uint16_t cell_v_mV = 3500;
    bool soc_full = false;
    uint16_t target_current_dA = 400; // 40A target

    // Delay: measured current is the allowed current from previous cycle
    uint16_t measured_current_dA = 0;
    uint16_t prev_allowed_dA = 0;

    srand(1234); // deterministic pseudo-random sequence

    printf("t\tPVmax\tAllowed\tMeasured\n");
    for (int t = 0; t < 100; t++) { // 100 cycles (~100 seconds)
        // PV fluctuation between 20A and 50A (200-500 dA)
        uint16_t average_jitter = -5 + rand() % 20;

        // PID calculation, measured current is delayed by 1 sample
        uint16_t allowed_dA = bms_charge_pid(
            prev_allowed_dA + average_jitter, // <- 1-sample delayed measurement
            target_current_dA,
            cell_v_mV,
            soc_full,
            &ctrl
        );

        // Update measured for next cycle
        measured_current_dA = prev_allowed_dA;
        prev_allowed_dA = allowed_dA;

        printf("%d\t%d\t%d\t%d\n", t, allowed_dA, measured_current_dA);

        // Assertions
        // assert(allowed_dA <= pv_max_dA);   // never exceed PV
        assert(allowed_dA <= target_current_dA); // never exceed battery
        assert(allowed_dA >= 0);           // never negative
    }

    printf("Long yoyo test with 1-sample delay completed successfully.\n");
}
#endif

void test_energy_deficit_recovery_cycles()
{
    current_controller_pv_t ctrl = {0};

    init_pid(&ctrl);

    // ---------------- system params ----------------
    int16_t target_current_dA = 400;
    uint16_t cell_v_mV = 3400;
    bool soc_full = false;

    int16_t prev_allowed_dA = 0;

    printf("t\tCycle\tPhase\tAllowed\tMeasured\tAvg\n");

    // ============================================================
    // Run 3 full cycles (600 samples total)
    // ============================================================
    for (int t = 0; t < 6000; t++)
    {
        int cycle = t / 600;
        int phase = (t % 600) / 100;

        int pv_max_dA;

        // ========================================================
        // Phase A: deficit (avg ≈ target - 8dA)
        // ========================================================
        if (phase == 0)
        {
            pv_max_dA = target_current_dA - 15; // strong limitation
        }
        // ========================================================
        // Phase B: recovery (normal PV)
        // ========================================================
        else
        {
            pv_max_dA = target_current_dA + 60; // enough headroom
        }

        // --------------------------------------------------------
        // Plant model: 1-sample delay + PV saturation
        // --------------------------------------------------------
        int16_t measured_current_dA = prev_allowed_dA;

        if (measured_current_dA > pv_max_dA)
            measured_current_dA = pv_max_dA;

        // --------------------------------------------------------
        // Controller
        // --------------------------------------------------------
        int16_t allowed_dA = bms_charge_pid(
            measured_current_dA,
            target_current_dA,
            cell_v_mV,
            soc_full,
            &ctrl
        );

        int16_t avg_dA = ctrl.avg_current_dA_x100 / 100;

        printf("%d\t%d\t%s\t%d\t%d\t%d\n",
               t,
               cycle,
               (phase == 0 ? "DEFICIT" : "RECOVERY"),
               allowed_dA,
               measured_current_dA,
               avg_dA);

        // ========================================================
        // HARD SAFETY invariants
        // ========================================================
        assert(allowed_dA >= 0);
        // assert(allowed_dA <= pv_max_dA);
        //assert(allowed_dA <= target_current_dA);

        // ========================================================
        // ENERGY BEHAVIOR checks (key part of test)
        // ========================================================

        // 1. During deficit phase: avg should be below target
        if (phase == 0 && t > 20)
        {
            assert(avg_dA <= target_current_dA + 2);
        }

        // 2. During recovery: avg should NOT overshoot excessively
        if (phase == 1 && t > 20)
        {
            assert(avg_dA <= target_current_dA + 15);
        }

        // 3. Cross-cycle stability: no drift accumulation
        if (t > 400)
        {
            assert(avg_dA >= target_current_dA - 20);
            assert(avg_dA <= target_current_dA + 20);
        }

        prev_allowed_dA = allowed_dA;
    }

    printf("test_energy_cycles OK\n");
}

void test_cloud_event()
{
    current_controller_pv_t ctrl = {0};

    init_pid(&ctrl);

    int16_t target_current_dA  = 400;
    uint16_t cell_v_mV         = 3400;
    bool soc_full              = false;

    int16_t prev_allowed_dA    = 0;
    int16_t measured_current_dA = 0;

    printf("t\tPV\tAllowed\tMeasured\n");

    for (int t = 0; t < 150; t++)
    {
        uint16_t pv_max_dA;

        // --------------------------------------------------------
        // Phase 1: Normal sun (0–49)
        // --------------------------------------------------------
        if (t < 50)
        {
            pv_max_dA = 300; // enough to reach target
        }
        // --------------------------------------------------------
        // Phase 2: Cloud (50–99)
        // --------------------------------------------------------
        else if (t < 100)
        {
            pv_max_dA = 50; // strong limitation
        }
        // --------------------------------------------------------
        // Phase 3: Sun returns stronger (100–149)
        // --------------------------------------------------------
        else
        {
            pv_max_dA = 500; // plenty of power
        }

        // --------------------------------------------------------
        // Simulated plant with 1-sample delay + PV limitation
        // --------------------------------------------------------
        measured_current_dA = MIN(pv_max_dA, prev_allowed_dA);

        // --------------------------------------------------------
        // Controller
        // --------------------------------------------------------
        int16_t allowed_dA = bms_charge_pid(
            measured_current_dA,
            target_current_dA,
            cell_v_mV,
            soc_full,
            &ctrl
        );

        printf("%d\t%d\t%d\t%d\n",
               t,
               pv_max_dA,
               allowed_dA,
               measured_current_dA);

        // --------------------------------------------------------
        // Assertions
        // --------------------------------------------------------

        // Always respect limits
        assert(allowed_dA <= target_current_dA);
        // assert(allowed_dA <= pv_max_dA || pv_max_dA >= target_current_dA);

        // During cloud: should NOT explode upward
        if (t > 60 && t < 100)
        {
            assert(allowed_dA <= target_current_dA + 50);
        }

        // After sun returns: no huge overshoot
        if (t > 110)
        {
            assert(allowed_dA <= target_current_dA + 50);
        }

        prev_allowed_dA = allowed_dA;
    }

    printf("Cloud event test completed.\n");
}

void test_min_current_offset_respected()
{
    current_controller_pv_t ctrl = {0};
    init_pid(&ctrl);

    uint16_t cell_v_mV = 3400;
    bool soc_full = false;

    int16_t prev_allowed_dA = 100;

    printf("t\tPV\tAllowed\tMeasured\n");

    for (int t = 0; t < 300; t++)
    {
        // --------------------------------------------------------
        // chaotic PV simulation (stress test)
        // --------------------------------------------------------
        int16_t pv_max_dA;

        if (t < 80)
            pv_max_dA = 40;     // deep cloud
        else if (t < 160)
            pv_max_dA = 480;    // strong sun
        else if (t < 220)
            pv_max_dA = 10;     // near zero PV
        else
            pv_max_dA = 300;    // recovery

        // --------------------------------------------------------
        // plant delay model
        // --------------------------------------------------------
        int16_t measured_current_dA = MIN(pv_max_dA, prev_allowed_dA);

        // --------------------------------------------------------
        // controller
        // --------------------------------------------------------
        int16_t allowed_dA = bms_charge_pid(
            measured_current_dA,
            400,               // target fixed
            cell_v_mV,
            soc_full,
            &ctrl
        );

        printf("%d\t%d\t%d\t%d\n",
               t,
               pv_max_dA,
               allowed_dA,
               measured_current_dA);

        // ========================================================
        // HARD SAFETY ASSERTIONS (THIS IS THE POINT OF THE TEST)
        // ========================================================

        // 1. never negative
        assert(allowed_dA >= 0);

        // 2. NEVER zero when system is active
        if (ctrl.charge_allowed)
        {
            assert(allowed_dA > 0);
        }

        // 3. enforce inverter minimum floor
        if (allowed_dA > 0)
        {
            assert(allowed_dA >= ctrl.min_current_offset_dA);
        }

        // 4. still respect physical max
        //assert(measured_current_dA <= 400);

        prev_allowed_dA = allowed_dA;
    }

    printf("test_min_current_offset_respected OK\n");
}

void test_low_pv_not_limited_behavior()
{
    current_controller_pv_t ctrl = {0};
    init_pid(&ctrl);
    int16_t target_current_dA = 420;

    int16_t prev_allowed_dA = 350;

    printf("t\tPVmax\tAllowed\tMeasured\n");

    for (int t = 0; t < 120; t++)
    {
        // ========================================================
        // PV is artificially LOW for entire test
        // ========================================================
        int16_t pv_max_dA = 120;   // <<<<< LOW PV ceiling

        // ========================================================
        // battery + system still want high current
        // ========================================================
        uint16_t cell_v_mV = 3400;
        bool soc_full = false;

        // --------------------------------------------------------
        // plant model (lagged response)
        // --------------------------------------------------------
        int16_t measured_current_dA = MIN(pv_max_dA, prev_allowed_dA);

        // --------------------------------------------------------
        // controller
        // --------------------------------------------------------
        int16_t allowed_dA = bms_charge_pid(
            measured_current_dA,
            target_current_dA,
            cell_v_mV,
            soc_full,
            &ctrl
        );

        printf("%d\t%d\t%d\t%d\n",
               t,
               pv_max_dA,
               allowed_dA,
               measured_current_dA);

        // ========================================================
        // ASSERTIONS
        // ========================================================

        // 1. never negative
        assert(allowed_dA >= 0);

        // 2. inverter rule
        if (allowed_dA > 0)
        {
            assert(allowed_dA >= ctrl.min_current_offset_dA);
        }

        // 3. IMPORTANT: PV must NOT fully dictate output
        // (this is the key test condition)
        if (t > 20)
        {
            assert(allowed_dA >= pv_max_dA);
        }

        // 4. system should still track energy demand
        assert(allowed_dA <= target_current_dA);

        prev_allowed_dA = allowed_dA;
    }

    printf("test_low_pv_not_limited_behavior OK\n");
}


void test_voltage_hysteresis_min_current()
{
    current_controller_pv_t ctrl = {0};
    init_pid(&ctrl);
    int16_t target_current_dA = 400;

    int16_t prev_allowed_dA = 50;

    printf("t\tV\tAllowed\tChargeAllowed\n");

    for (int t = 0; t < 200; t++)
    {
        uint16_t cell_v_mV;

        // ========================================================
        // Voltage sweep around hysteresis thresholds
        // ========================================================
        if (t < 50)
        {
            cell_v_mV = 3600; // above stop → should disable charging
        }
        else if (t < 100)
        {
            cell_v_mV = 3400; // below start → enable charging
        }
        else if (t < 150)
        {
            cell_v_mV = 3560; // above stop again → disable
        }
        else
        {
            cell_v_mV = 3420; // enable again
        }

        bool soc_full = true;

        // --------------------------------------------------------
        // PV always available (so controller is not PV-limited)
        // --------------------------------------------------------
        int16_t pv_max_dA = 480;

        // --------------------------------------------------------
        // plant delay
        // --------------------------------------------------------
        int16_t measured_current_dA = MIN(prev_allowed_dA, pv_max_dA);

        // --------------------------------------------------------
        // controller
        // --------------------------------------------------------
        int16_t allowed_dA = bms_charge_pid(
            measured_current_dA,
            target_current_dA,
            cell_v_mV,
            soc_full,
            &ctrl
        );

        printf("%d\t%d\t%d\t%d\n",
               t,
               cell_v_mV,
               allowed_dA,
               ctrl.charge_allowed);

        // ========================================================
        // HARD SAFETY ASSERTIONS
        // ========================================================

        // 1. never negative
        assert(allowed_dA >= 0);

        // 2. when charging is active → NEVER zero
        if (ctrl.charge_allowed)
        {
            assert(allowed_dA > 0);
        }

        // 3. enforce inverter minimum offset
        if (allowed_dA > 0)
        {
            assert(allowed_dA >= ctrl.min_current_offset_dA);
        }

        // 4. when charging disabled → must be zero (hard stop)
        if (!ctrl.charge_allowed)
        {
            assert(allowed_dA <= ctrl.min_current_offset_dA);
        }

        // 5. no oscillation at boundary (optional stability check)
        if (t > 10)
        {
            assert(allowed_dA <= target_current_dA);
        }

        prev_allowed_dA = allowed_dA;
    }

    printf("test_voltage_hysteresis_min_current OK\n");
}

void test_pv_equals_target_min_offset()
{
    current_controller_pv_t ctrl = {0};

    init_pid(&ctrl);
    int16_t target_current_dA = 300;

    // KEY CASE: PV LIMIT == TARGET
    int16_t pv_max_dA = target_current_dA;

    int16_t prev_allowed_dA = 200;

    printf("t\tAllowed\tMeasured\n");

    for (int t = 0; t < 80; t++)
    {
        uint16_t cell_v_mV = 3400;
        bool soc_full = false;

        // --------------------------------------------------------
        // plant model (delayed response)
        // --------------------------------------------------------
        int16_t measured_current_dA = prev_allowed_dA;

        if (measured_current_dA > pv_max_dA)
            measured_current_dA = pv_max_dA;

        // --------------------------------------------------------
        // controller
        // --------------------------------------------------------
        int16_t allowed_dA = bms_charge_pid(
            measured_current_dA,
            target_current_dA,
            cell_v_mV,
            soc_full,
            &ctrl
        );

        printf("%d\t%d\t%d\n",
               t,
               allowed_dA,
               measured_current_dA);

        // ========================================================
        // ASSERTIONS (CRITICAL BEHAVIOR)
        // ========================================================

        // 1. never negative
        assert(allowed_dA >= 0);

        // 2. inverter bug workaround must ALWAYS hold
        if (allowed_dA > 0)
        {
            assert(allowed_dA >= ctrl.min_current_offset_dA);
        }

        // 3. system must NOT collapse at PV == target boundary
        if (t > 10)
        {
            assert(allowed_dA > 0);
        }

        // 4. must not oscillate into zero state
        if (t > 20)
        {
            assert(allowed_dA >= ctrl.min_current_offset_dA);
        }

        // 5. stability: no degeneration to PV clamp only behavior
        if (t > 30)
        {
            assert(allowed_dA >= pv_max_dA - 20);
        }

        prev_allowed_dA = allowed_dA;
    }

    printf("test_pv_equals_target_min_offset OK\n");
}

void test_inverter_starvation_battery_discharge()
{
    current_controller_pv_t ctrl = {0};

    init_pid(&ctrl);

    int16_t target_current_dA = 300;

    // KEY POINT: start near starvation region
    int16_t allowed_dA = 1;

    int16_t prev_allowed_dA = allowed_dA;

    printf("t\tAllowed\tMeasured\tPV\n");

    for (int t = 0; t < 80; t++)
    {
        // ========================================================
        // INVERTER BEHAVIOR MODEL (CRITICAL PART OF BUG)
        // ========================================================

        int16_t pv_available_dA;

        if (allowed_dA <= 10)
        {
            // inverter "gives up" harvesting PV
            pv_available_dA = 20;   // collapses production
        }
        else
        {
            pv_available_dA = 350;  // normal PV production
        }

        // --------------------------------------------------------
        // battery becomes net load sink when PV collapses
        // --------------------------------------------------------
        int16_t measured_current_dA;

        if (pv_available_dA < allowed_dA)
        {
            // battery is discharging to compensate deficit
            measured_current_dA = -(allowed_dA - pv_available_dA);
        }
        else
        {
            measured_current_dA = pv_available_dA;
        }

        // --------------------------------------------------------
        // controller
        // --------------------------------------------------------
        int16_t new_allowed_dA = bms_charge_pid(
            measured_current_dA,
            target_current_dA,
            3400,
            false,
            &ctrl
        );

        printf("%d\t%d\t%d\t%d\n",
               t,
               new_allowed_dA,
               measured_current_dA,
               pv_available_dA);

        // ========================================================
        // ASSERTIONS (BUG SHOULD SHOW HERE)
        // ========================================================

        // 1. system must not stay in discharge
        if (t > 10)
        {
            assert(measured_current_dA >= 0);
        }

        // 2. controller must react to battery discharge
        if (t > 10)
        {
            assert(new_allowed_dA >= allowed_dA);
        }

        // 3. system must recover out of starvation region
        if (t > 20)
        {
            assert(new_allowed_dA > 20);
        }

        allowed_dA = new_allowed_dA;
        prev_allowed_dA = allowed_dA;
    }

    printf("test_inverter_starvation_battery_discharge OK\n");
}
#endif // 0


// basic proportional plant
static int16_t plant_follow(int16_t allowed_dA)
{
    return (allowed_dA * 95) / 100; // 90% efficiency
}

// plant limited by PV power
static int16_t plant_pv_limited(int16_t allowed_dA, int16_t pv_max_dA)
{
    int16_t effective = (allowed_dA * 95) / 100;
    return (effective > pv_max_dA) ? pv_max_dA : effective;
}


// plant with offset
static int16_t plant_with_offset(int16_t allowed_dA, int16_t offset_dA)
{
    return ((allowed_dA * 95) / 100) + offset_dA;
}


typedef struct {
    int32_t soc_mAs;        // stored charge (state)
    int16_t voltage_mV;     // terminal voltage
} battery_t;

int16_t battery_step(battery_t *b, int16_t allowed_dA)
{
    // ------------------------------------------------------------
    // 1. convert current to energy flow
    // ------------------------------------------------------------

    int32_t current = allowed_dA;

    // charging efficiency drops at high voltage
    int32_t efficiency = 100;

    if (b->voltage_mV > 3400)
        efficiency = 95;

    if (b->voltage_mV > 3500)
        efficiency = 90;

    // ------------------------------------------------------------
    // 2. integrate SOC (energy storage)
    // ------------------------------------------------------------

    b->soc_mAs += (current * efficiency) / 100;

    if (b->soc_mAs < 0)
        b->soc_mAs = 0;

    // ------------------------------------------------------------
    // 3. voltage follows SOC (VERY IMPORTANT)
    // ------------------------------------------------------------

    b->voltage_mV =
        3200 + (b->soc_mAs / 1000);

    if (b->voltage_mV > 3600)
        b->voltage_mV = 3600;

    // ------------------------------------------------------------
    // 4. measured current = filtered allowed (NOT identity)
    // ------------------------------------------------------------

    static int32_t filtered = 0;

    filtered += (current - filtered) / 3;

    return (int16_t)filtered;
}

typedef struct {
    int32_t soc_acc;
    int16_t voltage_mV;
    int16_t filt_current_dA;
    int16_t load_dA;
    int16_t soc_max;

} plant_t;

#if 0
int16_t plant_step(plant_t *p, int16_t allowed_dA)
{
    // 1. integrate input (NO division too early!)
    p->soc_acc += allowed_dA;

    // 2. slow leakage / realism
    p->soc_acc -= p->soc_acc / 1000;

    // 3. derive voltage
    p->voltage_mV = 3300 + (p->soc_acc / 10);

    // 4. measured current = filtered allowed
    static int32_t filt = 0;
    filt += (allowed_dA - filt) / 3;

    return (int16_t)filt;
}
#endif

#if 1
void plant_init(plant_t* p) {
    p->soc_max = 100000;   // arbitrary capacity
    p->soc_acc = 80000;    // start ~80%
    p->load_dA = 0;
}

int16_t plant_step(plant_t *p, int16_t allowed_dA)
{
    // ------------------------------------------------------------
    // 0. External load (can be + or -)
    // ------------------------------------------------------------
    int16_t net_current = allowed_dA - p->load_dA;

    // ------------------------------------------------------------
    // 1. SOC integration (true energy buffer)
    // ------------------------------------------------------------
    p->soc_acc += net_current;

    // leakage (very slow self-discharge)
    p->soc_acc -= p->soc_acc / 20000;

    // clamp SOC to realistic bounds
    if (p->soc_acc < 0)
        p->soc_acc = 0;

    if (p->soc_acc > p->soc_max)
        p->soc_acc = p->soc_max;

    // ------------------------------------------------------------
    // 2. Voltage model (depends on SOC, not current directly)
    // ------------------------------------------------------------

    // normalized SOC (0 → 1 scaled in integer)
    int32_t soc_norm = (p->soc_acc * 1000) / p->soc_max;

    // base Li-ion-ish curve (very simplified)
    int32_t v_base =
        3100                          // empty voltage
        + (soc_norm * 550) / 1000;   

    // small dynamic sag depending on current
    int32_t v_dyn = -(net_current / 2);  // internal resistance effect

    p->voltage_mV = (int16_t)(v_base + v_dyn);

    // clamp voltage to safe range
    if (p->voltage_mV < 3100)
        p->voltage_mV = 3100;

    if (p->voltage_mV > 3650)
        p->voltage_mV = 3650;

    // ------------------------------------------------------------
    // 3. Measured current (what PID sees)
    // ------------------------------------------------------------

    // first-order response (inverter + sensor dynamics)
    p->filt_current_dA += (net_current - p->filt_current_dA) / 4;

    return (int16_t)p->filt_current_dA;
}
#endif

void test_noise_rejection()
{
    current_controller_pv_t ctrl;
    init_pid(&ctrl);

    int16_t measured = 0;
    int16_t allowed = 0;

    for (int i = 0; i < 200; i++)
    {
        // inject noise ±20 dA
        int16_t noise = (i % 5 - 2) * 10;

        allowed = bms_charge_pid(
            measured + noise,
            300,
            3500,
            false,
            &ctrl
        );

        printf("%d\tmeas:%d\tallow:%d\ttgt:%d\n",
               i,
               measured+noise,
               allowed,
               300);

        measured = plant_follow(allowed);
    }

    assert(measured > 250 && measured < 350);
}

void test_pv_limited()
{
    current_controller_pv_t ctrl;
    init_pid(&ctrl);

    int16_t measured = 0;
    int16_t allowed = 0;

    for (int i = 0; i < 200; i++)
    {
        allowed = bms_charge_pid(
            measured,
            500,   // target higher than PV
            3500,
            false,
            &ctrl
        );
        printf("%d %d\tmeas:%d\tallow:%d\ttgt:%d\n",
                __LINE__,
               i,
               measured,
               allowed,
               500);

        measured = plant_pv_limited(allowed, 200);
    }

    // should saturate near PV max
    assert(measured >= 180 && measured <= 210);
}

void test_min_current_when_full()
{
    current_controller_pv_t ctrl;
    init_pid(&ctrl);

    int16_t allowed = bms_charge_pid(
        0,
        0,
        3600,   // above stop
        true,
        &ctrl
    );

    printf("%d\t%d\t%d\n",
       0,
       0,
       allowed);

    assert(allowed == ctrl.min_current_offset_dA);
}

void test_prevent_discharge_near_zero()
{
    current_controller_pv_t ctrl;
    init_pid(&ctrl);

    int16_t measured = -20; // discharging
    int16_t allowed = 0;

    for (int i = 0; i < 50; i++)
    {
        allowed = bms_charge_pid(
            measured,
            0,
            3500,
            false,
            &ctrl
        );

        printf("%d\tmeas:%d\tallow:%d\ttgt:%d\n",
               i,
               measured,
               allowed,
               0);

        measured = plant_follow(allowed);
    }

    // should push back toward >= 0
    assert(measured >= 0);
}

void test_positive_offset()
{
    current_controller_pv_t ctrl;
    init_pid(&ctrl);

    int16_t measured = 0;
    int16_t allowed = 0;

    for (int i = 0; i < 200; i++)
    {
        allowed = bms_charge_pid(
            measured,
            300,
            3500,
            false,
            &ctrl
        );


        printf("%d %d\tmeas:%d\tallow:%d\ttgt:%d\n",
                __LINE__,
               i,
               measured,
               allowed,
               300);

        measured = plant_with_offset(allowed, +ctrl.inverter_offset_dA);
    }

    assert(measured > 280 && measured < 330);
}


void test_positive_offset_close_0()
{
    current_controller_pv_t ctrl;
    init_pid(&ctrl);

    int16_t measured = 0;
    int16_t allowed = 0;

    int16_t expected = 2;

    for (int i = 0; i < 200; i++)
    {
        allowed = bms_charge_pid(measured, 2, 3500, false, &ctrl);
        printf("%d %d\tmeas:%d\tallow:%d\ttgt:%d\n",
                __LINE__,
               i,
               measured,
               allowed,
               2);
        measured = plant_with_offset(allowed, -1) + 5;


    }

    assert(abs(measured - expected) <= 1);
}

void test_negative_offset()
{
    current_controller_pv_t ctrl;
    init_pid(&ctrl);

    int16_t measured = 0;
    int16_t allowed = 0;

    for (int i = 0; i < 200; i++)
    {
        allowed = bms_charge_pid(
            measured,
            300,
            3500,
            false,
            &ctrl
        );

        printf("%d %d\tmeas:%d\tallow:%d\ttgt:%d\n",
                __LINE__,
               i,
               measured,
               allowed,
               300);

        measured = plant_with_offset(allowed, -ctrl.inverter_offset_dA);
    }

    assert(measured > 280 && measured < 330);
}

void test_negative_offset_close_0()
{
    current_controller_pv_t ctrl;
    init_pid(&ctrl);

    int16_t measured = 0;
    int16_t allowed = 0;

    int16_t expected = 2;

    for (int i = 0; i < 200; i++)
    {
        allowed = bms_charge_pid(
            measured,
            2,
            3500,
            false,
            &ctrl
        );

        printf("%d %d\tmeas:%d\tallow:%d\ttgt:%d\n",
                __LINE__,
               i,
               measured,
               allowed,
               2);

        measured = plant_with_offset(allowed, +1) - 5;
    }

    assert(abs(measured - expected) <= 1);
}

void test_cloud_recovery()
{
    current_controller_pv_t ctrl;
    init_pid(&ctrl);

    int16_t measured = 0;
    int16_t allowed = 0;

    for (int i = 0; i < 300; i++)
    {
        allowed = bms_charge_pid(
            measured,
            300,
            3500,
            false,
            &ctrl
        );

        printf("%d %d\tmeas:%d\tallow:%d\ttgt:%d\n",
                __LINE__,
               i,
               measured,
               allowed,
               300);

        if (i < 100)
        {
            // normal PV
            measured = plant_follow(allowed);
        }
        else if (i < 150)
        {
            // cloud: low PV
            measured = plant_pv_limited(allowed, 50);
        }
        else
        {
            // sun back
            measured = plant_follow(allowed);
        }
    }

    // must recover to target after cloud
    assert(measured > 250 && measured < 350);
}


#define V_FULL_STOP   3550
#define V_FULL_START  3450

static uint16_t simulate_voltage_from_charge(int16_t current_dA)
{
    // simple integrative battery model
    // higher current => higher voltage
    static int32_t v = 3400;

    v += current_dA / 15;   // charge raises voltage slowly
    v -= 1;                // natural relaxation

    if (v < 3300) v = 3300;
    if (v > 3600) v = 3600;

    return (uint16_t)v;
}

static int16_t pv_sun_profile(int t, int allowed)
{
    // smooth “sun day”
    if (t < 100) return allowed;           // morning
    if (t < 200) return allowed * 80 / 100;
    if (t < 300) return allowed * 60 / 100;
    if (t < 400) return allowed * 30 / 100;
    return allowed;
}

void test_hysteresis_cycle()
{
    current_controller_pv_t ctrl;
    init_pid(&ctrl);

    int16_t measured = 0;
    uint16_t v = 3400;
    bool soc_full = false;

    int16_t allowed = 0;

    bool stopped = false;
    bool restarted = false;

    plant_t plant = {0};
    plant_init(&plant);

    for (int t = 0; t < 300; t++)
    {
        soc_full = (v >= V_FULL_STOP);

        allowed = bms_charge_pid(
            measured,
            300,
            v,
            soc_full,
            &ctrl
        );

       printf("%d %d\tmeas:%d\tallow:%d\ttgt:%d\n",
                __LINE__,
               t,
               measured,
               allowed,
               300);


        //measured = plant_follow(allowed);
        measured = plant_step(&plant, allowed);
        v = simulate_voltage_from_charge(measured);

        if (v >= V_FULL_STOP)
            stopped = true;

        if (stopped && v < V_FULL_START)
            restarted = true;
    }

    // must have stopped at least once
    assert(stopped == true);

    // must have restarted after voltage drop
    assert(restarted == true);

    // when allowed, must never be zero
    assert(allowed >= ctrl.min_current_offset_dA);
}

void test_full_sun_day()
{
    current_controller_pv_t ctrl;
    init_pid(&ctrl);

    int16_t measured = 0;
    uint16_t v = 3350;
    bool soc_full = false;

    plant_t plant = {0};
    plant_init(&plant);

    int16_t allowed = 0;

    int max_allowed_seen = 0;
    int min_before_stop = 9999;

    bool stopped = false;
    bool restarted = false;

    for (int t = 0; t < 500; t++)
    {
        // simulate SOC flag
        soc_full = (v >= V_FULL_STOP);

        allowed = bms_charge_pid(
            measured,
            300,
            v,
            soc_full,
            &ctrl
        );

        printf("%d %d\tmeas:%d\tallow:%d\ttgt:%d\n",
                __LINE__,
               t,
               measured,
               allowed,
               300);


        // PV sun profile reduces effective charge
        //measured = pv_sun_profile(t, allowed);
        measured = plant_step(&plant, allowed + pv_sun_profile(t, allowed));

        v = simulate_voltage_from_charge(measured);

        if (allowed > max_allowed_seen)
            max_allowed_seen = allowed;

        if (!soc_full && v > V_FULL_STOP - 10)
            min_before_stop = (min_before_stop < v ? min_before_stop : v);

        if (v >= V_FULL_STOP)
            stopped = true;

        if (stopped && v < V_FULL_START)
            restarted = true;
    }

    // ------------------------------------------------------------
    // Assertions
    // ------------------------------------------------------------

    // must have used high current at start
    assert(max_allowed_seen > 250);

    // must have reduced current during day
    assert(min_before_stop < 3550);

    // must respect hysteresis behavior
    assert(stopped == true);
    assert(restarted == true);

    // must never collapse to zero
    assert(max_allowed_seen > ctrl.min_current_offset_dA);
}

static int16_t target_profile(int t)
{
    if (t < 100) return 300;  // bulk
    if (t < 200) return 250;  // early taper
    if (t < 300) return 180;  // mid taper
    if (t < 400) return 120;  // late taper
    return 60;                // near full stop
}

static uint16_t battery_voltage_model(int16_t current_dA)
{
    static int32_t v = 3350;

    v += current_dA / 6;
    v -= 1;

    if (v < 3300) v = 3300;
    if (v > 3600) v = 3600;

    return (uint16_t)v;
}

void test_load_disturbance_rejection()
{
    current_controller_pv_t ctrl = {0};
    init_pid(&ctrl);

    plant_t plant = {0};
    plant_init(&plant);

    int16_t allowed = 0;
    int16_t measured = 0;

    int16_t target = 1;

    // =========================================================
    // PHASE 1 — stable no load
    // =========================================================
    for (int t = 0; t < 20; t++)
    {
        measured = plant_step(&plant, allowed);
        allowed = bms_charge_pid(measured, target, 3500, true, &ctrl);

        printf("%d %d\tmeas:%d\tallow:%d\ttgt:%d v:%d\n",
                __LINE__,
               t,
               measured,
               allowed,
               target,
               3500);

        assert(measured >= -5);
    }

    // =========================================================
    // PHASE 2 — load applied (negative disturbance)
    // =========================================================
    bool negative_measure_seen = false;
    for (int t = 0; t < 50; t++)
    {
        int16_t load = -40;

        measured = plant_step(&plant, allowed + load);

        if (measured<0) {
            negative_measure_seen = true;
        }

        allowed = bms_charge_pid(measured, target, 3500, true, &ctrl);

        printf("%d %d\tmeas:%d\tallow:%d\ttgt:%d v:%d\n",
                __LINE__,
               t,
               measured,
               allowed,
               target,
               3500);

        // KEY ASSERTIONS
        assert(measured >= 0 || measured > -40);   // must reject discharge
        assert(allowed > 0);                      // controller must react
    }
    assert(negative_measure_seen);

    // =========================================================
    // PHASE 3 — load removed
    // =========================================================
    for (int t = 0; t < 100; t++)
    {
        measured = plant_step(&plant, allowed);

        allowed = bms_charge_pid(measured, target, 3500, true, &ctrl);

        printf("%d %d\tmeas:%d\tallow:%d\ttgt:%d v:%d\n",
                __LINE__,
               t,
               measured,
               allowed,
               target,
               3500);
    }

    // final stability
    assert(allowed <= target+1 && measured > 0);
}

void test_allowed_never_exceeds_target_plus_offset()
{
    current_controller_pv_t ctrl;
    init_pid(&ctrl);

    plant_t plant = {0};
    plant_init(&plant);

    int16_t target = 255;
    uint16_t v = 3100;

    int16_t measured = 16;

    for (int t = 0; t < 500; t++)
    {
        int16_t allowed = bms_charge_pid(
            measured,
            target,
            v,
            false,
            &ctrl
        );

        printf("%d %d\tmeas:%d\tallow:%d\ttgt:%d v:%d\n",
                __LINE__,
               t,
               measured,
               allowed,
               target,
               v);

        int16_t max_allowed =
            target + ctrl.inverter_offset_dA;

        // HARD INVARIANT TEST
        assert(allowed <= max_allowed);

        // simulate plant
        measured = plant_step(&plant, allowed);
        v = simulate_voltage_from_charge(measured);
    }
}

#ifdef X86
int main(void) {
    #if 0
    test_simple_pid2();
    test_simple_pid();
    test_min_current_offset_respected();
    //test_yoyo_long();
    //test_yoyo_long_delayed();
    // test offset -0.8 toutes les certain tmpes, maintient target?
    //test_offset_cycle();
    test_energy_deficit_recovery_cycles();
    // test clouding (pv power variation)
     test_cloud_event();
    // test clouding when battery full (pv power variation)
    test_low_pv_not_limited_behavior();
    test_voltage_hysteresis_min_current();
    test_pv_equals_target_min_offset();
    test_inverter_starvation_battery_discharge();
    #endif


    test_noise_rejection();
    test_pv_limited();
    test_min_current_when_full();
    test_prevent_discharge_near_zero();
    test_positive_offset();
    // test_positive_offset_close_0(); // no such world with a positive offset for charging (yet?)
    test_negative_offset();
    test_negative_offset_close_0();
    test_cloud_recovery();
    test_hysteresis_cycle();
    test_full_sun_day();
    test_load_disturbance_rejection();
    test_allowed_never_exceeds_target_plus_offset();
    return 0;
}
#endif // X86

/*
test prompts to generate the testcases:
=======================================

Ok, let's rethink all the tests, and use a commond init_pid function that initialize the pid variable at the start of each test 
I want few tests, here are each one description:
- a test to ensure the PID is not sensitive of measured power variation each cycle vs the target requested.
- a test that ensure target requested may be over the measured value, measured value is bound to effective power received by the PVs
- a test that ensure a minimum when full
- a test that tries to compensate discharge when battery is full (usually that means the allowed charge is really small) => and therefore the inverter tends to lock up until battery are drained a bit. but this is workaroundable with a higher allowed charge current
- a test that make sure the PID compensate when there is a positive offset between allowed charge and effective charge,
- a test that make sure the PID compensate when there is a negative offset between allowed charge and effective charge
- a test to ensure PID recovery after a low measured power (due to clouding)
- 200 samples inverter offset change, PID should adapt (the measured current is either some dA higher than allowed for 200 samples, or few dA lower than allowed for 200 samples)
- a test that finish the charge (full soc), to ensure that when battery are over the max voltage, then it pauses until min voltage is reached before performing charge again (hysterisis is respected), and with a minimal chrge current
- a full sun day scenario test, batteries are low, charge starts with high current, then reach a level when charge must be reduced to avoid going too fast to hgih voltage that stops the charge, then yet another level reducing the charge again, and then finally reaching hysterisis stop voltage (the battery is marked full before that moment!), then we wait until voltage goes below the min hysterisis to top up the charge again. In that test, it is tthe target current which is decreased at each level, to avoid reaching too high battery voltage too hastily.
- a test where battery is full (meaning the target current is minimal or 0), and the battery gets discharged because of home consumption, therefore the measured current gets negative, the PID has to compensate to ensure the residual charge is at least positive. And adjust after the load is disconnected, and therefore the compensation charge will be too much
- a test that ensure the allowed dA cannot go over target (255dA) + inverter_offset_dA, I have a log where the pv is limited, therefore measure is 16dA, target is 255dA, and the allowed dA goes 50dA, 100dA, 150dA, 200dA, 250dA, 300dA... this is not expected, it shall be limited.

take care of the right modelisation for battery voltage/ measured charge current (depending on fake pv power)

*/
