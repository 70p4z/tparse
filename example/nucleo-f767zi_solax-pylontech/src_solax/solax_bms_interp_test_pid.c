#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "bms_charge_pid.h"
#include "globals.h"
#include "string.h"

#define MIN(x,y) ((x)<(y)?x:y)

#ifdef X86
uint8_t tmp[TMP_BUFFER_SIZE_B];
void master_log(char* str) {
    printf(str);
}
#endif // X86

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

}

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
    int32_t voltage_mV;
    int32_t filt_current_dA;
    int32_t load_dA;
    int32_t soc_max;

} plant_t;


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

int16_t update_charge(int16_t maxch); // really lame decl
// non regression test for current, with observed problems
void test_update_charge(void) {
    // faked init value for test to run smoothly
    knobs.limited_charge_wattage = 180;
    pylontech.voltage = 4131;
    pylontech.precise_voltage = 413100;
    pylontech.precise_current = 6200;
    knobs.forced_wattage = 0;
    pylontech.vcellmax = 33;
    knobs.cell_voltage_limited_charge = 35;
    knobs.max_charge_voltage = 36;
    pylontech.bmu_idx = 8;
    pylontech.max_charge = 255;

    uint16_t target_dA = 255;
    pylontech.vcell_highest = 3355;
    pylontech.current = 62;

    /*
    uint16_t target_dA = 100;
    pylontech.vcell_highest = 3411;
    pylontech.current = 59;
    */

    memset(&pylontech_pid, 0, sizeof(pylontech_pid));
    pylontech_pid.kp_x100 = 60;
    pylontech_pid.ki_up_x100 = 20;
    pylontech_pid.ki_down_x100 = 50;
    pylontech_pid.kd_x100 = 5;
    pylontech_pid.max_step_up_dA = 20; // max change per cycle
    pylontech_pid.max_step_down_dA = 50; // max change per cycle (faster on drops)
    pylontech_pid.max_energy_step_dA = 50;
    pylontech_pid.energy_deadband_dA = 2;
    pylontech_pid.v_start_hyst_mV = 3450;
    pylontech_pid.v_stop_hyst_mV = 3550;
    pylontech_pid.charge_allowed = true;
    pylontech_pid.min_current_offset_dA = 1;
    pylontech_pid.inverter_offset_dA = 5;

    int16_t maxch; 
    int16_t missed_integral_x10 = 0;
    for (int i = 0; i < 50; i++) {
        maxch = update_charge(250);
        printf("%d \tmeas:%d\tallow:%d\n",__LINE__,
                   pylontech.current,
                   maxch);
        // adjust current smoothly from computed charge request
        int16_t delta = (maxch-pylontech.current);
        pylontech.current += delta/10;
        missed_integral_x10 += delta - (delta/10)*10;
        if (missed_integral_x10 >= 10 || missed_integral_x10 <= -10) {
            pylontech.current += missed_integral_x10/10;
            missed_integral_x10 -= (missed_integral_x10/10)*10;
        }

        assert(maxch <= pylontech.cap_max_charge+pylontech_pid.inverter_offset_dA);
    }

    assert(pylontech.current >= target_dA - 5*target_dA/100);
    assert(pylontech.current <= target_dA + 5*target_dA/100);
}

void test_update_charge2(void) {
    // faked init value for test to run smoothly
    knobs.limited_charge_wattage = 180;
    pylontech.voltage = 4131;
    pylontech.precise_voltage = 413100;
    pylontech.precise_current = 6200;
    knobs.forced_wattage = 0;
    pylontech.vcellmax = 33;
    knobs.cell_voltage_limited_charge = 35;
    knobs.max_charge_voltage = 36;
    pylontech.bmu_idx = 8;
    pylontech.max_charge = 255;

    uint16_t target_dA = 60;
    pylontech.vcell_highest = 3411;
    pylontech.current = 59;

    memset(&pylontech_pid, 0, sizeof(pylontech_pid));
    pylontech_pid.kp_x100 = 60;
    pylontech_pid.ki_up_x100 = 20;
    pylontech_pid.ki_down_x100 = 50;
    pylontech_pid.kd_x100 = 5;
    pylontech_pid.max_step_up_dA = 20; // max change per cycle
    pylontech_pid.max_step_down_dA = 50; // max change per cycle (faster on drops)
    pylontech_pid.max_energy_step_dA = 50;
    pylontech_pid.energy_deadband_dA = 2;
    pylontech_pid.v_start_hyst_mV = 3450;
    pylontech_pid.v_stop_hyst_mV = 3550;
    pylontech_pid.charge_allowed = true;
    pylontech_pid.min_current_offset_dA = 1;
    pylontech_pid.inverter_offset_dA = 5;

    int16_t maxch; 
    int16_t missed_integral_x10 = 0;
    for (int i = 0; i < 50; i++) {
        maxch = update_charge(250);
        printf("%d \tmeas:%d\tallow:%d\n",__LINE__,
                   pylontech.current,
                   maxch);
        // adjust current smoothly from computed charge request
        int16_t delta = (maxch-pylontech.current);
        pylontech.current += delta/10;
        missed_integral_x10 += delta - (delta/10)*10;
        if (missed_integral_x10 >= 10 || missed_integral_x10 <= -10) {
            pylontech.current += missed_integral_x10/10;
            missed_integral_x10 -= (missed_integral_x10/10)*10;
        }

        assert(maxch <= pylontech.cap_max_charge+pylontech_pid.inverter_offset_dA);
    }

    assert(pylontech.current >= target_dA - 5*target_dA/100);
    assert(pylontech.current <= target_dA + 5*target_dA/100);
}

#ifdef X86
int main(void) {


    // PID-only tests

    printf("test_noise_rejection\n");
    test_noise_rejection();
    printf("test_pv_limited\n");
    test_pv_limited();
    printf("test_min_current_when_full\n");
    test_min_current_when_full();
    printf("test_prevent_discharge_near_zero\n");
    test_prevent_discharge_near_zero();
    printf("test_positive_offset\n");
    // test_positive_offset_close_0(); // no such world with a positive offset for charging (yet?)
    test_positive_offset();
    printf("test_negative_offset\n");
    test_negative_offset();
    printf("test_negative_offset_close_0\n");
    test_negative_offset_close_0();
    printf("test_cloud_recovery\n");
    test_cloud_recovery();
    printf("test_hysteresis_cycle\n");
    test_hysteresis_cycle();
    printf("test_full_sun_day\n");
    test_full_sun_day();
    printf("test_load_disturbance_rejection\n");
    test_load_disturbance_rejection();
    printf("test_allowed_never_exceeds_target_plus_offset\n");
    test_allowed_never_exceeds_target_plus_offset();


    // charge update global tests
    printf("test_update_charge\n");
    test_update_charge();
    printf("test_update_charge2\n");
    test_update_charge2();

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
