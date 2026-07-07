#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "bms_charge_pid.h"
#include "globals.h"
#include "string.h"

#ifdef X86
uint32_t uwTick;
uint8_t tmp[TMP_BUFFER_SIZE_B];
void master_log(char* str) {
    printf(str);
}

#include "stdlib.h"
#include "stdio.h"

char *bin2hex(void *_p, int len)
{
    char* p = (char*)_p;
    char *hex = malloc(((2*len) + 1));
    char *r = hex;

    while(len && p)
    {
        (*r) = ((*p) & 0xF0) >> 4;
        (*r) = ((*r) <= 9 ? '0' + (*r) : 'A' - 10 + (*r));
        r++;
        (*r) = ((*p) & 0x0F);
        (*r) = ((*r) <= 9 ? '0' + (*r) : 'A' - 10 + (*r));
        r++;
        p++;
        len--;
    }
    *r = '\0';

    return hex;
}

unsigned char *hex2bin(const char *str, int* length)
{
    int len, h;
    unsigned char *result, *err, *p, c;

    // default error is an empty freeable string
    err = malloc(1);
    *err = 0;
    // init length
    if (length) {
        *length = 0;
    }

    if (!str) {
        *err = '0';
        return err;
    }

    if (!*str) {
        *err = '1';
        return err;
    }

    len = 0;
    p = (unsigned char*) str;
    while (*p) {
        // skip blanks
        if (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') {
            p++;
            continue;
        }
        p++;
        len++;
    }

    result = malloc((len/2)+1);
    // accept to start at a half byte
    h = !(len%2) * 4;
    p = result;
    *p = 0;

    c = *str;
    while(c)
    {
        if(('0' <= c) && (c <= '9'))
            *p += (c - '0') << h;
        else if(('A' <= c) && (c <= 'F'))
            *p += (c - 'A' + 10) << h;
        else if(('a' <= c) && (c <= 'f'))
            *p += (c - 'a' + 10) << h;
        // skip blanks
        else if (c == ' ' || c == '\t' || c == '\r' || c == '\n') {
            str++;
            c = *str;
            continue;
        }
        else {
            *err = c;
            free(result);
            printf("invalid char in hex: %c", c);
            return err;
        }

        str++;
        c = *str;

        // char to nibble
        if (h)
            h = 0;
        else
        {
            h = 4;
            p++;
            *p = 0;
        }
    }
    if (length) {
        *length = len/2;
    }
    if (err) {
        free(err);
    }
    return result;
}

////////////////////////////////////////////////////////////////////////////////*/
///                                                                             */
///        ▄▄▄▄   ▄▄    ▄▄     ▄▄▄▄             ▄▄▄▄▄▄     ▄▄▄▄▄▄   ▄▄▄▄▄       */
///      ██▀▀▀▀█  ██    ██   ██▀▀▀▀█            ██▀▀▀▀█▄   ▀▀██▀▀   ██▀▀▀██     */
///     ██▀       ██    ██  ██                  ██    ██     ██     ██    ██    */
///     ██        ████████  ██  ▄▄▄▄            ██████▀      ██     ██    ██    */
///     ██▄       ██    ██  ██  ▀▀██            ██           ██     ██    ██    */
///      ██▄▄▄▄█  ██    ██   ██▄▄▄██            ██         ▄▄██▄▄   ██▄▄▄██     */
///        ▀▀▀▀   ▀▀    ▀▀     ▀▀▀▀             ▀▀         ▀▀▀▀▀▀   ▀▀▀▀▀       */
///                                                                             */
///                                                                             */
////////////////////////////////////////////////////////////////////////////////*/

void init_pid(current_controller_pv_t *ctrl)
{
    memset(ctrl, 0, sizeof(*ctrl));

    ctrl->kp_x100 = 59;
    ctrl->ki_up_x100 = 10;
    ctrl->ki_down_x100 = 20;
    ctrl->kd_x100 = 5;

    ctrl->max_step_up_dA = 5;
    ctrl->max_step_down_dA = 5;

    ctrl->max_energy_step_dA = 50;
    ctrl->energy_deadband_dA = 2;

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
        allowed = bms_charge_pid(measured, 2, &ctrl);
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
    int16_t last_measured = 0;

    for (int i = 0; i < 200; i++)
    {
        last_measured = measured;
        allowed = bms_charge_pid(
            measured,
            expected,
            &ctrl
        );

        printf("%d %d\tmeas:%d\tallow:%d\ttgt:%d\n",
                __LINE__,
               i,
               measured,
               allowed,
               expected);

        measured = plant_with_offset(allowed, +1) - 5;

    }

    assert(abs(last_measured- expected) <= 1);
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

    v += current_dA / 50;   // charge raises voltage slowly
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

/*
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
*/

/*
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
*/

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
        allowed = bms_charge_pid(measured, target, &ctrl);

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
        int16_t load = -32;

        measured = plant_step(&plant, allowed + load);

        if (measured<0) {
            negative_measure_seen = true;
        }

        allowed = bms_charge_pid(measured, target, &ctrl);

        printf("%d %d\tmeas:%d\tallow:%d\ttgt:%d v:%d\n",
                __LINE__,
               t,
               measured,
               allowed,
               target,
               3500);

        // KEY ASSERTIONS
        assert(measured >= 0 || measured >= -30);   // must reject discharge
        assert(allowed > 2);                      // controller must react
    }
    assert(negative_measure_seen);

    // =========================================================
    // PHASE 3 — load removed
    // =========================================================
    for (int t = 0; t < 100; t++)
    {
        measured = plant_step(&plant, allowed);

        allowed = bms_charge_pid(measured, target, &ctrl);

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

    int16_t measured = 16;

    for (int t = 0; t < 500; t++)
    {
        int16_t allowed = bms_charge_pid(
            measured,
            target,
            &ctrl
        );

        printf("%d %d\tmeas:%d\tallow:%d\ttgt:%d\n",
                __LINE__,
               t,
               measured,
               allowed,
               target);

        int16_t max_allowed =
            target + ctrl.inverter_offset_dA;

        // HARD INVARIANT TEST
        assert(allowed <= max_allowed);

        // simulate plant
        measured = plant_step(&plant, allowed);
        //v = simulate_voltage_from_charge(measured);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////*/
///                                                                                                 */
///     ▄▄▄▄▄▄     ▄▄▄▄▄▄   ▄▄▄▄▄                  ██                                               */
///     ██▀▀▀▀█▄   ▀▀██▀▀   ██▀▀▀██                ▀▀                 ██                            */
///     ██    ██     ██     ██    ██             ████     ██▄████▄  ███████    ▄████▄    ▄███▄██    */
///     ██████▀      ██     ██    ██               ██     ██▀   ██    ██      ██▄▄▄▄██  ██▀  ▀██    */
///     ██           ██     ██    ██               ██     ██    ██    ██      ██▀▀▀▀▀▀  ██    ██    */
///     ██         ▄▄██▄▄   ██▄▄▄██             ▄▄▄██▄▄▄  ██    ██    ██▄▄▄   ▀██▄▄▄▄█  ▀██▄▄███    */
///     ▀▀         ▀▀▀▀▀▀   ▀▀▀▀▀               ▀▀▀▀▀▀▀▀  ▀▀    ▀▀     ▀▀▀▀     ▀▀▀▀▀    ▄▀▀▀ ██    */
///                                                                                      ▀████▀▀    */
///                                                                                                 */
////////////////////////////////////////////////////////////////////////////////////////////////////*/

int16_t update_charge(int16_t maxch); // really lame decl
// non regression test for current, with observed problems
void test_update_charge(void) {
    // faked init value for test to run smoothly
    knobs.limited_charge_wattage = 180;
    pylontech.voltage_dV = 4131;
    pylontech.precise_voltage_mV = 413100;
    pylontech.precise_current_mA = 6200;
    knobs.forced_wattage = 0;
    pylontech.vcellmax = 33;
    knobs.cell_voltage_limited_charge = 35;
    knobs.max_charge_voltage = 36;
    pylontech.bmu_idx = 8;
    pylontech.max_charge_dA = 255;

    uint16_t target_dA = 255;
    pylontech.vcell_highest = 3355;
    pylontech.current_dA = 62;

    /*
    uint16_t target_dA = 100;
    pylontech.vcell_highest = 3411;
    pylontech.current_dA = 59;
    */

    pylontech.tcellmax = 200;
    knobs.max_charge_temperature = 400;

    memset(&pylontech_pid, 0, sizeof(pylontech_pid));
    pylontech_pid.kp_x100 = 60;
    pylontech_pid.ki_up_x100 = 20;
    pylontech_pid.ki_down_x100 = 50;
    pylontech_pid.kd_x100 = 5;
    pylontech_pid.max_step_up_dA = 20; // max change per cycle
    pylontech_pid.max_step_down_dA = 50; // max change per cycle (faster on drops)
    pylontech_pid.max_energy_step_dA = 50;
    pylontech_pid.energy_deadband_dA = 2;
    pylontech_pid.min_current_offset_dA = 1;
    pylontech_pid.inverter_offset_dA = 5;

    int16_t maxch; 
    int16_t missed_integral_x10 = 0;
    for (int i = 0; i < 50; i++) {
        maxch = update_charge(250);
        printf("%d \tmeas:%d\tallow:%d\n",__LINE__,
                   pylontech.current_dA,
                   maxch);
        // adjust current smoothly from computed charge request
        int16_t delta = (maxch-pylontech.current_dA);
        pylontech.current_dA += delta/10;
        missed_integral_x10 += delta - (delta/10)*10;
        if (missed_integral_x10 >= 10 || missed_integral_x10 <= -10) {
            pylontech.current_dA += missed_integral_x10/10;
            missed_integral_x10 -= (missed_integral_x10/10)*10;
        }

        assert(maxch <= pylontech.cap_max_charge_dA+pylontech_pid.inverter_offset_dA);
    }

    assert(pylontech.current_dA >= target_dA - 5*target_dA/100);
    assert(pylontech.current_dA <= target_dA + 5*target_dA/100);
}

void test_update_charge2(void) {
    // faked init value for test to run smoothly
    knobs.limited_charge_wattage = 180;
    pylontech.voltage_dV = 4131;
    pylontech.precise_voltage_mV = 413100;
    pylontech.precise_current_mA = 6200;
    knobs.forced_wattage = 0;
    pylontech.vcellmax = 33;
    knobs.cell_voltage_limited_charge = 35;
    knobs.max_charge_voltage = 36;
    pylontech.bmu_idx = 8;
    pylontech.max_charge_dA = 255;

    uint16_t target_dA = 90;
    pylontech.vcell_highest = 3411;
    pylontech.current_dA = 59;

    memset(&pylontech_pid, 0, sizeof(pylontech_pid));
    pylontech_pid.kp_x100 = 60;
    pylontech_pid.ki_up_x100 = 20;
    pylontech_pid.ki_down_x100 = 50;
    pylontech_pid.kd_x100 = 5;
    pylontech_pid.max_step_up_dA = 20; // max change per cycle
    pylontech_pid.max_step_down_dA = 50; // max change per cycle (faster on drops)
    pylontech_pid.max_energy_step_dA = 50;
    pylontech_pid.energy_deadband_dA = 2;
    pylontech_pid.min_current_offset_dA = 1;
    pylontech_pid.inverter_offset_dA = 5;

    int16_t maxch; 
    int16_t missed_integral_x10 = 0;
    for (int i = 0; i < 100; i++) {
        maxch = update_charge(250);
        printf("%d \tmeas:%d\tallow:%d\n",__LINE__,
                   pylontech.current_dA,
                   maxch);

        // ensure the test is correct (batt voltage vs expected target)
        assert (pylontech.cap_max_charge_dA == target_dA);

        // adjust current smoothly from computed charge request
        int16_t delta = (maxch-pylontech.current_dA);
        pylontech.current_dA += delta/10;
        missed_integral_x10 += delta - (delta/10)*10;
        if (missed_integral_x10 >= 10 || missed_integral_x10 <= -10) {
            pylontech.current_dA += missed_integral_x10/10;
            missed_integral_x10 -= (missed_integral_x10/10)*10;
        }

        assert(maxch <= pylontech.cap_max_charge_dA+pylontech_pid.inverter_offset_dA);
    }

    assert(pylontech.current_dA >= target_dA - 5*target_dA/100);
    assert(pylontech.current_dA <= target_dA + 5*target_dA/100);
}

void test_maintain_top_up(void) {
    // faked init value for test to run smoothly
    pylontech.voltage_dV = 4131;
    pylontech.precise_voltage_mV = 413100;
    pylontech.precise_current_mA = 6200;
    pylontech.precise_wattage = 0;
    knobs.forced_wattage = 0;
    knobs.cell_voltage_limited_charge = 35;
    knobs.limited_charge_wattage = 180;
    knobs.max_charge_voltage = 36;
    pylontech.bmu_idx = 8;
    pylontech.max_charge_dA = 255;

    uint16_t target_dA = 1;
    pylontech.current_dA = 0;

    pylontech.vcell_highest = 3400;
    pylontech.vcellmax = pylontech.vcell_highest/100;
    pylontech.tcellmax = 200;
    knobs.max_charge_temperature = 400;

    memset(&pylontech_pid, 0, sizeof(pylontech_pid));
    pylontech_pid.kp_x100 = 60;
    pylontech_pid.ki_up_x100 = 20;
    pylontech_pid.ki_down_x100 = 50;
    pylontech_pid.kd_x100 = 5;
    pylontech_pid.max_step_up_dA = 20; // max change per cycle
    pylontech_pid.max_step_down_dA = 50; // max change per cycle (faster on drops)
    pylontech_pid.max_energy_step_dA = 50;
    pylontech_pid.energy_deadband_dA = 2;
    pylontech_pid.min_current_offset_dA = 1;
    pylontech_pid.inverter_offset_dA = 5;


    int16_t maxch; 
    int32_t missed_integral_x10 = 0;

    for (int t = 0; t < 1000; t++)
    {
        //pylontech.vcell_highest = simulate_voltage_from_charge(pylontech.current);
        maxch = update_charge(0 /*BMS says end of charge*/);
        printf("%d %d \tmeas:%d\tallow:%d v:%d\n",__LINE__, t,
                   pylontech.current_dA,
                   maxch,
                   pylontech.vcell_highest);

        // adjust current smoothly from computed charge request
        int16_t delta = (maxch-pylontech.current_dA);
        pylontech.current_dA += delta/10;
        missed_integral_x10 += delta - (delta/10)*10;
        if (missed_integral_x10 >= 10 || missed_integral_x10 <= -10) {
            pylontech.current_dA += missed_integral_x10/10;
            missed_integral_x10 -= (missed_integral_x10/10)*10;
        }


        pylontech.vcell_highest += pylontech.current_dA*10/20;
        pylontech.vcell_highest--; // natural relaxation
        pylontech.vcellmax = pylontech.vcell_highest/100;
    }

    assert(pylontech.current_dA >= target_dA - 5 - 5*target_dA/100);
    assert(pylontech.current_dA <= target_dA + 5 + 5*target_dA/100);
}

void test_pid_delayed_feedback_integral(void) {
    // ------------------------------
    // Test parameters
    // ------------------------------
    const int M = 11;          // Max delay to test
    const int target_dA = 90;  // target current
    #define CYCLES 1000    // simulation cycles per delay
    const int allowed_to_measure_decimation = 30;

    // faked init value for test to run smoothly
    pylontech.voltage_dV = 4131;
    pylontech.precise_voltage_mV = 413100;
    pylontech.precise_current_mA = 6200;
    pylontech.precise_wattage = 0;
    knobs.forced_wattage = 0;
    knobs.cell_voltage_limited_charge = 35;
    knobs.limited_charge_wattage = 180;
    knobs.max_charge_voltage = 36;
    pylontech.bmu_idx = 8;
    pylontech.max_charge_dA = 255;

    pylontech.current_dA = 0;

    pylontech.vcell_highest = 3400;
    pylontech.vcellmax = pylontech.vcell_highest/100;
    pylontech.tcellmax = 200;
    knobs.max_charge_temperature = 400;

    // ------------------------------
    // Loop over different delay values
    // ------------------------------
    for (int delay_cycles = 1; delay_cycles < M; delay_cycles++) {
        printf("Testing delay_cycles = %d\n", delay_cycles);

        int16_t allowed_history[CYCLES] = {0}; // store past allowed_dA
        int32_t missed_integral_x10 = 0;

        // Reset integrators between delay runs
        pylontech.current_dA = 0;

        // ------------------------------
        // Initialize BMS PID struct
        // ------------------------------
        memset(&pylontech_pid, 0, sizeof(pylontech_pid));
        pylontech_pid.kp_x100 = 30;
        pylontech_pid.ki_up_x100 = 2;
        pylontech_pid.ki_down_x100 = 2;
        pylontech_pid.kd_x100 = 5;
        pylontech_pid.max_step_up_dA = 10;
        pylontech_pid.max_step_down_dA = 20;
        pylontech_pid.max_energy_step_dA = 20;
        pylontech_pid.energy_deadband_dA = 2;
        pylontech_pid.min_current_offset_dA = 1;
        pylontech_pid.inverter_offset_dA = 5;

        for (int t = 0; t < CYCLES; t++) {
            // ------------------------------
            // PID computation
            // ------------------------------
            /*
            int16_t allowed_dA = bms_charge_pid(
                pylontech.current,
                target_dA,
                &pylontech_pid
            );
            */
            int16_t allowed_dA = update_charge(target_dA);
            printf("%d %d delay:%d \tmeas:%d\tallow:%d v:%d\n",__LINE__, t,
                       delay_cycles,
                       pylontech.current_dA,
                       allowed_dA,
                       pylontech.vcell_highest);

            // ensure the test is correct (batt voltage vs expected target)
            assert (pylontech.cap_max_charge_dA == target_dA);


            assert(allowed_dA <= target_dA + 10);
            assert(allowed_dA >= 0);
            assert(allowed_dA >= target_dA / 2 || t <= target_dA / pylontech_pid.max_step_up_dA);


            // Store allowed for history
            allowed_history[t] = allowed_dA;

            // ------------------------------
            // Compute next pylontech.current using delta + integral
            // ------------------------------
            int hist_idx = (t >= delay_cycles) ? t - delay_cycles : 0;
            int16_t delta = allowed_history[hist_idx] - pylontech.current_dA;
            pylontech.current_dA += delta / allowed_to_measure_decimation;  // smooth fraction
            missed_integral_x10 += delta - (delta / allowed_to_measure_decimation) * allowed_to_measure_decimation;

            if (missed_integral_x10 >= 10 || missed_integral_x10 <= -10) {
                pylontech.current_dA += missed_integral_x10 / 10;
                missed_integral_x10 -= (missed_integral_x10 / 10) * 10;
            }

            // ------------------------------
            // Sanity checks
            // ------------------------------
            assert(allowed_dA >= pylontech_pid.min_current_offset_dA);
            assert(allowed_dA <= target_dA + pylontech_pid.inverter_offset_dA + 100);
        }
    }

    printf("✅ PID stability under delayed feedback (integral method) test passed for all delays 1..%d\n", M-1);
}

//////////////////////////////////////////////////////////////////////////////////////////*/
///                                                                                       */
///        ▄▄▄▄   ▄▄    ▄▄     ▄▄▄▄   ▄▄▄▄▄▄              ▄▄▄▄▄▄     ▄▄▄▄▄▄   ▄▄▄▄▄       */
///      ██▀▀▀▀█  ██    ██   ██▀▀▀▀█  ██▀▀▀▀██            ██▀▀▀▀█▄   ▀▀██▀▀   ██▀▀▀██     */
///     ██▀       ██    ██  ██        ██    ██            ██    ██     ██     ██    ██    */
///     ██        ████████  ██  ▄▄▄▄  ███████             ██████▀      ██     ██    ██    */
///     ██▄       ██    ██  ██  ▀▀██  ██  ▀██▄            ██           ██     ██    ██    */
///      ██▄▄▄▄█  ██    ██   ██▄▄▄██  ██    ██            ██         ▄▄██▄▄   ██▄▄▄██     */
///        ▀▀▀▀   ▀▀    ▀▀     ▀▀▀▀   ▀▀    ▀▀▀           ▀▀         ▀▀▀▀▀▀   ▀▀▀▀▀       */
///                                                                                       */
///                                                                                       */
//////////////////////////////////////////////////////////////////////////////////////////*/

void init_pid_grid(current_controller_pv_t *ctrl)
{
    memset(ctrl, 0, sizeof(*ctrl));

    ctrl->kp_x100 = 100;
    ctrl->ki_up_x100 = 10;
    ctrl->ki_down_x100 = 20;
    ctrl->kd_x100 = 10;

    ctrl->max_step_up_dA = 5;
    ctrl->max_step_down_dA = 5;

    ctrl->max_energy_step_dA = 50;
    ctrl->energy_deadband_dA = 2;

    ctrl->min_current_offset_dA = 0;

    ctrl->inverter_offset_dA = 0; // observed max offset internally consumed by the inverter (more surely expressed as something related to power of the link)

    // allow to respect the target as a measured value, not as a returned max allowed value
    ctrl->compensate_measure = 1;
}

void test_pid_for_charger_1() {
    current_controller_pv_t ctrl;
    init_pid_grid(&ctrl);

    int16_t measured = 0;
    int16_t allowed = 0;

    int bat_voltage = 400;
    int target_charge_W = 1000;
    #define W_to_dA(w) (((w)*10)/bat_voltage)
    int target_charge_dA = W_to_dA(target_charge_W);

    static const int measured_values_without_charge_W[] = {
        // simulate a switching load, such as an induction hob
        -300,
        -250,
        -850,
        -800,
        -850,
        -350,
        -900,
        -300,
        -260,
        -900,
        -840,
        -350,
        -300,
        -900,
        -850,
        -300,
        -250,
        -850,
        -800,
        -850,
        -350,
        -900,
        -300,
        -260,
        -900,
        -840,
        -350,
        -300,
        -900,
        -850,
        -300,
        -250,
        -850,
        -800,
        -850,
        -350,
        -900,
        -300,
        -260,
        -900,
        -840,
        -350,
        -300,
        -900,
        -850,
        // now add the heat pump in winter mode
        -300 -700,
        -250 -700,
        -850 -700,
        -800 -700,
        -850 -700,
        -350 -700,
        -900 -700,
        -300 -700,
        -260 -700,
        -900 -700,
        -840 -700,
        -350 -700,
        -300 -700,
        -900 -700,
        -850 -700,
        -300 -700,
        -250 -700,
        -850 -700,
        -800 -700,
        -850 -700,
        -350 -700,
        -900 -700,
        -300 -700,
        -260 -700,
        -900 -700,
        -840 -700,
        -350 -700,
        -300 -700,
        -900 -700,
        -850 -700,
        -300 -700,
        -250 -700,
        -850 -700,
        -800 -700,
        -850 -700,
        -350 -700,
        -900 -700,
        -300 -700,
        -260 -700,
        -900 -700,
        -840 -700,
        -350 -700,
        -300 -700,
        -900 -700,
        -850 -700,

        // get back to a switching load
        -300,
        -250,
        -850,
        -800,
        -850,
        -350,
        -900,
        -300,
        -260,
        -900,
        -840,
        -350,
        -300,
        -900,
        -850,
        -300,
        -250,
        -850,
        -800,
        -850,
        -350,
        -900,
        -300,
        -260,
        -900,
        -840,
        -350,
        -300,
        -900,
        -850,
        -300,
        -250,
        -850,
        -800,
        -850,
        -350,
        -900,
        -300,
        -260,
        -900,
        -840,
        -350,
        -300,
        -900,
        -850,

        // simulate a switching load, such as an induction hob
        -300,
        -250,
        -850,
        -800,
        -850,
        -350,
        -900,
        -300,
        -260,
        -900,
        -840,
        -350,
        -300,
        -900,
        -850,
        -300,
        -250,
        -850,
        -800,
        -850,
        -350,
        -900,
        -300,
        -260,
        -900,
        -840,
        -350,
        -300,
        -900,
        -850,
        -300,
        -250,
        -850,
        -800,
        -850,
        -350,
        -900,
        -300,
        -260,
        -900,
        -840,
        -350,
        -300,
        -900,
        -850,
        // now add the heat pump in winter mode
        -300 -700,
        -250 -700,
        -850 -700,
        -800 -700,
        -850 -700,
        -350 -700,
        -900 -700,
        -300 -700,
        -260 -700,
        -900 -700,
        -840 -700,
        -350 -700,
        -300 -700,
        -900 -700,
        -850 -700,
        -300 -700,
        -250 -700,
        -850 -700,
        -800 -700,
        -850 -700,
        -350 -700,
        -900 -700,
        -300 -700,
        -260 -700,
        -900 -700,
        -840 -700,
        -350 -700,
        -300 -700,
        -900 -700,
        -850 -700,
        -300 -700,
        -250 -700,
        -850 -700,
        -800 -700,
        -850 -700,
        -350 -700,
        -900 -700,
        -300 -700,
        -260 -700,
        -900 -700,
        -840 -700,
        -350 -700,
        -300 -700,
        -900 -700,
        -850 -700,

        // get back to a switching load
        -300,
        -250,
        -850,
        -800,
        -850,
        -350,
        -900,
        -300,
        -260,
        -900,
        -840,
        -350,
        -300,
        -900,
        -850,
        -300,
        -250,
        -850,
        -800,
        -850,
        -350,
        -900,
        -300,
        -260,
        -900,
        -840,
        -350,
        -300,
        -900,
        -850,
        -300,
        -250,
        -850,
        -800,
        -850,
        -350,
        -900,
        -300,
        -260,
        -900,
        -840,
        -350,
        -300,
        -900,
        -850,
    };

    // the test ensures the PID doesn't take long to ensure an average charge of target_charge_dA (that compensate the load + the charge request), and when the charge request it too high, then too bad, the battery gets drained
    int avg=0;
    int avg_load=0;
    int count = sizeof(measured_values_without_charge_W)/sizeof(measured_values_without_charge_W[0]);
    for (int i=0; i < count; i++)
    {
        // compute the total load +charge combined in the battery as a resulting current
        int measured_dA = W_to_dA(measured_values_without_charge_W[i]) + allowed /* one back log*/;

        // average the effective load current (discharge)
        avg_load += W_to_dA(measured_values_without_charge_W[i]);

        // the target is the CHARGING target, therefore the actual discharge has to be compensated
        int tgt=target_charge_dA/* - W_to_dA(measured_values_without_charge_W[i])*/;

        allowed = bms_charge_pid(
            measured_dA,
            tgt,
            &ctrl
        );

        // sum the delta of allowed current for charge (try to compensate the load AND charge at the given value)
        avg+=allowed;

        printf("%d \tmeas:%d\tallow:%d (avg:%d)\ttgt:%d (purechg:%d)\n",
                __LINE__,
               measured_dA,
               allowed,
               avg/(i+1),
               tgt, target_charge_dA);
    }

    int load_avg = avg_load/count;
    int grid_avg = avg/count;
    int bat_avg = avg/count + avg_load/count;
    printf("LOAD avg load %d\n",load_avg);
    printf("GRID avg %d\n",grid_avg);
    printf(">BAT avg charge %d\n",bat_avg);

    // ensure the battery is charging as expected
    assert(bat_avg > 2*target_charge_dA/3);
    assert(bat_avg < 4*target_charge_dA/3);

}

void test_pid_for_charger_2() {
    current_controller_pv_t ctrl;
    init_pid_grid(&ctrl);

    int16_t measured = 0;
    int16_t allowed = 0;

    int bat_voltage = 400;
    int target_charge_W = 1000;
    #define W_to_dA(w) (((w)*10)/bat_voltage)
    int target_charge_dA = W_to_dA(target_charge_W);

    static const int measured_values_without_charge_W[] = {
        // simulate a switching load, such as an induction hob
        -300,
        -275,
        -280,
        -302,
        -296,
        -273,
        -298,
        -300,
        -275,
        -280,
        -302,
        -296,
        -273,
        -298,
        -300,
        -275,
        -280,
        -302,
        -296,
        -273,
        -298,
        -300,
        -275,
        -280,
        -302,
        -296,
        -273,
        -298,
        -300,
        -275,
        -280,
        -302,
        -296,
        -273,
        -298,
        -300,
        -275,
        -280,
        -302,
        -296,
        -273,
        -298,
        -300,
        -275,
        -280,
        -302,
        -296,
        -273,
        -298,
    };

    // the test ensures the PID doesn't take long to ensure an average charge of target_charge_dA (that compensate the load + the charge request), and when the charge request it too high, then too bad, the battery gets drained
    int avg=0;
    int avg_load=0;
    int count = sizeof(measured_values_without_charge_W)/sizeof(measured_values_without_charge_W[0]);
    for (int i=0; i < count; i++)
    {
        // compute the total load +charge combined in the battery as a resulting current
        int measured_dA = W_to_dA(measured_values_without_charge_W[i]) + allowed /* one back log*/;

        // average the effective load current (discharge)
        avg_load += W_to_dA(measured_values_without_charge_W[i]);

        // the target is the CHARGING target, therefore the actual discharge has to be compensated
        int tgt=target_charge_dA/* - W_to_dA(measured_values_without_charge_W[i])*/;

        allowed = bms_charge_pid(
            measured_dA,
            tgt,
            &ctrl
        );

        // sum the delta of allowed current for charge (try to compensate the load AND charge at the given value)
        avg+=allowed;

        printf("%d \tmeas:%d\tallow:%d (avg:%d)\ttgt:%d (purechg:%d)\n",
                __LINE__,
               measured_dA,
               allowed,
               avg/(i+1),
               tgt, target_charge_dA);
    }

    int load_avg = avg_load/count;
    int grid_avg = avg/count;
    int bat_avg = avg/count + avg_load/count;
    printf("LOAD avg load %d\n",load_avg);
    printf("GRID avg %d\n",grid_avg);
    printf(">BAT avg charge %d\n",bat_avg);

    // ensure the battery is charging as expected
    assert(bat_avg > 2*target_charge_dA/3);
    assert(bat_avg < 4*target_charge_dA/3);

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
///                                                                                                           */
///        ▄▄▄▄   ▄▄    ▄▄     ▄▄▄▄   ▄▄▄▄▄▄                 ██                                               */
///      ██▀▀▀▀█  ██    ██   ██▀▀▀▀█  ██▀▀▀▀██               ▀▀                 ██                            */
///     ██▀       ██    ██  ██        ██    ██             ████     ██▄████▄  ███████    ▄████▄    ▄███▄██    */
///     ██        ████████  ██  ▄▄▄▄  ███████                ██     ██▀   ██    ██      ██▄▄▄▄██  ██▀  ▀██    */
///     ██▄       ██    ██  ██  ▀▀██  ██  ▀██▄               ██     ██    ██    ██      ██▀▀▀▀▀▀  ██    ██    */
///      ██▄▄▄▄█  ██    ██   ██▄▄▄██  ██    ██            ▄▄▄██▄▄▄  ██    ██    ██▄▄▄   ▀██▄▄▄▄█  ▀██▄▄███    */
///        ▀▀▀▀   ▀▀    ▀▀     ▀▀▀▀   ▀▀    ▀▀▀           ▀▀▀▀▀▀▀▀  ▀▀    ▀▀     ▀▀▀▀     ▀▀▀▀▀    ▄▀▀▀ ██    */
///                                                                                                ▀████▀▀    */
///                                                                                                           */
//////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void update_external_charger(void);

void test_charger_integration_1(void) {
    // faked init value for test to run smoothly
    pylontech.voltage_dV = 4131;
    pylontech.precise_voltage_mV = 413100;
    pylontech.precise_current_mA = 6200;
    pylontech.precise_wattage = 0;
    knobs.forced_wattage = 0;
    knobs.cell_voltage_limited_charge = 35;
    knobs.limited_charge_wattage = 180;
    knobs.max_charge_voltage = 36;
    knobs.allowed_charge_wattage = 1000;
    pylontech.bmu_idx = 8;
    pylontech.max_charge_dA = 255;
    knobs.charger_start_soc = 25;
    knobs.charger_stop_soc = 55;
    pylontech.soc = 15;
    pylontech.tcellmax = 200;
    knobs.max_charge_temperature = 400;

    uint16_t target_dA = 1;
    pylontech.current_dA = 0;

    pylontech.vcell_highest = 3400;
    pylontech.vcellmax = pylontech.vcell_highest/100;

    charger.out_voltage = pylontech.voltage_dV; 
    charger.charge_enabled = 0;

    memset(&charger_pid, 0, sizeof(charger_pid));
    charger_pid.kp_x100 = 100;
    charger_pid.ki_up_x100 = 10;
    charger_pid.ki_down_x100 = 20;
    charger_pid.kd_x100 = 10;

    charger_pid.max_step_up_dA = 5;
    charger_pid.max_step_down_dA = 5;

    charger_pid.max_energy_step_dA = 50;
    charger_pid.energy_deadband_dA = 2;

    charger_pid.min_current_offset_dA = 0;

    charger_pid.inverter_offset_dA = 0; // observed max offset internally consumed by the inverter (more surely expressed as something related to power of the link)

    // allow to respect the target as a measured value, not as a returned max allowed value
    charger_pid.compensate_measure = 1;

    auto_bat_charge = 1;


    int32_t missed_integral_x10 = 0;
    int avg_chg = 0;
    int avg_cnt = 0;
    int avg_meas = 0;
    for (int t = 0; t < 1000; t++)
    {
        update_external_charger();
        printf("%d %d \tmeas:%d\tchgr:%d v:%d\n",__LINE__, t,
                   pylontech.current_dA,
                   charger.max_charge_current,
                   pylontech.vcell_highest);

        //MODEL: charge current
        // adjust current smoothly from computed charge request
        int16_t delta = (charger.max_charge_current-pylontech.current_dA);
        #define CMD_CHG_DIV 5
        missed_integral_x10 += delta;
        if (missed_integral_x10 >= CMD_CHG_DIV || missed_integral_x10 <= -CMD_CHG_DIV) {
            pylontech.current_dA += missed_integral_x10/CMD_CHG_DIV;
            missed_integral_x10 -= (missed_integral_x10/CMD_CHG_DIV)*CMD_CHG_DIV;
        }

        // MODEL cell voltage
        pylontech.vcell_highest += pylontech.current_dA*10/30;
        pylontech.vcell_highest-=2; // natural relaxation
        pylontech.vcellmax = pylontech.vcell_highest/100;


        if (charger.charge_enabled) {
            avg_chg += charger.max_charge_current;
            avg_meas += pylontech.current_dA;
            avg_cnt ++;
        }
    }

    printf("avgchg:%d avgmeas:%d\n", avg_chg /avg_cnt, avg_meas / avg_cnt);
    assert( avg_meas / avg_cnt > 2* (knobs.allowed_charge_wattage * 100 / pylontech.voltage_dV) / 3);
    assert( avg_meas / avg_cnt < 4* (knobs.allowed_charge_wattage * 100 / pylontech.voltage_dV) / 3);
}

////////////////////////////////////////*/
///                                     */
///        ▄▄▄▄      ▄▄     ▄▄▄   ▄▄    */
///      ██▀▀▀▀█    ████    ███   ██    */
///     ██▀         ████    ██▀█  ██    */
///     ██         ██  ██   ██ ██ ██    */
///     ██▄        ██████   ██  █▄██    */
///      ██▄▄▄▄█  ▄██  ██▄  ██   ███    */
///        ▀▀▀▀   ▀▀    ▀▀  ▀▀   ▀▀▀    */
///                                     */
///                                     */
////////////////////////////////////////*/

__attribute__((weak)) void transcharge_disable_all(void) {

}
__attribute__((weak)) void offgrid_switch(uint32_t eps_mode_requested) {

}

__attribute__((weak)) void master_log_hex(void* data, size_t len) {

}

uint8_t can_inv_tx_buffer[2*(8+4)];
uint32_t can_inv_tx_offset;
__attribute__((weak)) void can_inv_tx_log(uint32_t cid, size_t cid_bitlen, uint8_t* canmsg, size_t canmsg_len)
{
    if (can_inv_tx_offset<sizeof(can_inv_tx_buffer)) {
        can_inv_tx_buffer[can_inv_tx_offset++] = cid>>24;
        can_inv_tx_buffer[can_inv_tx_offset++] = cid>>16;
        can_inv_tx_buffer[can_inv_tx_offset++] = cid>>8;
        can_inv_tx_buffer[can_inv_tx_offset++] = cid;
        memmove(can_inv_tx_buffer+can_inv_tx_offset, canmsg, canmsg_len);
        can_inv_tx_offset+=canmsg_len;
    }
}
uint8_t can_bms_tx_buffer[2*(8+4)];
uint32_t can_bms_tx_offset;
__attribute__((weak)) void can_bms_tx_log(uint32_t cid, size_t cid_bitlen, uint8_t* canmsg, size_t canmsg_len) 
{
    if (can_bms_tx_offset<sizeof(can_bms_tx_buffer)) {
        can_bms_tx_buffer[can_bms_tx_offset++] = cid>>24;
        can_bms_tx_buffer[can_bms_tx_offset++] = cid>>16;
        can_bms_tx_buffer[can_bms_tx_offset++] = cid>>8;
        can_bms_tx_buffer[can_bms_tx_offset++] = cid;
        memmove(can_bms_tx_buffer+can_bms_tx_offset, canmsg, canmsg_len);
        can_bms_tx_offset+=canmsg_len;
    }
}

void test_can_inv_interp(void) {
    memset(&pylontech, 0, sizeof(pylontech));
    memset(&pylontech_pid, 0, sizeof(pylontech_pid));
    memset(&solax, 0, sizeof(solax));
    can_bms_tx_offset=0;
    can_inv_tx_offset=0;

    assert(can_inv_interp(0x1, 29, "", 8) == 0);
    assert(can_inv_interp(0x1871, 29, "\x00", 8) == 0);
    assert(can_inv_interp(0x1871, 29, "\x01", 8) == 1);
    assert(solax.powered_on == 0);
    assert(can_inv_interp(0x1871, 29, "\x01\x00\x01", 8) == 1);
    assert(solax.powered_on == 1);
    assert(can_inv_interp(0x1871, 29, "\x02", 8) == 0);
    assert(can_inv_interp(0x1871, 29, "\x03", 8) == 0);
    assert(can_inv_interp(0x1871, 29, "\x04", 8) == 0);
    assert(can_inv_interp(0x1871, 29, "\x05", 8) == 0);

    can_bms_tx_offset=0;
    assert(can_inv_interp(0x4200, 29, "\x00", 8) == 1);
    assert(can_bms_tx_offset == 4+8);
    assert(memcmp(can_bms_tx_buffer,"\x00\x00\x42\x00\x02\x00\x00\x00\x00\x00\x00\x00",4+8)==0);

    assert(can_inv_interp(0x4200, 29, "\x01", 8) == 1);
    can_bms_tx_offset=0;
    assert(can_inv_interp(0x4200, 29, "\x02", 8) == 1);
    assert(can_bms_tx_offset == 0);
    assert(can_inv_interp(0x4200, 29, "", 8) == 1);
    assert(can_inv_interp(0x4210, 29, "", 8) == 0);
}

void test_can_bms_interp_solax(void) {
    memset(&pylontech, 0, sizeof(pylontech));
    memset(&pylontech_pid, 0, sizeof(pylontech_pid));
    memset(&solax, 0, sizeof(solax));
    can_bms_tx_offset=0;
    can_inv_tx_offset=0;
    char* p = NULL;

    assert(can_bms_interp(0x00001872,29,(p=hex2bin("e010980d0a00fa00",NULL)),8)==1);
    free(p);
    assert(can_bms_interp(0x00001873,29,(p=hex2bin("6410070062005907",NULL)),8)==1);
    free(p);
    assert(can_bms_interp(0x00001874,29,(p=hex2bin("0401f00023002200",NULL)),8)==0);
    free(p);
    assert(can_bms_interp(0x00001875,29,(p=hex2bin("540108000100f700",NULL)),8)==1);
    free(p);
    assert(can_bms_interp(0x00001876,29,(p=hex2bin("0100000000000000",NULL)),8)==0);
    free(p);
    assert(can_bms_interp(0x00001877,29,(p=hex2bin("0000000001000205",NULL)),8)==1);
    assert(memcmp(bin2hex(p,8),"0000000083000000",8)==0);
    free(p);
}

// test fix2_31_
void test_can_bms_interp_fix_2_31_solax(void) {
    memset(&pylontech, 0, sizeof(pylontech));
    memset(&pylontech_pid, 0, sizeof(pylontech_pid));
    memset(&solax, 0, sizeof(solax));
    can_bms_tx_offset=0;
    can_inv_tx_offset=0;
    char* p = NULL;

    assert(can_bms_interp(0x00001872,29,(p=hex2bin("e010980d00000000",NULL)),8)==1);
    assert(pylontech.fix2_31);
    free(p);
    assert(can_bms_interp(0x00001873,29,(p=hex2bin("6410070062005907",NULL)),8)==1);
    free(p);
    assert(can_bms_interp(0x00001874,29,(p=hex2bin("0401f00023002200",NULL)),8)==0);
    free(p);
    assert(can_bms_interp(0x00001875,29,(p=hex2bin("540108000100f700",NULL)),8)==1);
    free(p);
    assert(can_bms_interp(0x00001876,29,(p=hex2bin("0100000000000000",NULL)),8)==0);
    free(p);
    assert(can_bms_interp(0x00001877,29,(p=hex2bin("0000000001000205",NULL)),8)==1);
    assert(strcasecmp(bin2hex(p,8),"0000000083000000")==0);
    free(p);
}

void test_can_bms_interp_sc0500(void) {
    memset(&pylontech, 0, sizeof(pylontech));
    memset(&pylontech_pid, 0, sizeof(pylontech_pid));
    memset(&solax, 0, sizeof(solax));
    can_bms_tx_offset=0;
    can_inv_tx_offset=0;
    char* p = NULL;

    pylontech.max_discharge_dA = 250;
    pylontech.max_charge_dA = 250;

    assert(can_bms_interp(0x00004210,29,hex2bin("6b0e 3075 3205 6463",NULL),8)==1);
    assert(can_bms_interp(0x00004220,29,(p=hex2bin("4002 203A 2A76 3674",NULL)),8)==1);
    printf("%s\n",bin2hex(p,8));
    assert(strcasecmp(bin2hex(p,8),"4002203A30753674")==0);
    assert(can_bms_interp(0x00004230,29,hex2bin("0000000000000000",NULL),8)==1);
    assert(can_bms_interp(0x00004240,29,hex2bin("e803e80300000000",NULL),8)==1);
    assert(can_bms_interp(0x00004250,29,hex2bin("0302010000000000",NULL),8)==1);
    assert(can_bms_interp(0x00004260,29,hex2bin("0000000000000000",NULL),8)==0);
    assert(can_bms_interp(0x00004270,29,hex2bin("e803e80300000000",NULL),8)==0);
    assert(can_bms_interp(0x00004280,29,hex2bin("aaaa006300000000",NULL),8)==0);
    assert(can_bms_interp(0x00004290,29,hex2bin("0000000000000000",NULL),8)==0);
    assert(can_bms_interp(0x000042a0,29,hex2bin("0000000000000000",NULL),8)==0);
    assert(can_bms_interp(0x00007310,29,hex2bin("010010020502341c",NULL),8)==1);
    assert(can_bms_interp(0x00007320,29,hex2bin("6900070f50013200",NULL),8)==1);
    assert(can_bms_interp(0x00007330,29,hex2bin("50594c4f4e544543",NULL),8)==1);
    assert(can_bms_interp(0x00007340,29,hex2bin("4800000000000000",NULL),8)==1);
}

void test_can_bms_interp_sc0500_fix_discharge(void) {
    memset(&pylontech, 0, sizeof(pylontech));
    memset(&pylontech_pid, 0, sizeof(pylontech_pid));
    memset(&solax, 0, sizeof(solax));
    can_bms_tx_offset=0;
    can_inv_tx_offset=0;
    char* p = NULL;

    pylontech.max_discharge_dA = 250;
    pylontech.max_charge_dA = 250;

    assert(can_bms_interp(0x00004210,29,hex2bin("6b0e 3075 3205 6463",NULL),8)==1);
    assert(can_bms_interp(0x00004220,29,(p=hex2bin("4002 203A 2A76 3075",NULL)),8)==1);
    printf("%s\n",bin2hex(p,8));
    assert(strcasecmp(bin2hex(p,8),"4002203A30753674")==0);
    assert(can_bms_interp(0x00004230,29,hex2bin("0000000000000000",NULL),8)==1);
    assert(can_bms_interp(0x00004240,29,hex2bin("e803e80300000000",NULL),8)==1);
    assert(can_bms_interp(0x00004250,29,hex2bin("0302010000000000",NULL),8)==1);
    assert(can_bms_interp(0x00004260,29,hex2bin("0000000000000000",NULL),8)==0);
    assert(can_bms_interp(0x00004270,29,hex2bin("e803e80300000000",NULL),8)==0);
    assert(can_bms_interp(0x00004280,29,hex2bin("aaaa006300000000",NULL),8)==0);
    assert(can_bms_interp(0x00004290,29,hex2bin("0000000000000000",NULL),8)==0);
    assert(can_bms_interp(0x000042a0,29,hex2bin("0000000000000000",NULL),8)==0);
    assert(can_bms_interp(0x00007310,29,hex2bin("010010020502341c",NULL),8)==1);
    assert(can_bms_interp(0x00007320,29,hex2bin("6900070f50013200",NULL),8)==1);
    assert(can_bms_interp(0x00007330,29,hex2bin("50594c4f4e544543",NULL),8)==1);
    assert(can_bms_interp(0x00007340,29,hex2bin("4800000000000000",NULL),8)==1);
}

void test_can_bms_interp_fix_2_31_sc0500(void) {
    memset(&pylontech, 0, sizeof(pylontech));
    memset(&pylontech_pid, 0, sizeof(pylontech_pid));
    memset(&solax, 0, sizeof(solax));
    can_bms_tx_offset=0;
    can_inv_tx_offset=0;
    char* p = NULL;

    pylontech.max_discharge_dA = 250;
    pylontech.max_charge_dA = 250;

    assert(can_bms_interp(0x00004210,29,hex2bin("6b0e307532056463",NULL),8)==1);
    assert(can_bms_interp(0x00004220,29,(p=hex2bin("4002203a30753075",NULL)),8)==1);
    assert(pylontech.fix2_31);
    printf("%s\n",bin2hex(p,8));
    assert(strcasecmp(bin2hex(p,8),"4002203A30753674")==0);
    assert(can_bms_interp(0x00004230,29,hex2bin("0000000000000000",NULL),8)==1);
    assert(can_bms_interp(0x00004240,29,hex2bin("e803e80300000000",NULL),8)==1);
    assert(can_bms_interp(0x00004250,29,hex2bin("0302010000000000",NULL),8)==1);
    assert(can_bms_interp(0x00004260,29,hex2bin("0000000000000000",NULL),8)==0);
    assert(can_bms_interp(0x00004270,29,hex2bin("e803e80300000000",NULL),8)==0);
    assert(can_bms_interp(0x00004280,29,hex2bin("aaaa006300000000",NULL),8)==0);
    assert(can_bms_interp(0x00004290,29,hex2bin("0000000000000000",NULL),8)==0);
    assert(can_bms_interp(0x000042a0,29,hex2bin("0000000000000000",NULL),8)==0);
    assert(can_bms_interp(0x00007310,29,hex2bin("010010020502341c",NULL),8)==1);
    assert(can_bms_interp(0x00007320,29,hex2bin("6900070f50013200",NULL),8)==1);
    assert(can_bms_interp(0x00007330,29,hex2bin("50594c4f4e544543",NULL),8)==1);
    assert(can_bms_interp(0x00007340,29,hex2bin("4800000000000000",NULL),8)==1);
}


//////////////////////////////////////////////////*/
///                                               */
///     ▄▄▄  ▄▄▄     ▄▄      ▄▄▄▄▄▄   ▄▄▄   ▄▄    */
///     ███  ███    ████     ▀▀██▀▀   ███   ██    */
///     ████████    ████       ██     ██▀█  ██    */
///     ██ ██ ██   ██  ██      ██     ██ ██ ██    */
///     ██ ▀▀ ██   ██████      ██     ██  █▄██    */
///     ██    ██  ▄██  ██▄   ▄▄██▄▄   ██   ███    */
///     ▀▀    ▀▀  ▀▀    ▀▀   ▀▀▀▀▀▀   ▀▀   ▀▀▀    */
///                                               */
///                                               */
//////////////////////////////////////////////////*/
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
    // printf("test_hysteresis_cycle\n");
    // test_hysteresis_cycle();
    // printf("test_full_sun_day\n");
    // test_full_sun_day();
    printf("test_load_disturbance_rejection\n");
    test_load_disturbance_rejection();
    printf("test_allowed_never_exceeds_target_plus_offset\n");
    test_allowed_never_exceeds_target_plus_offset();


    // charge update global tests
    printf("test_update_charge\n");
    test_update_charge();
    printf("test_update_charge2\n");
    test_update_charge2();
    printf("test_maintain_top_up\n");
    test_maintain_top_up();
    printf("test_pid_delayed_feedback_integral\n");
    test_pid_delayed_feedback_integral();



    // test PID to drive the grid charger (instead of relying on the relay circuit breaker, which clearly 
    // disturb some appliances such as the heat pump, and which tend to have a small switching delay, implying
    // a short power outage.
    printf("test_pid_for_charger_1\n");
    test_pid_for_charger_1();
    printf("test_pid_for_charger_2\n");
    test_pid_for_charger_2();

    
    printf("test_charger_integration_1\n");
    test_charger_integration_1();

    printf("test INV CAN interp\n");
    test_can_inv_interp();

    printf("--test BMS CAN interp\n");
    test_can_bms_interp_solax();
    printf("--test BMS CAN interp\n");
    test_can_bms_interp_fix_2_31_solax();
    printf("--test BMS CAN interp\n");
    test_can_bms_interp_sc0500();
    printf("--test BMS CAN interp\n");
    test_can_bms_interp_sc0500_fix_discharge();
    printf("--test BMS CAN interp\n");
    test_can_bms_interp_fix_2_31_sc0500();

    return 0;
}
#endif // X86

/*
prompts to generate the testcases:
=================================

Ok, let's rethink all the tests, and use a commond init_pid function that initialize the pid variable at the start of each test 
I want few tests, here are each one description:
- a test to ensure the PID is not sensitive of measured power variation each cycle vs the target requested.
- a test that ensure target requested may be over the measured value, measured value is bound to effective power received by the PVs
- a test that ensure a minimum when full
- a test that tries to compensate discharge when battery is full (usually that means the allowed charge is really small) => and therefore the inverter tends to lock up until battery are drained a bit. but this is workaroundable with a higher allowed charge current
- a test that make sure the PID compensate when there is a positive offset between allowed charge and effective charge
- a test to ensure PID recovery after a low measured power (due to clouding)
- 200 samples inverter offset change, PID should adapt (the measured current is either some dA higher than allowed for 200 samples, or few dA lower than allowed for 200 samples)
- a test that finish the charge (full soc), to ensure that when battery are over the max voltage, then it pauses until min voltage is reached before performing charge again (hysterisis is respected), and with a minimal chrge current
- a full sun day scenario test, batteries are low, charge starts with high current, then reach a level when charge must be reduced to avoid going too fast to hgih voltage that stops the charge, then yet another level reducing the charge again, and then finally reaching hysterisis stop voltage (the battery is marked full before that moment!), then we wait until voltage goes below the min hysterisis to top up the charge again. In that test, it is tthe target current which is decreased at each level, to avoid reaching too high battery voltage too hastily.
- a test where battery is full (meaning the target current is minimal or 0), and the battery gets discharged because of home consumption, therefore the measured current gets negative, the PID has to compensate to ensure the residual charge is at least positive. And adjust after the load is disconnected, and therefore the compensation charge will be too much
- a test that ensure the allowed dA cannot go over target (255dA) + inverter_offset_dA, I have a log where the pv is limited, therefore measure is 16dA, target is 255dA, and the allowed dA goes 50dA, 100dA, 150dA, 200dA, 250dA, 300dA... this is not expected, it shall be limited.

- a test that reproduces the negative charge offset (the PID may want a negative offset)
- a test to check that when the measured current has a delay of multiple samples

/!\ take care of the right modelisation for battery voltage/measured charge current (depending on fake pv power)

*/
