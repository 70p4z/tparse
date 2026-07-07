/*
 * hashmap_u16_u32.h — compact open-addressing hashmap, uint16_t -> uint32_t
 *
 * Single-header, no dependencies, C99.
 * Drop this file into your project and #include it.
 *
 * Usage:
 *   #define HM_CAPACITY 64          // must be a power of two
 *   #include "hashmap_u16_u32.h"
 *
 *   hm_t map;
 *   hm_init(&map);
 *   hm_put(&map, 0x1234, 0xDEADBEEF);
 *
 *   uint32_t val;
 *   if (hm_get(&map, 0x1234, &val)) { ... }
 *
 *   hm_del(&map, 0x1234);
 *
 * Configuration macros (define before #include):
 *   HM_CAPACITY   — number of slots (default 64, must be power of two)
 *   HM_EMPTY_KEY  — sentinel for "slot is empty" (default 0xFFFF)
 *   HM_TOMB_KEY   — sentinel for "slot was deleted" (default 0xFFFE)
 *                   Keys equal to these two values cannot be stored.
 */


#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "hashmap_u16_u32.h"

/* ── internal helpers ────────────────────────────────────────────────────── */

#define HM_MASK  ((uint16_t)(HM_CAPACITY - 1))

/* Fibonacci hashing for uint16_t */
uint16_t hm__hash(uint16_t k)
{
    return (uint16_t)((k * (uint16_t)0x9E3Bu) >> (16 - /* log2 cap */
        (HM_CAPACITY ==    2 ?  1 :
         HM_CAPACITY ==    4 ?  2 :
         HM_CAPACITY ==    8 ?  3 :
         HM_CAPACITY ==   16 ?  4 :
         HM_CAPACITY ==   32 ?  5 :
         HM_CAPACITY ==   64 ?  6 :
         HM_CAPACITY ==  128 ?  7 :
         HM_CAPACITY ==  256 ?  8 :
         HM_CAPACITY ==  512 ?  9 :
         HM_CAPACITY == 1024 ? 10 :
         HM_CAPACITY == 2048 ? 11 :
         HM_CAPACITY == 4096 ? 12 : 6)));
}

/* ── public API ──────────────────────────────────────────────────────────── */

/* Initialise (or reset) a map. */
void hm_init(hm_t *m)
{
    m->count = 0;
    m->tombs = 0;
    for (uint16_t i = 0; i < HM_CAPACITY; i++)
        m->slots[i].key = HM_EMPTY_KEY;
}

/* Returns number of live entries. */
uint16_t hm_count(const hm_t *m) { return m->count; }

/*
 * Insert or update.
 * Returns true on success, false when the map is full.
 * Asserts that key is not a sentinel value.
 */
bool hm_put(hm_t *m, uint16_t key, uint32_t val)
{
    /* caller must not use sentinel keys */
    if (key == HM_EMPTY_KEY || key == HM_TOMB_KEY) return false;
    /* refuse insert when load >= 75% (count + tombs) */
    if ((uint16_t)(m->count + m->tombs) >= (uint16_t)(HM_CAPACITY * 3 / 4))
        return false;

    uint16_t idx  = hm__hash(key);
    uint16_t tomb = (uint16_t)HM_CAPACITY; /* index of first tombstone seen */

    for (uint16_t i = 0; i < HM_CAPACITY; i++) {
        uint16_t slot = (uint16_t)((idx + i) & HM_MASK);
        uint16_t sk   = m->slots[slot].key;

        if (sk == key) {
            m->slots[slot].val = val;   /* update existing */
            return true;
        }
        if (sk == HM_TOMB_KEY) {
            if (tomb == (uint16_t)HM_CAPACITY) tomb = slot;
        } else if (sk == HM_EMPTY_KEY) {
            /* place at tombstone if we passed one, else here */
            if (tomb < (uint16_t)HM_CAPACITY) {
                m->slots[tomb].key = key;
                m->slots[tomb].val = val;
                m->tombs--;
            } else {
                m->slots[slot].key = key;
                m->slots[slot].val = val;
            }
            m->count++;
            return true;
        }
    }

    /* map is full (shouldn't happen with the load check above) */
    return false;
}

/*
 * Look up a key.
 * Returns true and writes *val_out on hit; returns false on miss.
 * val_out may be NULL if you only need existence.
 */
bool hm_get(const hm_t *m, uint16_t key, uint32_t *val_out)
{
    if (key == HM_EMPTY_KEY || key == HM_TOMB_KEY) return false;

    uint16_t idx = hm__hash(key);
    for (uint16_t i = 0; i < HM_CAPACITY; i++) {
        uint16_t slot = (uint16_t)((idx + i) & HM_MASK);
        uint16_t sk   = m->slots[slot].key;

        if (sk == key) {
            if (val_out) *val_out = m->slots[slot].val;
            return true;
        }
        if (sk == HM_EMPTY_KEY) return false; /* probe chain broken */
    }
    return false;
}

/*
 * Delete a key.
 * Returns true if the key was present, false if it wasn't.
 */
bool hm_del(hm_t *m, uint16_t key)
{
    if (key == HM_EMPTY_KEY || key == HM_TOMB_KEY) return false;

    uint16_t idx = hm__hash(key);
    for (uint16_t i = 0; i < HM_CAPACITY; i++) {
        uint16_t slot = (uint16_t)((idx + i) & HM_MASK);
        uint16_t sk   = m->slots[slot].key;

        if (sk == key) {
            m->slots[slot].key = HM_TOMB_KEY;
            m->count--;
            m->tombs++;
            return true;
        }
        if (sk == HM_EMPTY_KEY) return false;
    }
    return false;
}

/*
 * Iterate over all live entries.
 * Start with *iter = 0. Returns true while entries remain,
 * writing the current key/val. Advance with (*iter)++.
 *
 * Example:
 *   uint16_t it = 0, k; uint32_t v;
 *   while (hm_iter(&map, &it, &k, &v)) { ...; it++; }
 */
bool hm_iter(const hm_t *m, uint16_t *iter,
                            uint16_t *key_out, uint32_t *val_out)
{
    while (*iter < (uint16_t)HM_CAPACITY) {
        uint16_t sk = m->slots[*iter].key;
        if (sk != HM_EMPTY_KEY && sk != HM_TOMB_KEY) {
            if (key_out) *key_out = sk;
            if (val_out) *val_out = m->slots[*iter].val;
            return true;
        }
        (*iter)++;
    }
    return false;
}

#undef HM_MASK
