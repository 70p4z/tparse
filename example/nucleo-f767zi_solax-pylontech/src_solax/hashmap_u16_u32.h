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

#ifndef HASHMAP_U16_U32_H
#define HASHMAP_U16_U32_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* ── configuration ───────────────────────────────────────────────────────── */

#ifndef HM_CAPACITY
#  define HM_CAPACITY 512
#endif

#ifndef HM_EMPTY_KEY
#  define HM_EMPTY_KEY  ((uint16_t)0xFFFF)
#endif

#ifndef HM_TOMB_KEY
#  define HM_TOMB_KEY   ((uint16_t)0xFFFE)
#endif

_Static_assert((HM_CAPACITY & (HM_CAPACITY - 1)) == 0,
               "HM_CAPACITY must be a power of two");
_Static_assert(HM_CAPACITY >= 2,
               "HM_CAPACITY must be at least 2");

/* ── types ───────────────────────────────────────────────────────────────── */

typedef struct {
    uint16_t key;
    uint32_t val;
} hm_slot_t;

typedef struct {
    hm_slot_t slots[HM_CAPACITY];
    uint16_t  count;    /* live entries   */
    uint16_t  tombs;    /* tombstone slots */
} hm_t;

/* ── internal helpers ────────────────────────────────────────────────────── */

/* Fibonacci hashing for uint16_t */
uint16_t hm__hash(uint16_t k);
void hm_init(hm_t *m);
uint16_t hm_count(const hm_t *m);
bool hm_put(hm_t *m, uint16_t key, uint32_t val);
bool hm_get(const hm_t *m, uint16_t key, uint32_t *val_out);
bool hm_del(hm_t *m, uint16_t key);
bool hm_iter(const hm_t *m, uint16_t *iter,
                            uint16_t *key_out, uint32_t *val_out);

#endif /* HASHMAP_U16_U32_H */