#include "assert.h"
#include "stdint.h"
#include "stdio.h"

uint32_t uwTick;
#define EXPIRED_DIFF(timestamp) ((uint32_t)uwTick - (uint32_t)(timestamp))
#define EXPIRED(timestamp) (EXPIRED_DIFF(timestamp) < (uint32_t)0x80000000)

void main(void) {
	uwTick = 0x100;
	assert (EXPIRED(0));
	assert (EXPIRED(1));
	assert (EXPIRED(0xFF));
	assert (EXPIRED(0x100));
	assert (!EXPIRED(0x101));

	assert (EXPIRED(0xFFFFFFFF));
	assert (!EXPIRED(0x80000000));
	assert (!EXPIRED(0x800000FF));
	assert (!EXPIRED(0x80000100));
	assert (EXPIRED(0x80000101));

	uwTick = 0x80000010;
	assert (EXPIRED(0x80000001));
	assert (EXPIRED(0x20000000));
	assert (!EXPIRED(0x10));
	assert (EXPIRED(0x11));
	assert (EXPIRED(0x80000010));
	assert (!EXPIRED(0x80000011));
	assert (!EXPIRED(0xFFFFFFFF));
}