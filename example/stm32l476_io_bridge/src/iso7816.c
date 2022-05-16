#include "iso7816.h"
#include "string.h"

#define NO_TIMEOUT 0
#define TIMEOUT_1S 1000

uint8_t iso_usart_recv_atr_byte(uint8_t* atr) {
  if (iso_usart_recv(atr, 1, TIMEOUT_1S) != 1) {
	  return 0;
  }
  return *atr;
}

const uint16_t ISO7816_TA_F[] =   {
	372,    372,    558,    744,
	1116,   1488,   1860,   0,
	256,    512,    768,    1024,
	1536,   2048,   0,      0   };
const uint16_t ISO7816_TA_D[] =   {
	1,      1,      2,      4,
	8,      16,     32,     64,
	12,     20,     1,      1,
	1,      372,    256,    128 };

// convert TA to ETU clk ticks
#define ETU_FROM_TA(TA) ((ISO7816_TA_F[TA>>4]) / ((ISO7816_TA_D[TA&0x0F])))

void iso_usart_TA(uint8_t TA) {
  iso_usart_ETU(ETU_FROM_TA(TA));
}

size_t iso_atr(uint8_t* atr, size_t atr_max_len) {
  uint8_t T_not_0 = 0;
  uint8_t TD = 0;
  uint8_t TA[2];
  uint8_t i,c;
  uint8_t* atr_p = atr;
  uint8_t pps[4];
  uint8_t ppsr[4];

  memset(atr, 0, atr_max_len);
  // ATR reception is done at TA=11 <=> ETU=372
  iso_usart_TA(0x11);
  iso_rst(0);
  iso_delay_ms(5);
  iso_usart_flush();

  iso_rst(1);

  // ATR TS and ATR T0
  if (iso_usart_recv(atr, 2, TIMEOUT_1S)!=2) {
	  return 0;
  }
  TD = atr[1];
  // check TS is expected
  switch(atr[0]) {
	  case 0x3B:
	  case 0x3F:
		  break;
	  default:
		  return 0;
  }
  atr_p = atr + 2;
  // receive the whole decoded ATR structure
  // note i are translated for -1.
  i = 0;
  TA[0] = 0;
  TA[1] = 0;
  // avoid overflown atr
  while((TD&0xF0)
		&& ((uintptr_t)atr_p - (uintptr_t)atr) < atr_max_len
		&& i < 4) {
    // TAi
    if(TD&0x10) {
      c = iso_usart_recv_atr_byte(atr_p++);
      // store first TAs
      if (i < 2) {
        TA[i] = c;
      }
    }
    // TBi
    if(TD&0x20) {
      iso_usart_recv_atr_byte(atr_p++);
    }
    // TCi
    if(TD&0x40) {
      iso_usart_recv_atr_byte(atr_p++);
    }
    // TDi
    if(TD&0x80) {
      TD = iso_usart_recv_atr_byte(atr_p++);
      i++;

      // check need for TCK (not in T0, which encode K instead of a protocol)
      if (TD&0xF) {
        T_not_0 = 1;
      }
    }
    else {
      // last TD received
      TD = 0;
    }
  }
  // historical bytes
  if ((atr[1] & 0xF)) {
    atr_p += iso_usart_recv(atr_p, atr[1] & 0xF, TIMEOUT_1S);
  }
  // receive TCK byte if mandatory
  if (T_not_0) {
    uint8_t tck;
    iso_usart_recv_atr_byte(atr_p++);

    // compute expected TCK
    tck = 0;
    for (i = 1; i < ((uintptr_t)atr_p - (uintptr_t)atr) ; i++) {
      tck ^= atr[i];
    }
    if (tck != 0) {
      // TODO tck is invalid, ATR shall be discarded
    }
  }
  // nickname TA array for readability
  #define TA1 TA[0]
  #define TA2 TA[1]
  if (TA2 != 0 && TA2 != 0x80) {
    // unsupported mode
    return 0;
  }
  else if (TA2) {
    // specific mode
    iso_usart_TA(TA1);
  }
  else if (TA1 != 0x11 && TA1 != 0) {
	// negociable mode and non default value
	iso_delay_ms(5);
#define PPS0 0xFF
	// PPS
    pps[0] = PPS0;
    pps[1] = 0x10;
    pps[2] = TA1;
    pps[3] = PPS0^0x10^TA1;
    iso_usart_send(pps, 4);
    // PPSS/PPS0
    if (iso_usart_recv(ppsr, 2, TIMEOUT_1S) != 2
        || ppsr[0] != PPS0) {
      return 0;
    }
    // interpret PPS0
    switch(ppsr[1]) {
      // unsupported value
      default:
        return 0;
      // refused
      case 0:
        // PCK
        if (iso_usart_recv(ppsr+2, 1, TIMEOUT_1S) != 1) {
        	return 0;
        }
        TA1 = 0x11; // in case error, use default PPS
        break;
      // accepted
      case 0x10:
        // PPSA/PCK
        if (iso_usart_recv(ppsr+2, 2, TIMEOUT_1S) != 2) {
        	return 0;
        }
        TA1 = ppsr[2];
        break;
    }
    iso_usart_TA(TA1);
  }
  return (uintptr_t)atr_p - (uintptr_t)atr;
}

void iso_power_down(void) {
	iso_gnd(1);
	iso_rst(1);
}

size_t iso_powercycle(uint8_t* atr, size_t atr_max_len) {

	iso_gnd(1);
	iso_rst(1);

	iso_delay_ms(5);

	iso_rst(0);
	iso_delay_ms(5);

	iso_gnd(0);
	iso_delay_ms(5);

    // grab ATR (force pps even if not needed)
    return iso_atr(atr, atr_max_len);
}

size_t iso_apdu_t0(uint8_t* apdu, size_t length) {

  uint8_t cmd;
  size_t rapdu_len=0;
  enum {
	  STATE_COMMAND,
	  STATE_SW2,
  } state;
  uint8_t ins;

  if (length < 5) {
    return 0;
  }
  length-=5;
  // check case 3 Lc
  if (length && length != apdu[4]) {
	  return 0;
  }

  state = STATE_COMMAND;
  // CASE 0/1
  ins = apdu[1];
  iso_usart_send(apdu, 5);
  // T=0
  while(1) {
    // sw1 / ins
    if (iso_usart_recv(&cmd, 1, TIMEOUT_1S) != 1) {
    	return 0;
    }
    switch(state) {
      case STATE_COMMAND:
        switch (cmd & 0xF0) {
          case 0x60:
          case 0x90:
        	// wait extension
            if (cmd == 0x60) {
              continue;
            }
            apdu[rapdu_len++] = cmd;
            state = STATE_SW2;
            continue;
          default:
        	// CDATA field exchange at once
            if (cmd == ins) {
              // case 3
              if (length > 0) {
                // P3 is Lc
                iso_usart_send(apdu+5,apdu[4]);
              }
              // case 2
              else {
			    // P3 is Le
                rapdu_len = (apdu[4]==0 ? 256:apdu[4]);
                if (iso_usart_recv(apdu, rapdu_len, TIMEOUT_1S) != rapdu_len) {
                	return 0;
                }
              }
              state = STATE_COMMAND;
              continue;
            }
            // CDATA byte per byte exchange
            else {
            	// TODO check that we're not receiving longer than Le
            	if (iso_usart_recv(apdu+rapdu_len, 1, TIMEOUT_1S) != 1) {
            		return 0;
            	}
            	rapdu_len++;
            	state = STATE_COMMAND;
            }
            break;
        }
        break;
      case STATE_SW2:
        apdu[rapdu_len++] = cmd;
        return rapdu_len;
    }
  }
}
