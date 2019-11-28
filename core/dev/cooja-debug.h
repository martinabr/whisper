/*
 * cooja-debug.h
 *
 *  Created on: Jan 12, 2011
 *      Author: simonduq
 */

#ifndef COOJA_DEBUG_H_
#define COOJA_DEBUG_H_

#if DISABLE_COOJA_DEBUG

#define HEXC(c)
#define COOJA_DEBUG_STR(str)
#define COOJA_DEBUG_POINT
#define COOJA_DEBUG_CINT(c, val)
#define COOJA_DEBUG_INT(val)
#define COOJA_DEBUG_INTH(val)
#define COOJA_DEBUG_PRINTF(...)
#define COOJA_DEBUG_ADDR(addr)

#else

#include <stdio.h>
#include <stdlib.h>
#include "dev/leds.h"

volatile char *cooja_debug_ptr;
volatile char cooja_debug_point;

#define HEXC(c) (((c) & 0xf) <= 9 ? ((c) & 0xf) + '0' : ((c) & 0xf) + 'a' - 10)

/* Print a simple string. */
#define COOJA_DEBUG_STR(str) do { cooja_debug_ptr = str; } while(0);
#define COOJA_DEBUG_C(c) do { char tmp[10]; cooja_debug_ptr = itoa(c, tmp, 10); } while(0);

/* Print a 8 bit, or 16 bit integer. */
#define COOJA_DEBUG_INT(val) do { char tmp[10] = {0}; uint32_t v = (uint32_t)(val); int i=9; if(v==0) tmp[--i] = '0'; while(v) { tmp[--i] = '0' + v%10; v /= 10; } cooja_debug_ptr = tmp+i;  } while(0)
/* Use like printf. */
#define COOJA_DEBUG_PRINTF(...) do { char tmp[200]; snprintf(tmp, sizeof(tmp), __VA_ARGS__); cooja_debug_ptr = tmp; } while(0);

#define COOJA_DEBUG_PRINT_INT(C) do { char tmp[5]; snprintf(tmp, sizeof(tmp), "%d", C); cooja_debug_ptr = tmp; } while(0);

#define COOJA_DEBUG_PRINT_UINT_16(C) do { char tmp[10]; snprintf(tmp, sizeof(tmp), "%u", C); cooja_debug_ptr = tmp; } while(0);

#define COOJA_DEBUG_PRINT_UINT_32(C) do { char tmp[10]; snprintf(tmp, sizeof(tmp), "%lu", C); cooja_debug_ptr = tmp; } while(0);

#define COOJA_DEBUG_PRINT_HEX(C) do { char tmp[6]; snprintf(tmp, sizeof(tmp), "%x", C); cooja_debug_ptr = tmp; } while(0);
#endif /* DISABLE_COOJA_DEBUG */
#endif /* COOJA_DEBUG_H_ */
