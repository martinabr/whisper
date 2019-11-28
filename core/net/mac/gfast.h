/*
 * Copyright (c) 2018, TU Dresden.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Author: Martina Brachmann <martina.brachmann@tu-dresden.de>
 *
 */

#ifndef GLOSSY_WHISPER_H_
#define GLOSSY_WHISPER_H_


#define CM_POS              CM_1
#define CM_NEG              CM_2
#define CM_BOTH             CM_3


#include "contiki.h"
#include <legacymsp430.h>
#include <msp430f1611.h>

#define ENABLE_COOJA_DEBUG 0
#include "dev/cooja-debug.h"

#include "whisper/whisper.h"
#include "glossy.h"


/**
 * If not zero, nodes print additional debug information (disabled by default).
 */
#ifndef LANEFLOOD_DEBUG
#define LANEFLOOD_DEBUG 0
#endif
#ifndef LANEFLOOD_DEBUG_PINS
#define LANEFLOOD_DEBUG_PINS 0
#endif

/**
 * Size of the window used to average estimations of slot lengths.
 */
#define GLOSSY_SYNC_WINDOW            64
#define WHISPER_SYNC_WINDOW         10

#define LANEFLOOD_TYPE                   0xc0
#define GLOSSY_TYPE                     0x40
#define WHISPER_PAYLOAD_MASK          0x1f

/**
 * The Glossy header fields
 */
typedef struct {
	uint8_t	lf_len_field;		/* The length of the packet (needs to be set for the radio) */
	uint8_t lf_relay_cnt_field;	/* The relay counter field */
	uint8_t lf_payload[];		/* Pointer to the next header or data struct */
} __attribute__((packed)) lf_header_glossy_struct;
lf_header_glossy_struct * lf_header_glossy;

/**
 * The whisper header fields
 */
typedef struct {
	uint8_t	whisper_len_field;		/* The length of the packet (needs to be set for the radio) */
    uint8_t whisper_data ;        /* whisper data consists of the identifier for lanefloods (2bit), msg type (3 bit) and the counter (3 bit) */
} __attribute__((packed)) lf_header_whisper_struct;
lf_header_whisper_struct * lf_header_whisper;

enum {
	LANEFLOOD_GLOSSY = 1, LANEFLOOD_WHISPER = 0
};

enum {
	INITIATOR = 1, RECEIVER = 0
};
/**
 * Shared variables
 */
uint8_t energest_on;
uint8_t t_ref_l_updated;
rtimer_clock_t t_ref_l;

/**
 * Shared functions
 */
uint8_t get_gfast_modus(void);
void set_gfast_modus(uint8_t modus_);
rtimer_clock_t get_t_ref_l(void);
void set_t_ref_l(rtimer_clock_t t);
uint8_t is_t_ref_l_updated(void);
void set_t_ref_l_updated(uint8_t updated);
void reset_t_ref_l_updated(void);

/** @} */

/** General callback function definition, it corresponds to a function
 * to be called in a position of the state machine table */
typedef void (*whisper_action)();
/* --------------------------- Flocklab pins ---------------------- */

#define SET_PIN(a,b)          do { P##a##OUT |=  BV(b); } while (0)
#define UNSET_PIN(a,b)        do { P##a##OUT &= ~BV(b); } while (0)
#define TOGGLE_PIN(a,b)       do { P##a##OUT ^=  BV(b); } while (0)
#define INIT_PIN_IN(a,b)      do { P##a##SEL &= ~BV(b); P##a##DIR &= ~BV(b); } while (0)
#define INIT_PIN_OUT(a,b)     do { P##a##SEL &= ~BV(b); P##a##DIR |=  BV(b); } while (0)
#define PIN_IS_SET(a,b)       (    P##a##IN  &   BV(b))

#if LOCAL_NODES || FLOCKLAB

// ADC2 (P6.2) -> LED3 (assume blue)
#define SET_PIN_ADC2         SET_PIN(6,2)
#define UNSET_PIN_ADC2       UNSET_PIN(6,2)
#define TOGGLE_PIN_ADC2      TOGGLE_PIN(6,2)
#define INIT_PIN_ADC2_IN     INIT_PIN_IN(6,2)
#define INIT_PIN_ADC2_OUT    INIT_PIN_OUT(6,2)
#define PIN_ADC2_IS_SET      PIN_IS_SET(6,2)

// ADC6 (P6.6) -> LED2 (assume green)
#define SET_PIN_ADC6         SET_PIN(6,6)
#define UNSET_PIN_ADC6       UNSET_PIN(6,6)
#define TOGGLE_PIN_ADC6      TOGGLE_PIN(6,6)
#define INIT_PIN_ADC6_IN     INIT_PIN_IN(6,6)
#define INIT_PIN_ADC6_OUT    INIT_PIN_OUT(6,6)
#define PIN_ADC6_IS_SET      PIN_IS_SET(6,6)

// ADC7 (P6.7) -> LED1 (assume red)
#define SET_PIN_ADC7         SET_PIN(6,7)
#define UNSET_PIN_ADC7       UNSET_PIN(6,7)
#define TOGGLE_PIN_ADC7      TOGGLE_PIN(6,7)
#define INIT_PIN_ADC7_IN     INIT_PIN_IN(6,7)
#define INIT_PIN_ADC7_OUT    INIT_PIN_OUT(6,7)
#define PIN_ADC7_IS_SET      PIN_IS_SET(6,7)

#if LOCAL_NODES

#define SET_PIN_GIO2        SET_PIN(2,3)
#define UNSET_PIN_GIO2      UNSET_PIN(2,3)
#define TOGGLE_PIN_GIO2      TOGGLE_PIN(2,3)
#define INIT_PIN_GIO2     INIT_PIN_IN(2,3)
#define INIT_PIN_GIO2_OUT    INIT_PIN_OUT(2,3)
#define PIN_GIO2_IS_SET      PIN_IS_SET(2,3)

#define SET_PIN_GIO3        SET_PIN(2,6)
#define UNSET_PIN_GIO3      UNSET_PIN(2,6)
#define TOGGLE_PIN_GIO3      TOGGLE_PIN(2,6)
#define INIT_PIN_GIO3     INIT_PIN_IN(2,6)
#define INIT_PIN_GIO3_OUT    INIT_PIN_OUT(2,6)
#define PIN_GIO3_IS_SET      PIN_IS_SET(2,6)

#define SET_PIN_ADC1        SET_PIN(6,1)
#define UNSET_PIN_ADC1      UNSET_PIN(6,1)
#define TOGGLE_PIN_ADC1      TOGGLE_PIN(6,1)
#define INIT_PIN_ADC1     INIT_PIN_IN(6,1)
#define INIT_PIN_ADC1_OUT    INIT_PIN_OUT(6,1)
#define PIN_ADC1_IS_SET      PIN_IS_SET(6,1)
#endif /* LOCAL_NODES */
#endif /* LOCAL_NODES || FLOCKLAB */


/* --------------------------- Radio functions ---------------------- */
//static inline void radio_abort_rx(uint8_t state_) {
//    //leds_on(LEDS_RED);
//    state = state_; //GLOSSY_STATE_ABORTED;
//    radio_flush_rx();
//    //leds_off(LEDS_RED);
//}

#if LANEFLOOD_DEBUG
uint16_t high_T_irq, rx_timeout, bad_length, bad_crc;
#endif /* LANEFLOOD_DEBUG */

#define FOOTER1_CRC_OK                0x80
#define FOOTER1_CORRELATION           0x7f
#define RSSI_FIELD             packet[packet_len_tmp - 1]
#define CRC_FIELD              packet[packet_len_tmp]


/* -------------------------- Clock Capture ------------------------- */
/**
 * \brief Capture next low-frequency clock tick and DCO clock value at that instant.
 * \param t_cap_h variable for storing value of DCO clock value
 * \param t_cap_l variable for storing value of low-frequency clock value
 */
#define CAPTURE_NEXT_CLOCK_TICK(t_cap_h, t_cap_l) do {\
		/* Enable capture mode for timers B6 and A2 (ACLK) */\
		TBCCTL6 = CCIS0 | CM_POS | CAP | SCS; \
		TACCTL2 = CCIS0 | CM_POS | CAP | SCS; \
		/* Wait until both timers capture the next clock tick */\
		while (!((TBCCTL6 & CCIFG) && (TACCTL2 & CCIFG))); \
		/* Store the capture timer values */\
		t_cap_h = TBCCR6; \
		t_cap_l = TACCR2; \
		/* Disable capture mode */\
		TBCCTL6 = 0; \
		TACCTL2 = 0; \
} while (0)

/** @} */

/* -------------------------------- SFD ----------------------------- */

/**
 * \defgroup glossy_sfd Management of SFD interrupts
 * @{
 */

/**
 * \brief Capture instants of SFD events on timer B1
 * \param edge Edge used for capture.
 *
 */
#define SFD_CAP_INIT(edge) do {\
	P4SEL |= BV(SFD);\
	TBCCTL1 = edge | CAP | SCS;\
} while (0)

/**
 * \brief Enable generation of interrupts due to SFD events
 */
#define ENABLE_SFD_INT()		do { TBCCTL1 |= CCIE; } while (0)

/**
 * \brief Disable generation of interrupts due to SFD events
 */
#define DISABLE_SFD_INT()		do { TBCCTL1 &= ~CCIE; } while (0)

/**
 * \brief Clear interrupt flag due to SFD events
 */
#define CLEAR_SFD_INT()			do { TBCCTL1 &= ~CCIFG; } while (0)

/**
 * \brief Clear capture overlfow flag due to SFD events
 */
#define CLEAR_COV_INT()                 do { TBCCTL1 &= ~COV; } while (0)

/**
 * \brief Check if generation of interrupts due to SFD events is enabled
 */
#define IS_ENABLED_SFD_INT()    !!(TBCCTL1 & CCIE)


#endif /* GLOSSY_WHISPER_H_ */

/** @} */
