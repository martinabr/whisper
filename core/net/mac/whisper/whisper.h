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

#ifndef WHISPER_H_
#define WHISPER_H_

#include "contiki.h"
#include "dev/watchdog.h"
#include "dev/cc2420_const.h"
#include "dev/cc2420.h"
#include "dev/leds.h"
#include "dev/spi.h"
#include <stdio.h>
#include <legacymsp430.h>
#include <msp430f1611.h>
#include <stdlib.h>

#define ENABLE_COOJA_DEBUG 0
#include "dev/cooja-debug.h"

#include "../gfast.h"
#include "../laneflood/laneflood.h"

#ifndef WHISPER_RADIO_ALWAYS_ON
#define WHISPER_RADIO_ALWAYS_ON 0
#endif

#define WHISPER_RELAY_CNT_LEN          sizeof(uint8_t)
#define WHISPER_DATA_LEN               sizeof(uint8_t)
#define WHISPER_PHY_LEN                (7 * sizeof(uint8_t))
#define WHISPER_LEN_LEN                sizeof(uint8_t)
#define WHISPER_IS_ON()                (get_whisper_state() != WHISPER_STATE_OFF)


#define WHISPER_RSSI_FIELD             whisper_packet_in[whisper_packet_len_tmp - 1]
#define WHISPER_CRC_FIELD              whisper_packet_in[whisper_packet_len_tmp]

#define WHISPER_WAIT_FOR_SFD_DURATION (RTIMER_SECOND / 3400) // 288 us (RTIMER_SECOND / 250) //4ms

#define WHISPER_MAX_SAMPLE_CNT     15

/**
 * List of possible Whisper states.
 */
enum whisper_state {
	WHISPER_STATE_OFF,          /**< Whisper is not executing */
	WHISPER_STATE_WAITING,      /**< Whisper is waiting for a packet being flooded */
	WHISPER_STATE_RECEIVING,    /**< Whisper is receiving a packet */
	WHISPER_STATE_TRANSMITTED,     /**< Whisper has just finished transmitting a packet */
	WHISPER_STATE_TRANSMITTING /**< Whisper is transmitting a packet */
};

PROCESS_NAME(whisper_process);

void whisper_interrupt(void) __attribute__((section(".gfast")));

extern uint8_t whisper_offset;

#if LANEFLOOD_DEBUG
uint16_t whisper_high_T_irq, whisper_rx_timeout, whisper_bad_length, whisper_bad_crc, whisper_bad_T_rx, whisper_cov;
uint16_t whisper_cov_tbccr1, whisper_cov_tbccr2, whisper_cov_tbccr3, whisper_cov_tbccr4, whisper_cov_tbccr5, whisper_cov_tbccr6, whisper_cov_tbccr_;
#endif /* LANEFLOOD_DEBUG */

/* ----------------------- Application interface -------------------- */

/**
 * \brief            		Start Glossy and stall all other application tasks.
 *
 * \param data_      		A pointer to the flooding data.
 *
 *                   		At the initiator, Glossy reads from the given memory
 *                   		location data provided by the application.
 *
 *                   		At a receiver, Glossy writes to the given memory
 *                   		location data for the application.
 * \param data_len_  		Length of the flooding data, in bytes.
 * \param initiator_ 		Not zero if the node is the initiator,
 *                  		zero if it is a receiver.
 * \param tx_max_    		Maximum number of transmissions (N).
 * \param t_stop_    		Time instant at which Glossy must stop, in case it is
 *                   		still running.
 * \param cb_        		Callback function, called when Glossy terminates its
 *                   		execution.
 * \param rtimer_    		First argument of the callback function.
 * \param ptr_       		Second argument of the callback function.
 */
void whisper_start(uint8_t initiator_, uint8_t tx_packet_len, uint8_t tx_max_, 
        rtimer_clock_t t_stop_, rtimer_clock_t t_start_, uint8_t sync_, 
        rtimer_callback_t cb_, struct rtimer *rtimer_, void *ptr_);

uint8_t whisper_create_packet_preamble_len4(void);
uint8_t whisper_create_packet7_preamble_len2(uint8_t packets);
uint8_t whisper_create_packet5_preamble_len2(uint8_t packets);

/**
 * \brief            Stop Glossy and resume all other application tasks.
 * \returns          Number of times the packet has been received during
 *                   last Glossy phase.
 *                   If it is zero, the packet was not successfully received.
 * \sa               get_rx_cnt
 */
uint8_t whisper_stop(void);

/**
 * \brief            Get the last received counter.
 * \returns          Number of times the packet has been received during
 *                   last Glossy phase.
 *                   If it is zero, the packet was not successfully received.
 */
uint8_t get_whisper_rx_cnt(void);

uint8_t get_whisper_cnt(void);

uint8_t get_whisper_tx_cnt(void);

uint8_t get_whisper_packet_len(void);

uint8_t *get_whisper_packet(void);

int8_t get_whisper_rssi(void);

uint8_t get_whisper_crc0(void);
uint8_t get_whisper_crc1(void);

uint8_t get_whisper_state(void);

uint16_t get_whisper_transmitted(void);

uint8_t get_whisper_cca_valid(void);

int8_t get_whisper_cur_countdown(void);

uint16_t get_whisper_T_rx(void);
#if MEASURE_IDLE_LISTENING
uint16_t get_whisper_active(void);
uint16_t get_whisper_passive(void);
#endif /* MEASURE_IDLE_LISTENING */

uint16_t get_whisper_delta_sfd_wait(void);

uint16_t get_whisper_delta_sfd_tx(void);

int16_t get_whisper_t_rx_delta(void);

uint8_t get_whisper_T_irq(void);

uint8_t get_whisper_got_packet(void);

uint8_t get_whisper_nops_to_add(void);
int16_t get_whisper_t_rx_delta_final(void);

uint8_t get_countdown_bak(void);

rtimer_clock_t get_T_ref_l(void);

/* ------------------------------ Timeouts -------------------------- */

inline void whisper_schedule_rx_timeout(void);
inline void whisper_stop_rx_timeout(void);

/* ----------------------- Interrupt functions ---------------------- */

inline void whisper_begin_rx(void);
inline void whisper_end_rx(void);
inline void whisper_begin_tx(void);
inline void whisper_end_tx(void);

inline void radio_abort_rx_whisper(void);

#define BUSYWAIT_UNTIL(cond, max_time)                                  \
do {                                                                    \
    rtimer_clock_t t0 = RTIMER_NOW();                                   \
    while(!(cond) && RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + (max_time)));   \
} while(0)


#endif /* WHISPER_H_ */
