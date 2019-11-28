/*
 * Copyright (c) 2011, ETH Zurich.
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
 * Author: Federico Ferrari <ferrari@tik.ee.ethz.ch>
 *
 */

/**
 * \file
 *         Glossy core, header file.
 * \author
 *         Federico Ferrari <ferrari@tik.ee.ethz.ch>
 */

#ifndef GLOSSY_H_
#define GLOSSY_H_

#include "contiki.h"
#include "dev/watchdog.h"
#include "dev/cc2420_const.h"
#include "dev/leds.h"
#include "dev/spi.h"
#include <stdio.h>
#include <legacymsp430.h>
#include <msp430f1611.h>
#include <stdlib.h>

#define ENABLE_COOJA_DEBUG 0
#include "dev/cooja-debug.h"

#include "../gfast.h"


/**
 * Initiator timeout, in number of Glossy slots.
 * When the timeout expires, if the initiator has not received any packet
 * after its first transmission it transmits again.
 */
#define GLOSSY_INITIATOR_TIMEOUT      3

/**
 * Ratio between the frequencies of the DCO and the low-frequency clocks
 */
#if COOJA
#define CLOCK_PHI                     (4194304uL / RTIMER_SECOND)
#else
#define CLOCK_PHI                     (F_CPU / RTIMER_SECOND)
#endif /* COOJA */

#define GLOSSY_RELAY_CNT_LEN          sizeof(uint8_t)
#define GLOSSY_IS_ON()                (get_state() != GLOSSY_STATE_OFF)
#define FOOTER_LEN                    2


/**
 * List of possible Glossy states.
 */
enum glossy_state {
	GLOSSY_STATE_OFF,          /**< Glossy is not executing */
	GLOSSY_STATE_WAITING,      /**< Glossy is waiting for a packet being flooded */
	GLOSSY_STATE_RECEIVING,    /**< Glossy is receiving a packet */
	GLOSSY_STATE_RECEIVED,     /**< Glossy has just finished receiving a packet */
	GLOSSY_STATE_TRANSMITTING, /**< Glossy is transmitting a packet */
	GLOSSY_STATE_TRANSMITTED,  /**< Glossy has just finished transmitting a packet */
	GLOSSY_STATE_ABORTED       /**< Glossy has just aborted a packet reception */
};

PROCESS_NAME(glossy_process);
void glossy_interrupt(void) __attribute__((section(".gfast")));

/* ----------------------- Application interface -------------------- */
/**
 * \defgroup glossy_interface Glossy API
 * @{
 * \file   glossy.h
 * \file   glossy.c
 */

/**
 * \defgroup glossy_main Interface related to flooding
 * @{
 */

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
void glossy_start(uint8_t *data_, uint8_t *data_len_, uint8_t initiator_, uint8_t sync_, uint8_t tx_max_, rtimer_clock_t t_stop_, rtimer_callback_t cb_,
		struct rtimer *rtimer_, void *ptr_);

/**
 * \brief            Stop Glossy and resume all other application tasks.
 * \returns          Number of times the packet has been received during
 *                   last Glossy phase.
 *                   If it is zero, the packet was not successfully received.
 * \sa               get_rx_cnt
 */
uint8_t glossy_stop(void);

/**
 * \brief            Get the last received counter.
 * \returns          Number of times the packet has been received during
 *                   last Glossy phase.
 *                   If it is zero, the packet was not successfully received.
 */
uint8_t get_rx_cnt(void);

/**
 * \brief            Get the current Glossy state.
 * \return           Current Glossy state, one of the possible values
 *                   of \link glossy_state \endlink.
 */
uint8_t get_state(void);
/**
 * \brief            Get low-frequency time of first packet reception
 *                   during the last Glossy phase.
 * \returns          Low-frequency time of first packet reception
 *                   during the last Glossy phase.
 */
rtimer_clock_t get_t_first_rx_l(void);

/** @} */

/**
 * \defgroup glossy_sync Interface related to time synchronization
 * @{
 */

/**
 * \brief            Get the last relay counter.
 * \returns          Value of the relay counter embedded in the first packet
 *                   received during the last Glossy phase.
 */
uint8_t get_relay_cnt(void);

/**
 * \brief            Get the local estimation of T_slot, in DCO clock ticks.
 * \returns          Local estimation of T_slot.
 */
rtimer_clock_t get_T_slot_h(void);

/**
 * \brief            Get low-frequency synchronization reference time.
 * \returns          Low-frequency reference time
 *                   (i.e., time at which the initiator started the flood).
 */
//rtimer_clock_t get_t_ref_l(void);

/**
 * \brief            Provide information about current synchronization status.
 * \returns          Not zero if the synchronization reference time was
 *                   updated during the last Glossy phase, zero otherwise.
 */
//uint8_t is_t_ref_l_updated(void);

/**
 * \brief            Set low-frequency synchronization reference time.
 * \param t          Updated reference time.
 *                   Useful to manually update the reference time if a
 *                   packet has not been received.
 */
//void set_t_ref_l(rtimer_clock_t t);

/**
 * \brief            Set the current synchronization status.
 * \param updated    Not zero if a node has to be considered synchronized,
 *                   zero otherwise.
 */
//void set_t_ref_l_updated(uint8_t updated);


void glossy_reset(void);

//void reset_t_ref_l_updated(void);


uint8_t get_win_cnt(void);
unsigned long get_T_slot_h_sum(void);
/** @} */

/** @} */

/**
 * \defgroup glossy_internal Glossy internal functions
 * @{
 * \file   glossy.h
 * \file   glossy.c
 */

/* ------------------------------ Timeouts -------------------------- */
/**
 * \defgroup glossy_timeouts Timeouts
 * @{
 */

inline void glossy_schedule_rx_timeout(void);
inline void glossy_stop_rx_timeout(void);
inline void glossy_schedule_initiator_timeout(void);
inline void glossy_stop_initiator_timeout(void);

/** @} */

/* ----------------------- Interrupt functions ---------------------- */
/**
 * \defgroup glossy_interrupts Interrupt functions
 * @{
 */

inline void glossy_begin_rx(void);
inline void glossy_end_rx(void);
inline void glossy_begin_tx(void);
inline void glossy_end_tx(void);

inline void glossy_radio_abort_rx(void);



/** @} */


#endif /* GLOSSY_H_ */

/** @} */
