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

#ifndef CORE_NET_MAC_LANEFLOOD_LANEFLOOD_H_
#define CORE_NET_MAC_LANEFLOOD_LANEFLOOD_H_

#include "../whisper/whisper.h"
#include "contiki.h"


#include "node-id.h"

/**
 * \brief NodeId of the initiator.
 *        Default value: 1
 */
#ifndef INITIATOR_NODE_ID
#define INITIATOR_NODE_ID       1
#endif

#define N_TX	3

/**
 * \brief Period with which a Glossy phase is scheduled.
 *        Default value: 250 ms.
 */

#if !INDRIYA
#define GLOSSY_PERIOD          (RTIMER_SECOND / 10) // 200 ms //(RTIMER_SECOND/6) //
#else
#define GLOSSY_PERIOD          (RTIMER_SECOND / 4) // 250 ms
#endif 

#if !INDRIYA
#define LANEFLOOD_WAKEUP_PERIOD		GLOSSY_PERIOD // 100ms
#else
#define LANEFLOOD_WAKEUP_PERIOD		GLOSSY_PERIOD //(RTIMER_SECOND / 4) // 250 ms
#endif

#define WHISPER_GLOSSY_GUARD          (RTIMER_SECOND / 500) // 2ms

/**
 * \brief Duration of each Glossy phase.
 *        Default value: 20 ms.
 */
#if !INDRIYA
#define GLOSSY_DURATION         (RTIMER_SECOND / 50) // 30.3 ms
#else
#define GLOSSY_DURATION         (RTIMER_SECOND / 50) // 20 ms
#endif

/**
 * \brief Guard-time at receivers.
 *        Default value: 526 us.
 */
#if COOJA
#define GLOSSY_GUARD_TIME       (RTIMER_SECOND / 1000)
#define WHISPER_GUARD_TIME       (RTIMER_SECOND / 6000)   // 526 us
#else
#define GLOSSY_GUARD_TIME       (RTIMER_SECOND / 1900)   // 526 us
#define WHISPER_GUARD_TIME       (RTIMER_SECOND / 6000)   // 526 us
#endif /* COOJA */

/**
 * \brief Period during bootstrapping at receivers.
 *        It should not be an exact fraction of \link GLOSSY_PERIOD \endlink.
 *        Default value: 69.474 ms.
 */
#define GLOSSY_INIT_PERIOD      (GLOSSY_INIT_DURATION + RTIMER_SECOND / 100)                   //  69.474 ms

/**
 * \brief Duration during bootstrapping at receivers.
 *        Default value: 59.474 ms.
 */
#define GLOSSY_INIT_DURATION    (GLOSSY_DURATION - GLOSSY_GUARD_TIME + GLOSSY_INIT_GUARD_TIME) //  59.474 ms

/**
 * \brief Guard-time during bootstrapping at receivers.
 *        Default value: 50 ms.
 */
#define GLOSSY_INIT_GUARD_TIME  (RTIMER_SECOND / 20)                                           //  50 ms

#if INDRIYA || TWIST
#define LF_INIT_SYNC_GUARD		(GLOSSY_BOOTSTRAP_PERIODS + 10)
#else
#define LF_INIT_SYNC_GUARD		5
#endif

#define LF_SYNC_GUARD			1 // TODO: Currently does *not* work with other values

#if WHISPER_PREAMBLE_LENGTH == 4
#define MAX_COUNT 14
#elif WHISPER_PREAMBLE_LENGTH == 2
#if WHISPER_PACKET_LENGTH == 7
#define MAX_COUNT 18
#endif
#endif

#if CC2420_ENABLE_TESTMODE && WHISPER_LONG_WAKEUP_PACKET && WHISPER_PACKET_LENGTH == 9
#define WHISPER_DURATION  (RTIMER_SECOND / 250) // 4 ms
#elif CC2420_ENABLE_TESTMODE && WHISPER_NTX ==5  && WHISPER_PACKET_LENGTH == 9
#define WHISPER_DURATION  (RTIMER_SECOND / 180) // 5.56 ms
#else
#define WHISPER_DURATION  (RTIMER_SECOND / 200) // 5 ms
#endif 

#define WHISPER_PREPARE (RTIMER_SECOND/500) //2
#define WHISPER_RADIO_CRYSTAL_STABLE 8 // 8*30.5us
/**
 * \brief Number of consecutive Glossy phases with successful computation of reference time required to exit from bootstrapping.
 *        Default value: 3.
 */
#define GLOSSY_BOOTSTRAP_PERIODS 5

/**
 * \brief Check if Glossy is still bootstrapping.
 * \sa \link GLOSSY_BOOTSTRAP_PERIODS \endlink.
 */
#define GLOSSY_IS_BOOTSTRAPPING()   (skew_estimated < GLOSSY_BOOTSTRAP_PERIODS)

#define WHISPER_COLLECTION_IS_BOOTSTRAPPING()  (rounds < GLOSSY_BOOTSTRAP_PERIODS)

#define WHISPER_ONE_TO_ONE_IS_BOOTSTRAPPING()  (whisper_avg_counter_len < 2)

enum lf_header_type {
    GLOSSY_HEADER,
    LF_SYNC_HEADER,
    LF_SETUP_HEADER,
    LF_RESPONSE_HEADER,
    LF_DATA_HEADER
};
/**
 * List of possible LF states
 */
enum laneflood_state {
 	LF_STATE_SETUP_SYNC,
 	LF_STATE_RESPONSE,
 	LF_STATE_DATA,
	LF_STATE_WAIT,
 	LF_STATE_SLEEP,
 	LF_STATE_OFF
 };

/**
 * \brief	The common header struct
 *
 * 			Header struct that is used by all packets
 */
typedef struct {
	uint8_t lf_header_field;
	uint16_t lf_seq_no;
	uint8_t lf_next_field[];
} __attribute__((packed)) lf_header_common_struct;

/**
 * \brief	FS data structure to sync to the common heartbeat
 *
 * 			Data struct that is used to represent data during point-to-point communication.
 */
typedef struct {
	uint16_t  lf_seq_no_to_sync;
	uint8_t	  lf_next_field[];
} __attribute__((packed)) lf_sync_struct;

/**
 * \brief Length of data structure.
 */
#define LF_HEADER_COMMON_LEN  sizeof(lf_header_common_struct)
#define LF_GLOSSY_HEADER_LEN  (sizeof(lf_header_glossy_struct) + FOOTER_LEN)
#define LF_SYNC_LEN           sizeof(lf_sync_struct)

/**
 * \brief Check if the nodeId matches the one of the initiator.
 */
#define IS_INITIATOR()              (node_id == INITIATOR_NODE_ID)

// WHISPER_DIRECTION_MODE
#define WHISPER_ANYONE        0
#define WHISPER_DISSEMINATION 1
#define WHISPER_COLLECTION    2
#define WHISPER_ONE_TO_ONE    3

// WHISPER_MANY_SENDERS
#define WHISPER_SINGLE_SENDERS        0
#define WHISPER_CLOSE_SENDERS         1
#define WHISPER_DISTRIBUTED_SENDERS   2

#if WHISPER_DIRECTION_AWARE_SAMPLING

#if WHISPER_DIRECTION_MODE == WHISPER_ANYONE // Anyone can start
    #error [ERROR:] Wrong mode for direction-aware sampling.
#elif WHISPER_DIRECTION_MODE == WHISPER_DISSEMINATION // Dissemination
    #define IS_SENDER()                 (node_id == INITIATOR_NODE_ID)
#elif WHISPER_DIRECTION_MODE == WHISPER_COLLECTION // Collection
    #if WHISPER_MANY_SENDERS == WHISPER_SINGLE_SENDERS     // Changing single sender
        #define IS_SENDER()                 ((node_id == 10 && rounds < 1000) \
                                            || (node_id == 22 && rounds >= 1000 && rounds < 2000) || (node_id == 11 && rounds >= 2000 && rounds < 3000) \
                                            || (node_id == 16 && rounds >= 3000 && rounds < 4000) || (node_id == 23 && rounds >= 4000 && rounds < 5000) \
                                            || (node_id == 19 && rounds >= 5000 && rounds < 6000) || (node_id == 20 && rounds >= 6000 && rounds < 7000) \
                                            || (node_id == 31 && rounds >= 7000 && rounds < 8000) || (node_id == 26 && rounds >= 8000 && rounds < 9000) \
                                            || (node_id == 7 && rounds >= 9000 && rounds < 10000))
//#define IS_SENDER()                 ((node_id == 3 && rounds < 10) \
//                                            || (node_id == 5 && rounds >= 10 && rounds < 20) || (node_id == 2 && rounds >= 20 && rounds < 30) \
//                                            || (node_id == 4 && rounds >= 30 && rounds < 40) || (node_id == 6 && rounds >= 40 && rounds < 50) \
//                                            || (node_id == 3 && rounds >= 50 && rounds < 60) \
//                                            || (node_id == 5 && rounds >= 60 && rounds < 70) || (node_id == 2 && rounds >= 70 && rounds < 80) \
//                                            || (node_id == 4 && rounds >= 80 && rounds < 90) || (node_id == 6 && rounds >= 90 && rounds < 100) \
//                                    )
    #elif WHISPER_MANY_SENDERS == WHISPER_CLOSE_SENDERS   // Close senders
        #define IS_SENDER()                 (node_id == 18 || node_id == 27 || node_id == 24 || node_id == 23)
        //#define IS_SENDER()                 (node_id == 1)
    #elif WHISPER_MANY_SENDERS == WHISPER_DISTRIBUTED_SENDERS  // Distributed senders
        #define IS_SENDER()                 (node_id == 16 || node_id == 19 || node_id == 7 || node_id == 4)
    #endif
#elif WHISPER_DIRECTION_MODE == WHISPER_ONE_TO_ONE // One-to-one
    #define IS_SENDER()                 (rounds % 2 == 0 && node_id == 1 || rounds % 2 == 1 && node_id == 3)
//(rounds % 2 == 0 && node_id == 8 || rounds % 2 == 1 && node_id == 16)
#endif /* WHISPER_DIRECTION_MODE */

#else /* !WHISPER_DIRECTION_AWARE_SAMPLING */

#if WHISPER_DIRECTION_MODE == WHISPER_ANYONE // Anyone can start
    #if WHISPER_MANY_SENDERS == WHISPER_SINGLE_SENDERS     // Changing single sender
        #define IS_SENDER()                 ((node_id == 10 && rounds < 1000) \
                                            || (node_id == 22 && rounds >= 1000 && rounds < 2000) || (node_id == 11 && rounds >= 2000 && rounds < 3000) \
                                            || (node_id == 16 && rounds >= 3000 && rounds < 4000) || (node_id == 23 && rounds >= 4000 && rounds < 5000) \
                                            || (node_id == 19 && rounds >= 5000 && rounds < 6000) || (node_id == 20 && rounds >= 6000 && rounds < 7000) \
                                            || (node_id == 31 && rounds >= 7000 && rounds < 8000) || (node_id == 26 && rounds >= 8000 && rounds < 9000) \
                                            || (node_id == 7 && rounds >= 9000 && rounds < 10000))
    #elif WHISPER_MANY_SENDERS == WHISPER_CLOSE_SENDERS   // Close senders
        #define IS_SENDER()                 (node_id == 4 || node_id == 2 || node_id == 8 || node_id == 1)
    #elif WHISPER_MANY_SENDERS == WHISPER_DISTRIBUTED_SENDERS   // Distributed senders
        #define IS_SENDER()                 (node_id == 16 || node_id == 19 || node_id == 7 || node_id == 1)
    #endif
#elif WHISPER_DIRECTION_MODE == WHISPER_DISSEMINATION // Dissemination
    #define IS_SENDER()                 (node_id == INITIATOR_NODE_ID)
#elif WHISPER_DIRECTION_MODE == WHISPER_COLLECTION // Collection
    #if WHISPER_MANY_SENDERS == 0     // Changing single sender
        #define IS_SENDER()                 ((node_id == 10 && rounds < 1000) \
                                            || (node_id == 22 && rounds >= 1000 && rounds < 2000) || (node_id == 11 && rounds >= 2000 && rounds < 3000) \
                                            || (node_id == 16 && rounds >= 3000 && rounds < 4000) || (node_id == 23 && rounds >= 4000 && rounds < 5000) \
                                            || (node_id == 19 && rounds >= 5000 && rounds < 6000) || (node_id == 20 && rounds >= 6000 && rounds < 7000) \
                                            || (node_id == 31 && rounds >= 7000 && rounds < 8000) || (node_id == 26 && rounds >= 8000 && rounds < 9000) \
                                            || (node_id == 7 && rounds >= 9000 && rounds < 10000))
    #elif WHISPER_MANY_SENDERS == 1   // Close senders
        #define IS_SENDER()                 (node_id == 18 || node_id == 27 || node_id == 24 || node_id == 23)
    #elif WHISPER_MANY_SENDERS == 2   // Distributed senders
        #define IS_SENDER()                 (node_id == 16 || node_id == 19 || node_id == 7 || node_id == 4)
#endif

#endif 

#endif /* WHISPER_DIRECTION_AWARE_SAMPLING */


#define LF_IS_BOOTSTRAPPING()		((signed short)(seq_no_to_sync - cur_seq_no) >= 0)

/**
 * \brief Check if Glossy is synchronized.
 *
 * The application assumes that a node is synchronized if it updated the reference time
 * during the last Glossy phase.
 * \sa \link is_t_ref_l_updated \endlink
 */
#define IS_SYNCED()          (is_t_ref_l_updated())

/**
 * \brief Get Glossy reference time.
 * \sa \link get_t_ref_l \endlink
 */
#define REFERENCE_TIME       (get_t_ref_l())

char glossy_scheduler(struct rtimer *t, void *ptr);
char fs_scheduler(struct rtimer *t, void *ptr);

#if LANEFLOOD_DEBUG
uint16_t bad_header;
#endif /* LANEFLOOD_DEBUG */

unsigned long rounds;

#if WHISPER_DIRECTION_AWARE_SAMPLING
unsigned long whisper_avg_counter[WHISPER_MAX_COUNTER_LEN];
uint8_t whisper_avg_counter_index;        // the index for the current whisper slot
uint8_t whisper_avg_counter_len;          // how many values exist in the counter buffer
uint8_t whisper_avg_start[WHISPER_MAX_COUNTER_LEN];             // the duration to wait in ticks (32kHz)
rtimer_clock_t whisper_avg_start_t[WHISPER_MAX_COUNTER_LEN];    // the actual time to turn the radio on
#endif /* WHISPER_DIRECTION_AWARE_SAMPLING */

PROCESS_NAME(laneflood_process);
#define LANEFLOOD_EPOCH 1

#endif /* CORE_NET_MAC_LANEFLOOD_LANEFLOOD_H_ */
