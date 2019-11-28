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

#include "laneflood.h"

static lf_sync_struct *lf_sync;

static struct rtimer rt;                   /**< \brief Rtimer used to schedule Glossy. */
static struct pt pt;                       /**< \brief Protothread used to schedule Glossy. */
static rtimer_clock_t t_ref_l_old = 0;     /**< \brief Reference time computed from the Glossy
                                                phase before the last one. \sa get_t_ref_l */

static uint8_t sync_missed = 0;            /**< \brief Current number of consecutive phases without
                                                synchronization (reference time not computed). */
static rtimer_clock_t t_start = 0;         /**< \brief Starting time (low-frequency clock)
                                                of the last Glossy phase. */
static rtimer_clock_t t_schedule = 0;
static int16_t period_skew = 0;                /**< \brief Current estimation of clock skew over a period
                                                of length \link GLOSSY_PERIOD \endlink. */

static uint8_t skew_estimated = 0;         /**< \brief Not zero if the clock skew over a period of length
                                                \link GLOSSY_PERIOD \endlink has already been estimated. */

static lf_header_common_struct *lf_header_common;

static uint32_t data_buf[128/sizeof(uint32_t)] = {0};	/**< \brief The packet buffer */

static struct pt lf_pt;

//static unsigned long rounds = 0;
static uint16_t seq_no_to_sync = 0;
static uint16_t cur_seq_no = 0;
static rtimer_clock_t t_heartbeat_start = 0;
static uint8_t data_length;

static rtimer_clock_t t_stop;

// Forwarder selection
static uint8_t lf_state = LF_STATE_OFF;
static uint8_t lf_prev_state = LF_STATE_OFF;

static struct etimer et;

static uint8_t lf_header;
static uint16_t lf_seq_no;

static uint8_t whisper_packet_length;
static int16_t whisper_cur_sync_counter = -1;


PROCESS(glossy_print_stats_process, "Glossy print stats");
PROCESS_THREAD(glossy_print_stats_process, ev, data) {
    unsigned long avg_radio_on = 0;

    PROCESS_BEGIN();

    while (1) {
        // Before we print whisper and glossy statistics
        // we push the data to the application layer
        PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
        if (rounds == 1) {
            avg_radio_on = 0;
            energest_init();
        } else {
            avg_radio_on = (unsigned long) LANEFLOOD_WAKEUP_PERIOD * 1e6 / RTIMER_SECOND *
                    (energest_type_time(ENERGEST_TYPE_LISTEN) + energest_type_time(ENERGEST_TYPE_TRANSMIT)) /
                    (energest_type_time(ENERGEST_TYPE_CPU) + energest_type_time(ENERGEST_TYPE_LPM)) * LANEFLOOD_EPOCH;
        }

        /** print **/
        if (!GLOSSY_IS_BOOTSTRAPPING()) {
            printf("tx %u/%lu, counter: %d, rx: %u, tx: %d, bak: %u\n",
                    get_whisper_transmitted(), rounds, get_whisper_cur_countdown(), get_rx_cnt(), get_whisper_tx_cnt(), get_countdown_bak());
            printf("T_rx: %u, T_irq: %u, nops: %u, T_wait: %u, T_tx: %u\n",
                    get_whisper_T_rx(), get_whisper_T_irq(), get_whisper_nops_to_add(), get_whisper_delta_sfd_wait(), get_whisper_delta_sfd_tx());
#if WHISPER_DIRECTION_AWARE_SAMPLING
            uint8_t i;
            printf("avg_counter: ");
            for (i = 0; i < sizeof (whisper_avg_counter) / sizeof (unsigned long); i++) {
                printf("%lu ", whisper_avg_counter[i]);
            }
            printf("\n");
            printf("whisper_avg_start: ");
            for (i = 0; i < sizeof (whisper_avg_start) / sizeof (uint8_t); i++) {
                printf("%u ", whisper_avg_start[i]);
            }
           printf("\n");
           printf("whisper_avg_counter_len %d\n", whisper_avg_counter_len);
#endif
#if FLOCKLAB || LOCAL_NODES || COOJA || INDRIYA
            // Print information about average radio-on time.
            printf("ra%lu.%03lu ",
                    avg_radio_on / 1000, avg_radio_on % 1000);
            if (get_whisper_cca_valid()) {
#if MEASURE_IDLE_LISTENING
                printf("passive: %lu.%03lu, active: %lu.%03lu\n",
                        (unsigned long) (get_whisper_passive() * 1e6 / 32758) / 1000, (unsigned long) (get_whisper_passive() * 1e6 / 32758) % 1000,
                        (unsigned long) (get_whisper_active() * 1e6 / 32758) / 1000, (unsigned long) (get_whisper_active() * 1e6 / 32758) % 1000);
#endif /* MEASURE_IDLE_LISTENING */
            } 
            printf("\n");
#endif /* FLOCKLAB || LOCAL_NODES */
#if LANEFLOOD_DEBUG
#if FLOCKLAB || LOCAL_NODES || COOJA             
            if (!IS_INITIATOR()) {
                printf("sk: %ld, sync_missed: %u, t_ref_l %u, T_ref_l %u, cur_syn_cnt %d\n",
                        (long) (period_skew * 1e6) / LANEFLOOD_WAKEUP_PERIOD, sync_missed, get_t_ref_l(), get_T_ref_l(), whisper_cur_sync_counter);
                printf("T_slot %d, T_slot_sum %ld, win_cnt %d, relay_cnt %d\n", get_T_slot_h(), get_T_slot_h_sum(), get_win_cnt(), get_relay_cnt());
            }
#endif /* FLOCKLAB || LOCAL_NODES */
            printf("whisper: ir%urt%ubl%ubc%urx%ucv%u\n",
                    whisper_high_T_irq, whisper_rx_timeout, whisper_bad_length, whisper_bad_crc, whisper_bad_T_rx, whisper_cov);

            //                printf("whisper: ir%urt%ubl%ubc%urx%ucv%u(a%ub%uc%ud%ue%uf%ug%u)\n",
            //                        whisper_high_T_irq, whisper_rx_timeout, whisper_bad_length, whisper_bad_crc, whisper_bad_T_rx, whisper_cov,
            //                        whisper_cov_tbccr1, whisper_cov_tbccr2, whisper_cov_tbccr3, whisper_cov_tbccr4, whisper_cov_tbccr5, whisper_cov_tbccr6, whisper_cov_tbccr_);

#if FLOCKLAB || LOCAL_NODES || COOJA
            printf("glossy: ir%urt%ubl%ubc%ubh%u\n",
                    high_T_irq, rx_timeout, bad_length, bad_crc, bad_header);
#endif /* FLOCKLAB || LOCAL_NODES */
#endif /* LANEFLOOD_DEBUG */

        }
    }
    PROCESS_END();
}

void set_lf_state(uint8_t state) {
	lf_prev_state = lf_state;
	lf_state = state;
}
//--------------------------------------------------------------
inline void
estimate_period_skew(void) {
    // Estimate clock skew over a period only if the reference time has been updated.
    if (IS_SYNCED()) {
        // Estimate clock skew based on previous reference time and the Glossy period.
        period_skew = get_t_ref_l() - (t_ref_l_old + (rtimer_clock_t) GLOSSY_PERIOD);
        // Update old reference time with the newer one.
        t_ref_l_old = get_t_ref_l();
        // If Glossy is still bootstrapping, count the number of consecutive updates of the reference time.
        if (GLOSSY_IS_BOOTSTRAPPING()) {
            // Increment number of consecutive updates of the reference time.
            skew_estimated++;
            // Check if Glossy has exited from bootstrapping.
            if (!GLOSSY_IS_BOOTSTRAPPING()) {
                // Glossy has exited from bootstrapping.
                // Initialize Energest values.
                energest_init();
#if LANEFLOOD_DEBUG
                high_T_irq = 0;
                bad_crc = 0;
                bad_length = 0;
#endif /* LANEFLOOD_DEBUG */
                if (IS_INITIATOR()) {
                    // Store the reference seq-no for the heartbeat
                    seq_no_to_sync = lf_seq_no + LF_INIT_SYNC_GUARD + 1;
                }
            }
        }
    }
}

inline void
laneflood_estimate_period_skew(void) {
    // Estimate clock skew over a period only if the reference time has been updated.
    if (IS_SYNCED()) {
        // Estimate clock skew based on previous reference time and the Glossy period. 
        period_skew = (int16_t)(get_t_ref_l() - (t_ref_l_old + (rtimer_clock_t) LANEFLOOD_WAKEUP_PERIOD));
        // Update old reference time with the newer one.
        t_ref_l_old = get_t_ref_l();
    }
}

//--------------------------------------------------------------
void
lf_write_header_common(lf_header_common_struct *lf_header_common_, uint8_t header_, uint16_t seq_no_) {
    lf_header_common_->lf_header_field = header_;
    lf_header_common_->lf_seq_no = seq_no_;
}

void
lf_read_header_common(lf_header_common_struct *lf_header_common_) {
    lf_header = lf_header_common_->lf_header_field;
    lf_seq_no = lf_header_common_->lf_seq_no;
}

void
lf_read_sync(lf_sync_struct *lf_sync_) {
    seq_no_to_sync = lf_sync_->lf_seq_no_to_sync;
}

char laneflood_scheduler(struct rtimer *t, void *ptr) {
    PT_BEGIN(&lf_pt);

    energest_init();
    sync_missed = 0;
    set_t_ref_l(REFERENCE_TIME + GLOSSY_PERIOD);
#if WITH_GLOSSY
    t_ref_l_old += (GLOSSY_PERIOD);
#else
    t_ref_l_old += (GLOSSY_PERIOD + WHISPER_PREPARE);
    t_schedule = REFERENCE_TIME + period_skew - WHISPER_PREPARE + LANEFLOOD_WAKEUP_PERIOD;
#endif

#if LANEFLOOD_DEBUG
    high_T_irq = 0;
    bad_crc = 0;
    bad_length = 0;
#endif /* LANEFLOOD_DEBUG */
                
    while (1) {        
        if (IS_INITIATOR()) {
            rtimer_set(t, RTIMER_TIME(t) + LANEFLOOD_WAKEUP_PERIOD, 1, (rtimer_callback_t) laneflood_scheduler, ptr);
        } else {
#if WITH_GLOSSY
            rtimer_set(t, REFERENCE_TIME + LANEFLOOD_WAKEUP_PERIOD + period_skew - (GLOSSY_GUARD_TIME * (1 + sync_missed)),
                    1, (rtimer_callback_t) laneflood_scheduler, ptr);
#else
            rtimer_set(t, t_schedule, 1, (rtimer_callback_t) laneflood_scheduler, ptr);
#endif      
        }
        PT_YIELD(&lf_pt);
        
        // ----------------------- Synchronize the network ----------------------- //
#if WITH_GLOSSY  
        // Let's run Glossy first
        set_gfast_modus(LANEFLOOD_GLOSSY);
        t_stop = RTIMER_TIME(t) + GLOSSY_DURATION;
        energest_on = 0; // Do not measure energy-consumption during sync

        if (IS_INITIATOR()) {
            lf_write_header_common(lf_header_common, GLOSSY_HEADER, ++lf_seq_no);
            data_length = LF_HEADER_COMMON_LEN;
            glossy_start((uint8_t *) & data_buf[0], &data_length, INITIATOR, 1,
                    N_TX, t_stop, (rtimer_callback_t) laneflood_scheduler, t, ptr);
        } else {
            data_length = 0;
            glossy_start((uint8_t *) & data_buf[0], &data_length, RECEIVER, 1,
                    N_TX, t_stop, (rtimer_callback_t) laneflood_scheduler, t, ptr);
        }
        PT_YIELD(&lf_pt);
        glossy_stop();
#else /* !WITH_GLOSSY */
        set_gfast_modus(LANEFLOOD_WHISPER);
        energest_on = 0; // Do not measure energy-consumption during sync
        t_start = RTIMER_TIME(t) + WHISPER_PREPARE;
        t_stop = t_start + WHISPER_DURATION;
        if (rounds < 10000) {
            if (IS_INITIATOR()) {
                whisper_start(INITIATOR, whisper_packet_length,
                        MAX_COUNT, t_stop, t_start, 1, (rtimer_callback_t) laneflood_scheduler, t, ptr);
            } else {
                // wake up a little bit earlier
                t_start -= (WHISPER_GUARD_TIME * (sync_missed + 1));
                whisper_start(RECEIVER, 0,
                        MAX_COUNT, t_stop, t_start, 1, (rtimer_callback_t) laneflood_scheduler, t, ptr);
            }
        } else {
            // Just checking how bad it can be without sync
            whisper_start(RECEIVER, 0,
                    MAX_COUNT, t_stop, t_start, 0, (rtimer_callback_t) laneflood_scheduler, t, ptr);
        }
        PT_YIELD(&lf_pt);
        whisper_stop();
#endif /*WITH_GLOSSY*/
        
        if (!IS_INITIATOR()) {
            if (!IS_SYNCED()) {
                // The reference time was not updated: increment reference time by GLOSSY_PERIOD.
                set_t_ref_l(REFERENCE_TIME + LANEFLOOD_WAKEUP_PERIOD);
                set_t_ref_l_updated(1);
                // Increment sync_missed.
                sync_missed++;
            } else {
#if !WITH_GLOSSY
                period_skew = (int16_t) (get_t_ref_l() - (t_ref_l_old + LANEFLOOD_WAKEUP_PERIOD)) / (sync_missed + 1);
#endif
                // The reference time was updated: reset sync_missed to zero.
                sync_missed = 0;
            }
#if WITH_GLOSSY
            laneflood_estimate_period_skew();
#else
            t_ref_l_old = get_t_ref_l();
            whisper_cur_sync_counter = get_whisper_cur_countdown();
#endif   
        }

        // Let's start Whisper
        set_gfast_modus(LANEFLOOD_WHISPER);
        energest_on = 1;
#if WITH_GLOSSY        
        t_start = RTIMER_TIME(t) + GLOSSY_DURATION + WHISPER_GLOSSY_GUARD + WHISPER_PREPARE;
        t_stop = t_start + WHISPER_DURATION;
#else
        if (IS_INITIATOR()) {
            t_start = t_stop + WHISPER_GLOSSY_GUARD + WHISPER_PREPARE;
            t_stop = t_start + WHISPER_DURATION;
        } else {
            t_start = REFERENCE_TIME + WHISPER_DURATION + WHISPER_GLOSSY_GUARD + WHISPER_PREPARE - WHISPER_RADIO_CRYSTAL_STABLE;
            t_stop = t_start + WHISPER_DURATION;
            t_schedule = REFERENCE_TIME + period_skew + LANEFLOOD_WAKEUP_PERIOD - (WHISPER_PREPARE + WHISPER_RADIO_CRYSTAL_STABLE);
        }
#endif

#if WHISPER_DIRECTION_MODE == 0 // Anyone can start
        if (rounds < 10000) {
            if (IS_SENDER()) {
                whisper_start(INITIATOR, whisper_packet_length,
                        MAX_COUNT, t_stop, 0, (rtimer_callback_t) laneflood_scheduler, t, ptr);
            } else {         
                whisper_start(RECEIVER, 0,
                        MAX_COUNT, t_stop, 0, (rtimer_callback_t) laneflood_scheduler, t, ptr);
            }
        } else {
            whisper_start(RECEIVER, 0,
                    MAX_COUNT, t_stop, 0, (rtimer_callback_t) laneflood_scheduler, t, ptr);
        }
#elif WHISPER_DIRECTION_MODE == 1 // Dissemination
        if (rounds < 10000) {
            if (IS_INITIATOR()) {
                whisper_start(INITIATOR, whisper_packet_length,
                        MAX_COUNT, t_stop, t_start, 0, (rtimer_callback_t) laneflood_scheduler, t, ptr);
            } else {
                whisper_start(RECEIVER, 0,
                        MAX_COUNT, t_stop, t_start, 0, (rtimer_callback_t) laneflood_scheduler, t, ptr);
            }
        } else {            
            whisper_start(RECEIVER, 0,
                    MAX_COUNT, t_stop, t_start, 0, (rtimer_callback_t) laneflood_scheduler, t, ptr);
        }

#elif WHISPER_DIRECTION_MODE == 2 // Collection
#if WHISPER_DIRECTION_AWARE_SAMPLING
        if (WHISPER_COLLECTION_IS_BOOTSTRAPPING()) {
            if (IS_INITIATOR()) {
                whisper_start(INITIATOR, whisper_packet_length,
                        MAX_COUNT, t_stop, t_start, 0, (rtimer_callback_t) laneflood_scheduler, t, ptr);
            } else {
                whisper_start(RECEIVER, 0,
                        MAX_COUNT, t_stop, t_start, 0, (rtimer_callback_t) laneflood_scheduler, t, ptr);
            }
        } else {
#endif /* WHISPER_DIRECTION_AWARE_SAMPLING  */
            if (rounds < 10000 + GLOSSY_BOOTSTRAP_PERIODS) {
                if (IS_SENDER()) {
                    whisper_start(INITIATOR, whisper_packet_length,
                            MAX_COUNT, t_stop, t_start, 0, (rtimer_callback_t) laneflood_scheduler, t, ptr);
                } else {
                    whisper_start(RECEIVER, 0,
                            MAX_COUNT, t_stop, t_start, 0, (rtimer_callback_t) laneflood_scheduler, t, ptr);
                }
            } else {
                whisper_start(RECEIVER, 0,
                        MAX_COUNT, t_stop, t_start, 0, (rtimer_callback_t) laneflood_scheduler, t, ptr);
            }
#if WHISPER_DIRECTION_AWARE_SAMPLING
        }
#endif /* WHISPER_DIRECTION_AWARE_SAMPLING  */
#elif WHISPER_DIRECTION_MODE == 3 // One-to-one
        if (rounds < 10000) {
            if (IS_SENDER()) {
                whisper_start(INITIATOR, whisper_packet_length,
                        MAX_COUNT, t_stop, 0, (rtimer_callback_t) laneflood_scheduler, t, ptr);
            } else {
#if WHISPER_LONG_WAKEUP_PACKET
            rtimer_clock_t t0 = RTIMER_NOW() + WHISPER_TX_WRITE_DURATION;
            while (RTIMER_CLOCK_LT(RTIMER_NOW(), t0));            
#elif WHISPER_NTX
            rtimer_clock_t t0 = RTIMER_NOW() + WHISPER_TX_WRITE_DURATION_NTX;
            while (RTIMER_CLOCK_LT(RTIMER_NOW(), t0));
#endif
                whisper_start(RECEIVER, 0,
                        MAX_COUNT, t_stop, 0, (rtimer_callback_t) laneflood_scheduler, t, ptr);
            }
        } else {            
            whisper_start(RECEIVER, 0,
                    MAX_COUNT, t_stop, 0, (rtimer_callback_t) laneflood_scheduler, t, ptr);
        }
#endif
        PT_YIELD(&lf_pt);
        whisper_stop();
        
#if WHISPER_DIRECTION_AWARE_SAMPLING
        // Cluster the current counter in the counter buffer
        uint8_t updated = 0;
        uint8_t whisper_countdown_tmp = 0;
        int8_t whisper_countdown_old = -1;
               
        do {
            if ((get_whisper_cur_countdown() >= 0) && (get_whisper_cur_countdown() <= MAX_COUNT)) {
#if WHISPER_DIRECTION_MODE == 1 || WHISPER_DIRECTION_MODE == 3 // Dissemination & One-to-one
                if (whisper_countdown_old < 0) {
                    whisper_countdown_tmp = (uint8_t) get_whisper_cur_countdown();
                } else {
                    whisper_countdown_tmp = (uint8_t) whisper_countdown_old;
                }
#elif WHISPER_DIRECTION_MODE == 2 // Collection
                if (WHISPER_COLLECTION_IS_BOOTSTRAPPING()) {
                    if (whisper_countdown_old < 0) {
                        if (IS_INITIATOR()) {
#if FLOCKLAB || COOJA
#if CC2420_DEFAULT_TX_POWER == 31                                
                            whisper_countdown_tmp = (uint8_t) (5);
#elif CC2420_DEFAULT_TX_POWER == 11
                            //whisper_countdown_tmp = (uint8_t) (11);
                            whisper_countdown_tmp = (uint8_t) (12);
#else 
#error [ERROR]: Need TX power for collection.
#endif 
#else
#error [ERROR]: Need Testbed for collection.
#endif
                        } else {
#if CC2420_DEFAULT_TX_POWER == 31 && WHISPER_NTX == 3
                            // Network diameter is 3 to 4 hops, thus 
                            // p^diss_slot = 2 * 4 + 3 - 1 = 10
                            // p^coll_slot = 6 + 2 = 12
                            // p_revers = 12 - (p + 3 + 3) = 6 - p
                            if (get_whisper_cur_countdown() > 6) {
                                whisper_countdown_tmp = (uint8_t) 0;
                            } else {
                                whisper_countdown_tmp = (uint8_t) (6 - (get_whisper_cur_countdown()));
                            }
                            
#elif CC2420_DEFAULT_TX_POWER == 11 && WHISPER_NTX == 3
                            // Network diameter is 5 to 6 hops, thus 
                            // p^diss_slot = 2 * 6 + 3 - 1 = 14
                            // p^coll_slot = 6 + 2 = 16
                            // p_revers = 16 - (p + 3 + 3) = 10 - p
                            if (get_whisper_cur_countdown() > 10) {
                                whisper_countdown_tmp = (uint8_t) 0;
                            } else {
                                whisper_countdown_tmp = (uint8_t) (10 - (get_whisper_cur_countdown()));
                            }
#endif 
                        }
                    } else {
                        whisper_countdown_tmp = (uint8_t) (whisper_countdown_old);
                    }     
                } else {
                    if (whisper_countdown_old < 0) {
                        whisper_countdown_tmp = (uint8_t) get_whisper_cur_countdown();
                    } else {
                        whisper_countdown_tmp = (uint8_t) whisper_countdown_old;
                    }     
                }      
#endif           
                whisper_countdown_old = -1;
                updated = 0;  
                if (whisper_avg_counter_len == 0) {
                    whisper_avg_counter[0] = whisper_countdown_tmp * 1e6;
                    whisper_avg_start[0] = WHISPER_PACKET_LENGTH * (whisper_countdown_tmp);
                    whisper_avg_counter_len++;
                    updated = 1;
                } else {
                    uint8_t i;            
                if (whisper_avg_counter[0] < (whisper_countdown_tmp * 1e6)) {
                        for (i = 1; i < whisper_avg_counter_len; i++) {
                            if (whisper_avg_counter_len < (WHISPER_MAX_COUNTER_LEN - 1)
                                    && whisper_avg_counter[i] >= (2 + whisper_countdown_tmp) * 1e6) {
                                // no need to avg, we still have space at the end
                                whisper_countdown_old = (whisper_avg_counter[i] / 1e6);
                                whisper_avg_counter[i] = whisper_countdown_tmp * 1e6;
                                whisper_avg_start[i] = WHISPER_PACKET_LENGTH * (whisper_countdown_tmp);
                                updated = 1;
                                break;
                            } else {
                                if (whisper_avg_counter[i] + (2 * 1e6) >= (whisper_countdown_tmp * 1e6)) {
                                    whisper_avg_counter[i] = (whisper_avg_counter[i] + (whisper_countdown_tmp * 1e6)) / 2;
                                    // Calculate ticks (32kHz-crystal) that have to elapse before transmission start (how many ticks the nodes wait and save energy) 
                                    //whisper_avg_start[i] = (WHISPER_PACKET_LENGTH * 32 * whisper_avg_counter[i] * RTIMER_SECOND) / 1e12;
                                    whisper_avg_start[i] = WHISPER_PACKET_LENGTH * ((whisper_avg_counter[i] / 1e6));
                                    updated = 1;
                                    break;
                                }
                            }
                        }
                    } else {
                        // Ensure that the first value is minimal so that we wake up as early as possible
                        if (whisper_avg_counter[0] != (whisper_countdown_tmp * 1e6)) {
                            leds_on(LEDS_RED);
                            whisper_countdown_old = (whisper_avg_counter[0] / 1e6);
                            leds_off(LEDS_RED);
                        }
                        whisper_avg_counter[0] = whisper_countdown_tmp * 1e6;
                        whisper_avg_start[0] = WHISPER_PACKET_LENGTH * (whisper_countdown_tmp);
                        updated = 1;
                    }
                }
            if (!updated) {
                if (whisper_avg_counter_len < WHISPER_MAX_COUNTER_LEN) {
                    // whisper_avg_counter_len == 1, 2, 3, 4
                    whisper_avg_counter[whisper_avg_counter_len] = whisper_countdown_tmp * 1e6;
                    whisper_avg_start[whisper_avg_counter_len] = WHISPER_PACKET_LENGTH * ((whisper_avg_counter[whisper_avg_counter_len] / 1e6));
                    whisper_avg_counter_len++;
                } else {
                    if (whisper_avg_counter_len == WHISPER_MAX_COUNTER_LEN) {
                        // whisper_avg_counter_len == 5
                        whisper_avg_counter[whisper_avg_counter_len - 1] = (whisper_avg_counter[whisper_avg_counter_len - 1] + (whisper_countdown_tmp * 1e6)) / 2;
                    }
                }
            }
        }
                    
        } while(whisper_countdown_old != -1);
#if WHISPER_PREAMBLE_LENGTH == 4
        whisper_offset = (9 * (whisper_avg_counter[0] / 1e6));
#elif WHISPER_PREAMBLE_LENGTH == 2
#if WHISPER_PACKET_LENGTH == 7
        // Send as the node where in the next hop
        // TODO: Also do for other NTX and preamble length
        whisper_offset = (7 * ((whisper_avg_counter[0] / 1e6) + 2));              
#endif
#endif
#endif
        process_poll(&glossy_print_stats_process);
        rounds++;
    }
    PT_END(&lf_pt);
}

char glossy_scheduler(struct rtimer *t, void *ptr) {
    // This is for Bootstrapping
    PT_BEGIN(&pt);
    // Set LaneFlood state to OFF
    set_lf_state(LF_STATE_OFF);

    while (lf_state == LF_STATE_OFF) {
        if (IS_INITIATOR()) { // Glossy initiator.
            rtimer_clock_t t_stop;
            if (GLOSSY_IS_BOOTSTRAPPING()) {
                // Nodes need to be synchronised and bootstrapped,
                // thus Glossy must have been running at least GLOSSY_BOOTSTRAP_PERIODS
                lf_write_header_common(lf_header_common, GLOSSY_HEADER, ++lf_seq_no);
                data_length = LF_HEADER_COMMON_LEN;
            } else {
                // We are done with the initial boostrapping
                // Set the the LF_SYNC_HEADER so the nodes can learn and calculate the first heartbeat
                lf_write_header_common(lf_header_common, LF_SYNC_HEADER, ++lf_seq_no);
                lf_sync = (lf_sync_struct *) lf_header_common->lf_next_field;
                lf_sync->lf_seq_no_to_sync = seq_no_to_sync;
                data_length = LF_HEADER_COMMON_LEN + LF_SYNC_LEN;
            }
            // Schedule end of Glossy phase based on GLOSSY_DURATION.
            t_stop = RTIMER_TIME(t) + GLOSSY_DURATION;
            // Glossy phase.
            glossy_start((uint8_t *) & data_buf[0], &data_length, INITIATOR, 1, N_TX, t_stop,
                    (rtimer_callback_t) glossy_scheduler, t, ptr);
            // Store time at which Glossy has started.
            t_start = RTIMER_TIME(t);
            // Yield the protothread. It will be resumed when Glossy terminates.
            PT_YIELD(&pt);

            // Stop Glossy.
            glossy_stop();
            // Read the common header
            lf_read_header_common(lf_header_common);
            // Increase the current sequence number otherwise the initiator schedules one phase too much
            cur_seq_no = lf_seq_no + 1;
            if (!GLOSSY_IS_BOOTSTRAPPING()) {
                // Glossy has already successfully bootstrapped.
                if (!IS_SYNCED()) {
                    // The reference time was not updated: increment reference time by GLOSSY_PERIOD.
                    set_t_ref_l(REFERENCE_TIME + GLOSSY_PERIOD);
                    set_t_ref_l_updated(1);
                }
            }
            if (GLOSSY_IS_BOOTSTRAPPING() || LF_IS_BOOTSTRAPPING()) {
                // Schedule begin of next Glossy phase based on GLOSSY_PERIOD.
                rtimer_set(t, t_start + GLOSSY_PERIOD, 1, (rtimer_callback_t) glossy_scheduler, ptr);
                // Estimate the clock skew over the last period.
                estimate_period_skew();
                process_poll(&glossy_print_stats_process);
                // Reset the data buffer
                memset(data_buf, 0, sizeof (data_buf));
                // Yield the protothread.
                PT_YIELD(&pt);
            } else {
                // It's time to schedule the heartbeat
                t_heartbeat_start = t_start + GLOSSY_PERIOD;
                // Schedule the heartbeat
                rtimer_set(t, t_heartbeat_start, 1, (rtimer_callback_t) laneflood_scheduler, NULL);
                // Estimate the clock skew over the last period.
                estimate_period_skew();
                // Set the forwarder selection state
                set_lf_state(LF_STATE_SETUP_SYNC);
                // Poll the process that prints statistics (will be activated later by Contiki).
                process_poll(&glossy_print_stats_process);
            }
        } else { // Glossy receiver.
            // Glossy phase.
            rtimer_clock_t t_stop;
            if (GLOSSY_IS_BOOTSTRAPPING()) {
                // Glossy is still bootstrapping:
                // Schedule end of Glossy phase based on GLOSSY_INIT_DURATION.
                t_stop = RTIMER_TIME(t) + GLOSSY_INIT_DURATION;
            } else {
                // Glossy has already successfully bootstrapped:
                // Schedule end of Glossy phase based on GLOSSY_DURATION.
                t_stop = RTIMER_TIME(t) + GLOSSY_DURATION;
            }
            data_length = 0;
            // Start Glossy.
            glossy_start((uint8_t *) &data_buf[0], &data_length, RECEIVER, 1,
                    N_TX, t_stop, (rtimer_callback_t) glossy_scheduler, t, ptr);
            // Yield the protothread. It will be resumed when Glossy terminates.
            PT_YIELD(&pt);

            // Stop Glossy.
            glossy_stop();
            if (get_rx_cnt()) {
                // Read the common header
                lf_read_header_common(lf_header_common);
                if (lf_header == LF_SYNC_HEADER) {
                    cur_seq_no = lf_seq_no;
                }
            }
            if (GLOSSY_IS_BOOTSTRAPPING()) {
                // Glossy is still bootstrapping.
                if (!IS_SYNCED()) {
                    // The reference time was not updated: reset skew_estimated to zero.
                    skew_estimated = 0;
                }
            } else {
                // Glossy has already successfully bootstrapped.
                if (!IS_SYNCED()) {
                    // The reference time was not updated:
                    // increment reference time by GLOSSY_PERIOD + period_skew.
                    set_t_ref_l(REFERENCE_TIME + GLOSSY_PERIOD + period_skew);
                    set_t_ref_l_updated(1);
                    // Increment sync_missed.
                    sync_missed++;
                } else {
                    // The reference time was not updated: reset sync_missed to zero.
                    sync_missed = 0;
                }
            }
            // Estimate the clock skew over the last period.
            estimate_period_skew();
            // TODO: This is ugly
            uint8_t msg_to_receive = 10;
            if (get_rx_cnt()) {
                if ((lf_header == LF_SYNC_HEADER) && (data_length == (LF_HEADER_COMMON_LEN + LF_SYNC_LEN))) {
                    lf_sync = (lf_sync_struct *) lf_header_common->lf_next_field;
                    lf_read_sync(lf_sync);
                    // SYNC msg received, calulate/update the start of the heartbeat
                    // Calulcate the amount of message exchanges left
                    msg_to_receive = seq_no_to_sync - lf_seq_no;
                    if (msg_to_receive) {
                        t_heartbeat_start = REFERENCE_TIME + ((msg_to_receive + 1) * GLOSSY_PERIOD);
                    } else {
                        t_heartbeat_start = REFERENCE_TIME + GLOSSY_PERIOD;
                    }
                }
            } else {
                if (!GLOSSY_IS_BOOTSTRAPPING() && !seq_no_to_sync) {
                    // Reference sequence number not set, yet.
                    // We do not wait explicitly for a message from the receiver.
                    // In case a node does not receive anything,
                    // it is still able to calculate the first heartbeat
                    // We have not received a message
                    // Just in case, schedule the heartbeat
                    t_heartbeat_start = REFERENCE_TIME + ((LF_INIT_SYNC_GUARD + 1) * GLOSSY_PERIOD) - period_skew - GLOSSY_GUARD_TIME * (1 + sync_missed);
                } else {
                    msg_to_receive = seq_no_to_sync - lf_seq_no - sync_missed;
                }
            }
            if (!t_heartbeat_start || msg_to_receive) {
                // t_heartbeat_start is not set now or
                // we can savely schedule at least one more Glossy wave
                if (GLOSSY_IS_BOOTSTRAPPING()) {
                    // Glossy is still bootstrapping.
                    if (skew_estimated == 0) {
                        // The reference time was not updated:
                        // Schedule begin of next Glossy phase based on last begin and GLOSSY_INIT_PERIOD.
                        rtimer_set(t, RTIMER_TIME(t) + GLOSSY_INIT_PERIOD, 1,
                                (rtimer_callback_t) glossy_scheduler, ptr);
                    } else {
                        // The reference time was updated:
                        // Schedule begin of next Glossy phase based on reference time and GLOSSY_INIT_PERIOD.
                        rtimer_clock_t time  = REFERENCE_TIME + GLOSSY_PERIOD - GLOSSY_INIT_GUARD_TIME;
                        if RTIMER_CLOCK_LT(RTIMER_NOW(), time) {
                            rtimer_set(t, REFERENCE_TIME + GLOSSY_PERIOD - GLOSSY_INIT_GUARD_TIME,
                                    1, (rtimer_callback_t) glossy_scheduler, ptr);
                        } else {
                            // We missed the current Glossy phase. 
                            // We schedule the timer for the next period.
                            rtimer_set(t, REFERENCE_TIME + GLOSSY_PERIOD - GLOSSY_INIT_GUARD_TIME + GLOSSY_PERIOD,
                                    1, (rtimer_callback_t) glossy_scheduler, ptr);
                        }
                    }
                } else {
                    // Glossy has already successfully bootstrapped:
                    // Schedule begin of next Glossy phase based on reference time and GLOSSY_PERIOD.
                    rtimer_set(t, REFERENCE_TIME + GLOSSY_PERIOD + period_skew - (GLOSSY_GUARD_TIME * (1 + sync_missed)),
                            1, (rtimer_callback_t) glossy_scheduler, ptr);
                }
                // Poll the process that prints statistics (will be activated later by Contiki).
                process_poll(&glossy_print_stats_process);
                // Reset the data buffer
                memset(data_buf, 0, sizeof (data_buf));
                // Yield the protothread.
                PT_YIELD(&pt);
            } else {
                // We are close to the heartbeat, lets schedule it
                rtimer_set(t, t_heartbeat_start, 1, (rtimer_callback_t) laneflood_scheduler, NULL);
                // Set the forwarder selection state
                set_lf_state(LF_STATE_SETUP_SYNC);
                // Poll the process that prints statistics (will be activated later by Contiki).
                process_poll(&glossy_print_stats_process);
            }
        }
    }
    PT_END(&pt);
}

/**
 * \defgroup glossy-test-scheduler Periodic scheduling
 * @{
 */
PROCESS(laneflood_process, "LaneFlood Process");
PROCESS_THREAD(laneflood_process, ev, data)
{
	PROCESS_BEGIN();

	printf("LaneFlood process started \n");

	// Initialise the common header
	lf_header_common = (lf_header_common_struct *) &data_buf[0];
	// Reset the data buffer
	memset(data_buf, 0, sizeof(data_buf));
	// Initialize the sequence number.
	lf_seq_no = 0;
    rounds = 0;
	// Start print stats processes.
	process_start(&glossy_print_stats_process, NULL);
	// Start Glossy busy-waiting process.
	process_start(&glossy_process, NULL);
	process_start(&whisper_process, NULL);
#if WHISPER_PREAMBLE_LENGTH == 4
        // The length field in the packet does not include itself, only the MPDU
        whisper_packet_length = whisper_create_packet_preamble_len4();
#elif WHISPER_PREAMBLE_LENGTH == 2
#if WHISPER_PACKET_LENGTH == 7
        whisper_packet_length = whisper_create_packet7_preamble_len2(14);
#elif WHISPER_PACKET_LENGTH == 5
        whisper_packet_len = whisper_create_packet5_preamble_len2(14);
#else
        error [ERROR:] Strange packet length
#endif
#else
        error [ERROR:] Strange preamble length
#endif
	// Wait for some time to ensure that all nodes are awake
	if (IS_INITIATOR()) {
		// Initiator waits for all other nodes to boot up.
#if COOJA || LOCAL_NODES
		etimer_set(&et, 4 * CLOCK_SECOND);
#elif FLOCKLAB
                etimer_set(&et, 30 * CLOCK_SECOND);
#else
                etimer_set(&et, 240 * CLOCK_SECOND);
#endif /* COOJA */
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
	}
    energest_on = 0;
    set_gfast_modus(LANEFLOOD_GLOSSY);
	// Start Glossy experiment in one second.
	rtimer_set(&rt, RTIMER_NOW() + RTIMER_SECOND, 1, (rtimer_callback_t)glossy_scheduler, NULL);

	PROCESS_END();
}
