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

#include "whisper.h"

#include "lib/crc.h"
#include "msp430contiki.h"


static uint8_t whisper_initiator, whisper_tx_cnt, whisper_tx_max = 0;
static uint32_t whisper_packet_in_aligned[128 / 4];
static uint8_t *whisper_packet_in;
#if CC2420_ENABLE_TESTMODE && !WHISPER_LONG_WAKEUP_PACKET
static uint32_t whisper_packet_aligned[140 / 4];
#else 
static uint32_t whisper_packet_aligned[128 / 4];
#endif
static uint8_t *whisper_packet;
static uint8_t whisper_packet_len, whisper_packet_len_tmp;
static uint8_t whisper_bytes_read, whisper_bytes_wrote;
static volatile uint8_t whisper_state;
static rtimer_clock_t whisper_t_rx_start, whisper_t_rx_stop, whisper_t_tx_start, whisper_t_tx_stop;
static rtimer_clock_t whisper_t_rx_timeout;
static rtimer_clock_t whisper_T_irq;
static rtimer_clock_t whisper_t_stop, whisper_t_start;
static rtimer_callback_t whisper_cb;
static struct rtimer *whisper_rtimer;
static void *whisper_ptr;
static unsigned short whisper_ie1, whisper_ie2, whisper_p1ie, whisper_p2ie, whisper_tbiv;

static uint8_t whisper_cca_valid = 0;
static uint16_t whisper_transmitted = 0;
static int8_t whisper_cur_countdown;
static uint8_t countdown_bak = 0;
static volatile uint8_t whisper_got_packet;
static volatile rtimer_clock_t whisper_t_cancle;

static uint8_t whisper_sync_win_cnt = 0;
static unsigned long whisper_sync_counter_sum = 0;
static uint8_t whisper_sync_counter = 0;

static rtimer_clock_t whisper_t_cap_h, whisper_t_cap_l, T_ref_l, T_ref_to_tx_h;
uint32_t packlet_sync_to_ticks;
uint16_t packlets_to_ref;
uint16_t sync_header;
static uint8_t whisper_sync;
static struct rtimer *whisper_rtimer;

uint8_t whisper_offset = 0;

#if MEASURE_IDLE_LISTENING
static uint16_t whisper_t_radio_start_l, whisper_t_rx_start_l, whisper_t_radio_stop_l;
#endif /* MEASURE_IDLE_LISTENING */

static uint8_t whisper_nops_to_add = 0;

#if WHISPER_CLOCKDRIFT_COMP
#if WHISPER_PREAMBLE_LENGTH == 4

#define whisper_clock_drift_values 39
#define whisper_clock_drift_offset 510

static const uint8_t whisper_clock_drift_table[whisper_clock_drift_values] = {
    56, 56, // 510, 511, 
    54,     // 512, 
    52,     // 513,
    50, 50, // 514, 515,
    48,     // 516, 
    46,     // 517,
    44, 44, // 518, 519
    42,     // 520
    40,     // 521
    38, 38, // 522, 523,
    36,     // 524,
    34,     // 525,
    32, 32, // 526, 527,
    30,     // 528,
    28,     // 529
    26, 26, // 530, 531
    24,     // 532,
    22,     // 533,
    20, 20, // 534, 535,
    18,     // 536,
    16,     // 537
    14,     // 538
    12, 12, // 539, 540,
    10,     // 541,
    8,      // 542
    6, 6,   // 543, 544,
    4,      // 545
    2,      // 546
    0, 0    // 547, 548
};

#elif WHISPER_PREAMBLE_LENGTH == 2

#define whisper_clock_drift_values 38
#define whisper_clock_drift_offset 511

static const uint8_t whisper_clock_drift_table[whisper_clock_drift_values] = {
    18, 18, 18, 18, // 511, 512, 513, 514
    16, 16, 16, 16, // 515, 516, 517, 518
    14, 14, 14, 14, // 519, 520, 521, 522
    12, 12, 12, 12, // 523, 524, 525, 526
    10, 10, 10, 10, // 527, 528, 529, 530
    8, 8, 8, 8,     // 531, 532, 533, 534
    6, 6, 6, 6, 6,  // 535, 536, 537, 538, 539
    4, 4, 4, 4,     // 540, 541, 542, 543
    2, 2, 2, 2,     // 544, 545, 546, 547
    0//, 0, 0, 0    // 548, 561, 562, 563,
};

#endif /* WHISPER_PREAMBLE_LENGTH */
#endif /* WHISPER_CLOCKDRIFT_COMP */

// -------------------- Whisper radio functions -------------------- //

static inline __attribute__ ((always_inline)) uint8_t 
radio_status(void) {
    uint8_t status;
    SPI_ENABLE();
    SPI_TXBUF = CC2420_SNOP;
    SPI_WAITFOREOTx();
    status = SPI_RXBUF;
    SPI_DISABLE();
    return status;
}

static inline void 
radio_flush_rx(void) {
    uint8_t dummy;
    DISABLE_SFD_INT();
    SPI_ENABLE();
    FASTSPI_RX_ADDR(CC2420_RXFIFO);
    (void)SPI_RXBUF;
    FASTSPI_RX(dummy);
    asm("mov #1, r15");
    asm("add #-1, r15");
    asm("jnz $-2");
    SPI_DISABLE();
    SPI_ENABLE();
    FASTSPI_TX_ADDR(CC2420_SFLUSHRX);
    SPI_DISABLE();
    SPI_ENABLE();
    FASTSPI_TX_ADDR(CC2420_SFLUSHRX);
    SPI_DISABLE();
    CLEAR_SFD_INT();
    CLEAR_COV_INT();
    ENABLE_SFD_INT();   
}

static inline void 
whisper_radio_write_tx(uint8_t offset) {
    FASTSPI_WRITE_FIFO(whisper_packet + offset, whisper_packet_len_tmp - 1);
}

static inline void radio_flush_tx(void) {
    FASTSPI_STROBE(CC2420_SFLUSHTX);
}

static inline __attribute__ ((always_inline)) void 
radio_on(void) {
    // 23.87 us until crystal oscillator stable
    // 28.71 us including calling this function
    
    // To captue a packet successfully, it requires from here 
    // to the successfully decoded SFD 192 us.
    //SET_PIN_GIO3;
    SPI_ENABLE();
    FASTSPI_TX_ADDR(CC2420_SRXON);
    SPI_DISABLE();
    //UNSET_PIN_GIO3;
    while (!(radio_status() & (BV(CC2420_XOSC16M_STABLE))));    
    if (energest_on) {
        ENERGEST_ON(ENERGEST_TYPE_LISTEN);
    }
}

static inline __attribute__ ((always_inline)) void 
radio_off(void) {
    // 22.46 us including calling this function
#if ENERGEST_CONF_ON
    if (energest_on) {
        if (energest_current_mode[ENERGEST_TYPE_TRANSMIT]) {
            ENERGEST_OFF(ENERGEST_TYPE_TRANSMIT);
        }
        if (energest_current_mode[ENERGEST_TYPE_LISTEN]) {
            ENERGEST_OFF(ENERGEST_TYPE_LISTEN);
        }
    }
#endif /* ENERGEST_CONF_ON */
    // 12.12 us
    //SET_PIN_GIO3;
    SPI_ENABLE();
    FASTSPI_TX_ADDR(CC2420_SRFOFF);
    SPI_DISABLE();
    //UNSET_PIN_GIO3;
}

static inline void 
radio_abort_tx(void) {
    FASTSPI_STROBE(CC2420_SRXON);
#if ENERGEST_CONF_ON
    if (energest_on) {
        if (energest_current_mode[ENERGEST_TYPE_TRANSMIT]) {
            ENERGEST_OFF(ENERGEST_TYPE_TRANSMIT);
            ENERGEST_ON(ENERGEST_TYPE_LISTEN);
        }
    }
#endif /* ENERGEST_CONF_ON */
    radio_flush_rx();
    whisper_got_packet = 1;
}

static inline __attribute__ ((always_inline)) void 
whisper_radio_start_tx(void) {
    SPI_ENABLE();
    FASTSPI_TX_ADDR(CC2420_STXON);
    SPI_DISABLE();
#if ENERGEST_CONF_ON
    if (energest_on) {
        ENERGEST_OFF(ENERGEST_TYPE_LISTEN);
        ENERGEST_ON(ENERGEST_TYPE_TRANSMIT);
    }
#endif /* ENERGEST_CONF_ON */
}

inline void 
radio_abort_rx_whisper(void) {
    whisper_got_packet = 0;
    whisper_state = WHISPER_STATE_WAITING;
    radio_flush_rx();
}

static inline __attribute__ ((always_inline)) void 
whisper_set_tx_mode(uint8_t mode) {
	uint16_t reg;
        SPI_ENABLE();
        FASTSPI_RX_ADDR(CC2420_MDMCTRL1);
        reg = (u8_t)SPI_RXBUF;
	FASTSPI_RX_WORD(reg);
	reg = (reg & 0xfff3) | ((mode & 0x3) << 2);
	FASTSPI_TX_ADDR(CC2420_MDMCTRL1);
	FASTSPI_TX((u8_t) ((reg) >> 8));
	FASTSPI_TX((u8_t) (reg));
        SPI_WAITFORTx_ENDED();
	SPI_DISABLE();
}

static inline __attribute__ ((always_inline)) void 
whisper_set_tx_mode_looping(void) {
	uint16_t reg;
        SPI_ENABLE();
        FASTSPI_RX_ADDR(CC2420_MDMCTRL1);
        reg = (u8_t)SPI_RXBUF;
	FASTSPI_RX_WORD(reg);
	reg = (reg & 0xfff3) | (0x8);
	FASTSPI_TX_ADDR(CC2420_MDMCTRL1);
	FASTSPI_TX((u8_t) ((reg) >> 8));
	FASTSPI_TX((u8_t) (reg));
        SPI_WAITFORTx_ENDED();
	SPI_DISABLE();
}

static inline __attribute__ ((always_inline)) void
whisper_set_tx_power(uint8_t power) {
    uint16_t reg;
    FASTSPI_GETREG(CC2420_TXCTRL, reg);
    reg = (reg & 0xffe0) | (power & 0x1f);
    FASTSPI_SETREG(CC2420_TXCTRL, reg);
}

static inline void  
whisper_compute_sync_reference_time(void) {
    rtimer_clock_t T_tx_to_cap_h = RTIMER_CLOCK_DIFF(whisper_t_cap_h, whisper_t_tx_start);
#if CC2420_ENABLE_TESTMODE && !COOJA
    T_ref_to_tx_h = (((whisper_cur_countdown + 2) * WHISPER_PACKET_LENGTH * 32768) / 31250);
#else 
    T_ref_to_tx_h = (uint16_t)((uint32_t)(32768 * (((whisper_cur_countdown + 2) * WHISPER_PACKET_LENGTH) + (WHISPER_PREAMBLE_LENGTH + 1)))) / 31250;
#endif 
    rtimer_clock_t T_tx_to_cap_l = 1 + T_tx_to_cap_h / CLOCK_PHI;
    T_ref_l =  T_tx_to_cap_l + T_ref_to_tx_h;
    t_ref_l = whisper_t_cap_l - T_ref_l;
    t_ref_l_updated = 1;
}

// -------------------- Glossy SFD interrupt functions -------------------- //

void
whisper_interrupt(void) {
    if ((whisper_state == WHISPER_STATE_RECEIVING) && !SFD_IS_1) {
        rtimer_clock_t tbccr1 = TBCCR1;
        whisper_T_irq = ((RTIMER_NOW_DCO() - tbccr1) - 45) << 1;
        if (whisper_T_irq < 8) {

#if WHISPER_PREAMBLE_LENGTH == 4

#if WHISPER_CLOCKDRIFT_COMP && !COOJA
            rtimer_clock_t whisper_T_rx = (tbccr1 - whisper_t_rx_start) - whisper_clock_drift_offset;
            if (whisper_T_rx < whisper_clock_drift_values) {
                whisper_nops_to_add = (whisper_clock_drift_table[whisper_T_rx]) + whisper_T_irq;
                asm volatile("add %[d], r0" : : [d] "m" (whisper_nops_to_add));
                // Interrupt
                asm volatile("nop"); // irq_delay = 0   //1
                asm volatile("nop"); // irq_delay = 2   //2
                asm volatile("nop"); // irq_delay = 4   //3
                asm volatile("nop"); // irq_delay = 6   //4
                asm volatile("nop"); // irq_delay = 8   //5
                asm volatile("nop"); // irq_delay = 10  //6
                // Clock drift compensation
                asm volatile("nop"); // irq_delay = 12  //7
                asm volatile("nop"); // irq_delay = 14  //8
                asm volatile("nop"); // irq_delay = 16  //9
                asm volatile("nop"); // irq_delay = 18  //10
                asm volatile("nop"); // irq_delay = 20  //11
                asm volatile("nop"); // irq_delay = 22  //12
                asm volatile("nop"); // irq_delay = 24  //13
                asm volatile("nop"); // irq_delay = 26  //14
                asm volatile("nop"); // irq_delay = 28  //15
                asm volatile("nop"); // irq_delay = 30  //16
                asm volatile("nop"); // irq_delay = 32  //17
                asm volatile("nop"); // irq_delay = 34  //18
                asm volatile("nop"); // irq_delay = 36  //19
                asm volatile("nop"); // irq_delay = 38  //20
                asm volatile("nop"); // irq_delay = 40  //21
                asm volatile("nop"); // irq_delay = 42  //22
                asm volatile("nop"); // irq_delay = 44  //23
                asm volatile("nop"); // irq_delay = 46  //24
                asm volatile("nop"); // irq_delay = 48  //25
                asm volatile("nop"); // irq_delay = 50  //26
                asm volatile("nop"); // irq_delay = 52  //27
                asm volatile("nop"); // irq_delay = 54  //28
                asm volatile("nop"); // irq_delay = 56  //29
                asm volatile("nop"); // irq_delay = 58  //30
                asm volatile("nop"); // irq_delay = 60  //31
                asm volatile("nop"); // irq_delay = 62  //32
                asm volatile("nop"); // irq_delay = 64  //33
                asm volatile("nop"); // irq_delay = 66  //34

                // needs to be calculated
                // get the assembler and calculate the instructions
                // delayCycles has to be greater than 15
                uint16_t delayCycles = 254; // --> 2cc

                // NOPs for Offset
                // from https://gist.github.com/RickKimball/1643162
                // Note: this Code only works for msp430f1611
                __asm__(
                        " sub #15, %[delayCycles]  \n" // 53fa: sub #15, r15	;#0x0014	--> 2cc
                        "1: sub #4, %[delayCycles] \n" // 53fe: sub #4,  r15	;r2 As==10	--> 2cc
                        " nop                      \n" // 5400: nop						-->	1cc
                        " jc 1b                    \n" // 5402: jc	 $-4      	;abs 0x53fe	-->	2cc
                        " inv %[delayCycles]       \n" // 5404: inv r15				--> 2cc
                        " rla %[delayCycles]       \n" // 5406: rla r15					--> 2cc
                        " add %[delayCycles], r0   \n" // 5408: add r15, r0				-->	2cc
                        //depending on register value rn
                        " nop \n" //rn = 0		// 540a: nop						--> 1cc
                        " nop \n" //rn = 2		// 540c: nop						-->	1cc
                        " nop \n" //rn = 4		// 540e: nop						--> 1cc
                        : // no output
                        : [delayCycles] "r" (delayCycles) // input
                        : // no memory clobber
                        );

                // relay the packet
                whisper_radio_start_tx();
#if CC2420_ENABLE_TESTMODE
                whisper_set_tx_mode_looping();
                //cc2420_set_autocrc(0);
#endif /* CC2420_ENABLE_TESTMODE */
                whisper_end_rx();
                // read TBIV to clear IFG
                whisper_tbiv = TBIV;
            } else {
                // T_rx is too high: do not relay the packet
                //printf("brx%ucr%ust%u\n", whisper_T_rx, tbccr1, whisper_t_rx_start);
                radio_flush_rx();
                whisper_state = WHISPER_STATE_WAITING;
                // read TBIV to clear IFG
                whisper_tbiv = TBIV;
#if LANEFLOOD_DEBUG
                whisper_bad_T_rx++;
#endif /* LANEFLOOD_DEBUG */
            }

#else  /* WHISPER_CLOCKDRIFT_COMP && !COOJA */
            asm volatile("add %[d], r0" : : [d] "m" (whisper_T_irq));
            // Compensate Interrupt
            asm volatile("nop"); // irq_delay = 0
            asm volatile("nop"); // irq_delay = 2
            asm volatile("nop"); // irq_delay = 4
            asm volatile("nop"); // irq_delay = 6
            asm volatile("nop"); // irq_delay = 8
            asm volatile("nop"); // irq_delay = 10

            // delayCycles has to be greater than 15
            uint16_t delayCycles = 289; //277; // --> 2cc

            // NOPs for Offset
            // from https://gist.github.com/RickKimball/1643162
            // Note: this Code only works for msp430f1611
            __asm__(
                    " sub #15, %[delayCycles]  \n" // 53fa: sub #15, r15	;#0x0014	--> 2cc
                    "1: sub #4, %[delayCycles] \n" // 53fe: sub #4,  r15	;r2 As==10	--> 2cc
                    " nop                      \n" // 5400: nop					--> 1cc
                    " jc 1b                    \n" // 5402: jc	 $-4      	;abs 0x53fe	--> 2cc
                    " inv %[delayCycles]       \n" // 5404: inv r15				--> 2cc
                    " rla %[delayCycles]       \n" // 5406: rla r15				--> 2cc
                    " add %[delayCycles], r0   \n" // 5408: add r15, r0				--> 2cc
                    //depending on register value rn
                    " nop \n" //rn = 0		// 540a: nop					--> 1cc
                    " nop \n" //rn = 2		// 540c: nop					--> 1cc
                    " nop \n" //rn = 4		// 540e: nop					--> 1cc
                    : // no output
                    : [delayCycles] "r" (delayCycles) // input
                    : // no memory clobber
                    );
                
                // relay the packet
                whisper_radio_start_tx();
#if CC2420_ENABLE_TESTMODE
                whisper_set_tx_mode(2);
                cc2420_set_autocrc(0);
#endif /* CC2420_ENABLE_TESTMODE */
                whisper_end_rx();
                // read TBIV to clear IFG
                whisper_tbiv = TBIV;
#endif /* WHISPER_CLOCKDRIFT_COMP */

#elif WHISPER_PREAMBLE_LENGTH == 2

#if WHISPER_CLOCKDRIFT_COMP && !COOJA
            rtimer_clock_t whisper_T_rx = (tbccr1 - whisper_t_rx_start) - whisper_clock_drift_offset;
            if (whisper_T_rx < whisper_clock_drift_values) {
                whisper_nops_to_add = (whisper_clock_drift_table[whisper_T_rx]) + whisper_T_irq;
                asm volatile("add %[d], r0" : : [d] "m" (whisper_nops_to_add));

                asm volatile("nop"); // irq_delay = 0
                asm volatile("nop"); // irq_delay = 2
                asm volatile("nop"); // irq_delay = 4
                asm volatile("nop"); // irq_delay = 6
                asm volatile("nop"); // irq_delay = 8
                asm volatile("nop"); // irq_delay = 10
                asm volatile("nop"); // irq_delay = 12
                asm volatile("nop"); // irq_delay = 14
                asm volatile("nop"); // irq_delay = 16
                asm volatile("nop"); // irq_delay = 18
                asm volatile("nop"); // irq_delay = 20
                asm volatile("nop"); // irq_delay = 22
                asm volatile("nop"); // irq_delay = 24

                // relay the packet
                whisper_radio_start_tx();
#if CC2420_ENABLE_TESTMODE
                whisper_set_tx_mode(2);
                cc2420_set_autocrc(0);
#endif /* CC2420_ENABLE_TESTMODE */
                whisper_end_rx();
                // read TBIV to clear IFG
                whisper_tbiv = TBIV;
            } else {
                // T_rx is too high: do not relay the packet
                //printf("brx%ucr%ust%u\n", whisper_T_rx, tbccr1, whisper_t_rx_start);
                radio_flush_rx();
                whisper_state = WHISPER_STATE_WAITING;
                // read TBIV to clear IFG
                whisper_tbiv = TBIV;
#if LANEFLOOD_DEBUG
                whisper_bad_T_rx++;
#endif /* LANEFLOOD_DEBUG */
            }
            
#else  /* WHISPER_CLOCKDRIFT_COMP && !COOJA */
                asm volatile("add %[d], r0" : : [d] "m" (whisper_T_irq));
                asm volatile("nop"); // irq_delay = 0
                asm volatile("nop"); // irq_delay = 2
                asm volatile("nop"); // irq_delay = 4
                asm volatile("nop"); // irq_delay = 6
                asm volatile("nop"); // irq_delay = 8
                asm volatile("nop"); // irq_delay = 10
                asm volatile("nop"); // irq_delay = 12
                asm volatile("nop"); // irq_delay = 14
                asm volatile("nop"); // irq_delay = 16
                asm volatile("nop"); // irq_delay = 18
                asm volatile("nop"); // irq_delay = 20
                asm volatile("nop"); // irq_delay = 22
                asm volatile("nop"); // irq_delay = 24
                asm volatile("nop"); // irq_delay = 26
                asm volatile("nop"); // irq_delay = 28
                asm volatile("nop"); // irq_delay = 30
                asm volatile("nop"); // irq_delay = 32
                asm volatile("nop"); // irq_delay = 34
                asm volatile("nop"); // irq_delay = 36
                asm volatile("nop"); // irq_delay = 12
                asm volatile("nop"); // irq_delay = 14
                asm volatile("nop"); // irq_delay = 16
                asm volatile("nop"); // irq_delay = 18
                
                asm volatile("nop"); // irq_delay = 0
                asm volatile("nop"); // irq_delay = 2
                asm volatile("nop"); // irq_delay = 4
                asm volatile("nop"); // irq_delay = 6
                asm volatile("nop"); // irq_delay = 8
                asm volatile("nop"); // irq_delay = 10
                asm volatile("nop"); // irq_delay = 12
                asm volatile("nop"); // irq_delay = 14
                asm volatile("nop"); // irq_delay = 16
                asm volatile("nop"); // irq_delay = 18
                asm volatile("nop"); // irq_delay = 20
                asm volatile("nop"); // irq_delay = 22
                asm volatile("nop"); // irq_delay = 24
                asm volatile("nop"); // irq_delay = 0
                asm volatile("nop"); // irq_delay = 2
                asm volatile("nop"); // irq_delay = 4
                asm volatile("nop"); // irq_delay = 6
                asm volatile("nop"); // irq_delay = 8
                asm volatile("nop"); // irq_delay = 10
                asm volatile("nop"); // irq_delay = 12
                asm volatile("nop"); // irq_delay = 14
                asm volatile("nop"); // irq_delay = 16
                asm volatile("nop"); // irq_delay = 18
                
                asm volatile("nop"); // irq_delay = 12
                asm volatile("nop"); // irq_delay = 14
                asm volatile("nop"); // irq_delay = 16
                asm volatile("nop"); // irq_delay = 18
                
                asm volatile("nop"); // irq_delay = 12
                asm volatile("nop"); // irq_delay = 14
                //asm volatile("nop"); // irq_delay = 16
                //asm volatile("nop"); // irq_delay = 18
                
                
                // relay the packet
                whisper_radio_start_tx();
#if CC2420_ENABLE_TESTMODE
                whisper_set_tx_mode(2);
                cc2420_set_autocrc(0);
#endif /* CC2420_ENABLE_TESTMODE */
                whisper_end_rx();
                // read TBIV to clear IFG
                whisper_tbiv = TBIV;
                //t_rx_delta_final = (RTIMER_NOW_DCO() - TBCCR1);
                // Ignore the missed packet
                CLEAR_SFD_INT();
#endif /* WHISPER_CLOCKDRIFT_COMP */
            
#else
            printf("[ERROR:] Strange preamble length \n");
#endif           
        } else {
            // interrupt service delay is too high: do not relay the packet
            //printf("Bad T_irq %u\n", whisper_T_irq);
            radio_flush_rx();
            whisper_state = WHISPER_STATE_WAITING;
            // read TBIV to clear IFG
            whisper_tbiv = TBIV;
#if LANEFLOOD_DEBUG
            whisper_high_T_irq++;
#endif /* LANEFLOOD_DEBUG */
        }
    } else {
        // read TBIV to clear IFG
        whisper_tbiv = TBIV;
        if (whisper_state == WHISPER_STATE_TRANSMITTED && SFD_IS_1) {
            //whisper_set_tx_power(CC2420_DEFAULT_TX_POWER); //11 -> -10dBm
            //cc2420_set_tx_power(CC2420_DEFAULT_TX_POWER);
            // packet transmission has started
            whisper_begin_tx();
        } else {
            if (whisper_state == WHISPER_STATE_WAITING && SFD_IS_1) {
                // packet reception has started
                whisper_begin_rx();
            } else {
                if (whisper_state == WHISPER_STATE_TRANSMITTING && !SFD_IS_1) {
                    whisper_end_tx();
                } else {
                        if (whisper_tbiv == TBIV_TBCCR5) {
                            // rx timeout
                            if (whisper_state == WHISPER_STATE_RECEIVING) {
                                // we are still trying to receive a packet: abort the reception
                                radio_abort_rx_whisper();
#if LANEFLOOD_DEBUG
                                whisper_rx_timeout++;
#endif /* LANEFLOOD_DEBUG */
                            }
                            // stop the timeout
                            whisper_stop_rx_timeout();
                        } else {
                            //                            if (!SFD_IS_1 && ((whisper_state != WHISPER_STATE_TRANSMITTING) || (whisper_state != WHISPER_STATE_RECEIVING))) {
                            //                                SET_PIN_ADC1;
                            //                                radio_flush_rx();
                            //                                UNSET_PIN_ADC1;
                            //                            } else {
                            if (whisper_tbiv == COV) {
#if LANEFLOOD_DEBUG
                                whisper_cov++;
                                if (whisper_tbiv == TBIV_TBCCR1) {
                                    whisper_cov_tbccr1++;
//                                } else {
//                                    if (whisper_tbiv == TBIV_TBCCR2) {
//                                        whisper_cov_tbccr2++;
//                                    } else {
//                                        if (whisper_tbiv == TBIV_TBCCR3) {
//                                            whisper_cov_tbccr3++;
//                                        } else {
//                                            if (whisper_tbiv == TBIV_TBCCR4) {
//                                                whisper_cov_tbccr4++;
//                                            } else {
//                                                if (whisper_tbiv == TBIV_TBCCR5) {
//                                                    whisper_cov_tbccr5++;
//                                                } else {
//                                                    if (whisper_tbiv == TBIV_TBCCR6) {
//                                                        whisper_cov_tbccr6++;
//                                                    } else {
//                                                        whisper_cov_tbccr_++;
//                                                    }
//                                                }
//                                            }
//                                        }
//                                    }
                               }
#endif /* LANEFLOOD_DEBUG */
                            } else {
                                if (whisper_state != WHISPER_STATE_OFF) {
                                    //SET_PIN_ADC1;
                                    // something strange is going on: go back to the waiting state
                                    //printf("st: %u, sfd: %u \n", whisper_state, SFD_IS_1);
                                    radio_flush_rx();
                                    whisper_state = WHISPER_STATE_TRANSMITTED;
#if LANEFLOOD_DEBUG_PINS
#if COOJA || LOCAL_NODES
                                    leds_off(LEDS_GREEN);
#endif /* COOJA */
#if FLOCKLAB || LOCAL_NODES
                                    UNSET_PIN_ADC6;
#endif /* FLOCKLAB */
#endif /* LANEFLOOD_DEBUG_PINS */
                                }
                            }
                        }
                    }
                }
            }
    }
}

/* --------------------------- Whisper process ----------------------- */

PROCESS(whisper_process, "Whisper busy-waiting process");

PROCESS_THREAD(whisper_process, ev, data) {
    PROCESS_BEGIN();
    
    whisper_packet = (uint8_t *) & whisper_packet_aligned[0];
    whisper_packet_in = (uint8_t *) & whisper_packet_in_aligned[0];
    lf_header_whisper = (lf_header_whisper_struct *) & whisper_packet_in_aligned[0];

#if LANEFLOOD_DEBUG_PINS
#if FLOCKLAB || LOCAL_NODES
    INIT_PIN_ADC7_OUT;
    INIT_PIN_ADC6_OUT;
    INIT_PIN_ADC2_OUT;
#endif /* FLOCKLAB || LOCAL_NODES */
#if LOCAL_NODES
    INIT_PIN_GIO2_OUT;
    INIT_PIN_GIO3_OUT;
    INIT_PIN_ADC1_OUT;
#endif /* LOCAL_NODES */
#endif /* LANEFLOOD_DEBUG_PINS */

#if WHISPER_RADIO_ALWAYS_ON
    while (1) {
        PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_POLL);
        // prevent the Contiki main cycle to enter the LPM mode or
        // any other process to run while Whisper is running
        
        whisper_t_cancle = whisper_t_stop;
        
        while (WHISPER_IS_ON() && RTIMER_CLOCK_LT(RTIMER_NOW(), whisper_t_stop) && RTIMER_CLOCK_LT(RTIMER_NOW(), whisper_t_cancle)) {
            if (!whisper_initiator && !whisper_got_packet) {
                BUSYWAIT_UNTIL(radio_status() & BV(CC2420_RSSI_VALID), RTIMER_SECOND / 1000);
            }
        }
        
        DISABLE_SFD_INT();
        radio_off();

        if (whisper_got_packet || whisper_initiator) {
            whisper_cca_valid = 1;
            if (!whisper_sync) {
                whisper_transmitted++;
            }
        }
        
#if COOJA
#if !CC2420_ENABLE_TESTMODE
        // Cooja sometimes just does not want to stop transmitting
        while (whisper_state == WHISPER_STATE_TRANSMITTING);
#endif /* !CC2420_ENABLE_TESTMODE */
#endif /* COOJA */
#if LANEFLOOD_DEBUG_PINS
#if COOJA || LOCAL_NODES
        leds_off(LEDS_GREEN);
        leds_off(LEDS_RED);
#endif /* COOJA */
#if FLOCKLAB || LOCAL_NODES
        UNSET_PIN_ADC6;
        UNSET_PIN_ADC7;
#endif /* FLOCKLAB */
#endif /* LANEFLOOD_DEBUG_PINS */
        
        // Whisper finished: execute the callback function
        dint();
        whisper_cb(whisper_rtimer, whisper_ptr);
        eint();
    }
#elif WHISPER_DIRECTION_AWARE_SAMPLING
    while (1) {
        PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_POLL);
        // prevent the Contiki main cycle to enter the LPM mode or
        // any other process to run while Whisper is running

        whisper_t_cancle = whisper_t_stop;

#if WHISPER_DIRECTION_MODE == 1 // Dissemination
        while (WHISPER_IS_ON() && RTIMER_CLOCK_LT(RTIMER_NOW(), whisper_t_stop) && RTIMER_CLOCK_LT(RTIMER_NOW(), whisper_t_cancle)) {
            if (!whisper_initiator && !whisper_got_packet) {
                BUSYWAIT_UNTIL(radio_status() & BV(CC2420_RSSI_VALID), RTIMER_SECOND / 1000);

                if (CCA_IS_1) { // No signal found yet
                    if (whisper_avg_counter_len > 0
#if !COOJA
                            // X ticks für (NTX + 1) packlets + 5 ticks für radio on und stable (128us)
#if WHISPER_PREAMBLE_LENGTH == 4
#if WHISPER_NTX == 2
                            && !RTIMER_CLOCK_LT(RTIMER_NOW(), whisper_avg_start_t[whisper_avg_counter_len - 1] + 34)
#elif WHISPER_NTX == 3
                            && !RTIMER_CLOCK_LT(RTIMER_NOW(), whisper_avg_start_t[whisper_avg_counter_len - 1] + 43)
#elif WHISPER_NTX == 4
                            && !RTIMER_CLOCK_LT(RTIMER_NOW(), whisper_avg_start_t[whisper_avg_counter_len - 1] + 53)
#else
#error [ERROR:] WHISPER_NTX not supported
#endif
#elif WHISPER_PREAMBLE_LENGTH == 2 && WHISPER_PACKET_LENGTH == 7
#if WHISPER_NTX == 2
                            && !RTIMER_CLOCK_LT(RTIMER_NOW(), whisper_avg_start_t[whisper_avg_counter_len - 1] + 28)
#elif WHISPER_NTX == 3
                            && !RTIMER_CLOCK_LT(RTIMER_NOW(), whisper_avg_start_t[whisper_avg_counter_len - 1] + 35)
#elif WHISPER == 4
                            && !RTIMER_CLOCK_LT(RTIMER_NOW(), whisper_avg_start_t[whisper_avg_counter_len - 1] + 42)
#else
#error [ERROR:] WHISPER_NTX not supported
#endif                            
#endif /* WHISPER_PREAMBLE_LENGTH */

#else /* COOJA */
                            // X ticks für (NTX + 1) packlets + 11 ticks for RX Calibrate and RSSI Valid (320us)
#if WHISPER_PREAMBLE_LENGTH == 4
#if WHISPER_NTX == 2
                            && !RTIMER_CLOCK_LT(RTIMER_NOW(), whisper_avg_start_t[whisper_avg_counter_len - 1] + 34)
#elif WHISPER_NTX == 3
                            && !RTIMER_CLOCK_LT(RTIMER_NOW(), whisper_avg_start_t[whisper_avg_counter_len - 1] + 41)
#elif WHISPER_NTX == 4
                            && !RTIMER_CLOCK_LT(RTIMER_NOW(), whisper_avg_start_t[whisper_avg_counter_len - 1] + 48)
#else
#error [ERROR:] WHISPER_NTX not supported
#endif
#elif WHISPER_PREAMBLE_LENGTH == 2 && WHISPER_PACKET_LENGTH == 7
#if WHISPER_NTX == 2
                            && !RTIMER_CLOCK_LT(RTIMER_NOW(), whisper_avg_start_t[whisper_avg_counter_len - 1] + 34)
#elif WHISPER_NTX == 3
                            && !RTIMER_CLOCK_LT(RTIMER_NOW(), whisper_avg_start_t[whisper_avg_counter_len - 1] + 42)
#elif WHISPER_NTX == 4
                            && !RTIMER_CLOCK_LT(RTIMER_NOW(), whisper_avg_start_t[whisper_avg_counter_len - 1] + 54)
#else
#error [ERROR:] WHISPER_NTX not supported
#endif 
#endif
#endif /* !COOJA */                         
                            && !whisper_got_packet) {
                            // Turn radio off
                            break;
                    }
                }              
            }
        }
#elif WHISPER_DIRECTION_MODE == 2 // Collection
        if (!WHISPER_COLLECTION_IS_BOOTSTRAPPING()) {
            while (WHISPER_IS_ON() && RTIMER_CLOCK_LT(RTIMER_NOW(), whisper_t_stop) && RTIMER_CLOCK_LT(RTIMER_NOW(), whisper_t_cancle)) {
                if (!whisper_initiator && !whisper_got_packet) {
                    BUSYWAIT_UNTIL(radio_status() & BV(CC2420_RSSI_VALID), RTIMER_SECOND / 1000);
                    if (CCA_IS_1) {
                        if (whisper_avg_counter_len > 0
                                // We wait until the last counter + 3 packets + radio on
                                // 30ticks für 4 packlets + 5 ticks für radio on und stable (128us)
//#if !COOJA
                            // X ticks für (NTX + 1) packlets + 7 ticks für radio on, stable and ready for packet reception (192us)
#if WHISPER_PREAMBLE_LENGTH == 4
#if WHISPER_NTX == 2
                            && !RTIMER_CLOCK_LT(RTIMER_NOW(), whisper_avg_start_t[whisper_avg_counter_len - 1] + 36)
#elif WHISPER_NTX == 3
                            && !RTIMER_CLOCK_LT(RTIMER_NOW(), whisper_avg_start_t[whisper_avg_counter_len - 1] + 45)
#elif WHISPER_NTX == 4
                            && !RTIMER_CLOCK_LT(RTIMER_NOW(), whisper_avg_start_t[whisper_avg_counter_len - 1] + 55)
#else
#error [ERROR:] WHISPER_NTX not supported
#endif
#elif WHISPER_PREAMBLE_LENGTH == 2 && WHISPER_PACKET_LENGTH == 7
#if WHISPER_NTX == 2
                            && !RTIMER_CLOCK_LT(RTIMER_NOW(), whisper_avg_start_t[whisper_avg_counter_len - 1] + 29)
#elif WHISPER_NTX == 3
                            && !RTIMER_CLOCK_LT(RTIMER_NOW(), whisper_avg_start_t[whisper_avg_counter_len - 1] + 37)
#elif WHISPER_NTX == 4
                            && !RTIMER_CLOCK_LT(RTIMER_NOW(), whisper_avg_start_t[whisper_avg_counter_len - 1] + 44)
#else
#error [ERROR:] WHISPER_NTX not supported
#endif                            
#endif /* WHISPER_PREAMBLE_LENGTH */
                                && !whisper_got_packet) {
                            // Turn radio off
                            break;
                        }
                    }
                }
            }
        } else {
            while (WHISPER_IS_ON() && RTIMER_CLOCK_LT(RTIMER_NOW(), whisper_t_stop) && RTIMER_CLOCK_LT(RTIMER_NOW(), whisper_t_cancle)) {
                if (!whisper_initiator && !whisper_got_packet) {
                    BUSYWAIT_UNTIL(radio_status() & BV(CC2420_RSSI_VALID), RTIMER_SECOND / 1000);
                }
            }
        }
#elif WHISPER_DIRECTION_MODE == 3 // One-to-one
        if (!WHISPER_ONE_TO_ONE_IS_BOOTSTRAPPING()) {
            while (WHISPER_IS_ON() && RTIMER_CLOCK_LT(RTIMER_NOW(), whisper_t_stop) && RTIMER_CLOCK_LT(RTIMER_NOW(), whisper_t_cancle)) {
                if (!whisper_initiator && !whisper_got_packet) {
                    BUSYWAIT_UNTIL(radio_status() & BV(CC2420_RSSI_VALID), RTIMER_SECOND / 1000);
                    if (CCA_IS_1) {
                        if (whisper_avg_counter_len > 0
                                // We wait until the last counter + 3 packets + radio on
                                // 30ticks für 4 packlets + 5 ticks für radio on und stable (128us)
#if !COOJA
                            // X ticks für (NTX + 1) packlets + 5 ticks für radio on und stable (128us)
#if WHISPER_PREAMBLE_LENGTH == 4
#if WHISPER_NTX == 2
                            && !RTIMER_CLOCK_LT(RTIMER_NOW(), whisper_avg_start_t[whisper_avg_counter_len - 1] + 34)
#elif WHISPER_NTX == 3
                            && !RTIMER_CLOCK_LT(RTIMER_NOW(), whisper_avg_start_t[whisper_avg_counter_len - 1] + 43)
#elif WHISPER_NTX == 4
                            && !RTIMER_CLOCK_LT(RTIMER_NOW(), whisper_avg_start_t[whisper_avg_counter_len - 1] + 53)
#else
#error [ERROR:] WHISPER_NTX not supported
#endif
#elif WHISPER_PREAMBLE_LENGTH == 2 && WHISPER_PACKET_LENGTH == 7
#if WHISPER_NTX == 2
                            && !RTIMER_CLOCK_LT(RTIMER_NOW(), whisper_avg_start_t[whisper_avg_counter_len - 1] + 28)
#elif WHISPER_NTX == 3
                            && !RTIMER_CLOCK_LT(RTIMER_NOW(), whisper_avg_start_t[whisper_avg_counter_len - 1] + 35)
#elif WHISPER_NTX == 4
                            && !RTIMER_CLOCK_LT(RTIMER_NOW(), whisper_avg_start_t[whisper_avg_counter_len - 1] + 42)
#else
#error [ERROR:] WHISPER_NTX not supported
#endif                            
#endif /* WHISPER_PREAMBLE_LENGTH */
#else /* COOJA */
                            // X ticks für (NTX + 1) packlets + 11 ticks for RX Calibrate and RSSI Valid (320us)
#if WHISPER_PREAMBLE_LENGTH == 4
#if WHISPER_NTX == 2
                            && !RTIMER_CLOCK_LT(RTIMER_NOW(), whisper_avg_start_t[whisper_avg_counter_len - 1] + 40)
#elif WHISPER_NTX == 3
                            && !RTIMER_CLOCK_LT(RTIMER_NOW(), whisper_avg_start_t[whisper_avg_counter_len - 1] + 49)
#elif WHISPER_NTX == 4
                            && !RTIMER_CLOCK_LT(RTIMER_NOW(), whisper_avg_start_t[whisper_avg_counter_len - 1] + 59)
#else
#error [ERROR:] WHISPER_NTX not supported
#endif
#elif WHISPER_PREAMBLE_LENGTH == 2 && WHISPER_PACKET_LENGTH == 7
#if WHISPER_NTX == 2
                            && !RTIMER_CLOCK_LT(RTIMER_NOW(), whisper_avg_start_t[whisper_avg_counter_len - 1] + 34)
#elif WHISPER_NTX == 3
                            && !RTIMER_CLOCK_LT(RTIMER_NOW(), whisper_avg_start_t[whisper_avg_counter_len - 1] + 42)
#elif WHISPER_NTX == 4
                            && !RTIMER_CLOCK_LT(RTIMER_NOW(), whisper_avg_start_t[whisper_avg_counter_len - 1] + 48)
#else
#error [ERROR:] WHISPER_NTX not supported
#endif 
#endif
#endif /* !COOJA */  
                                && !whisper_got_packet) {
                            // Turn radio off
                            break;
                        }
                    }
                }
            }
        } else {
            // We are bootstrapping
            while (WHISPER_IS_ON() && RTIMER_CLOCK_LT(RTIMER_NOW(), whisper_t_stop) && RTIMER_CLOCK_LT(RTIMER_NOW(), whisper_t_cancle)) {
                //TOGGLE_PIN_GIO2;
                if (!whisper_initiator && !whisper_got_packet) {
                    BUSYWAIT_UNTIL(radio_status() & BV(CC2420_RSSI_VALID), RTIMER_SECOND / 1000);
                }
            }
        }
        
#endif /* WHISPER_DIRECTION_MODE */
       
        DISABLE_SFD_INT();
        radio_off();
        
        if (whisper_got_packet || whisper_initiator) {
            whisper_cca_valid = 1;
            if (!whisper_sync) {
                whisper_transmitted++;
            }
        }

#if COOJA
#if !CC2420_ENABLE_TESTMODE
        // Cooja sometimes just does not want to stop transmitting
        while (whisper_state == WHISPER_STATE_TRANSMITTING);
#endif /* !CC2420_ENABLE_TESTMODE */ 
#endif /* COOJA */
#if LANEFLOOD_DEBUG_PINS
#if COOJA || LOCAL_NODES
        leds_off(LEDS_GREEN);
        leds_off(LEDS_RED);
#endif /* COOJA */
#if FLOCKLAB || LOCAL_NODES
        UNSET_PIN_ADC6;
        UNSET_PIN_ADC7;
#endif /* FLOCKLAB */
#endif /* LANEFLOOD_DEBUG_PINS */

        // Whisper finished: execute the callback function
        dint();
        whisper_cb(whisper_rtimer, whisper_ptr);
        eint();
    }
#endif /* WHISPER_RADIO_ALWAYS_ON */
    PROCESS_END();    

}

/* -------------------- Whisper interrupts -------------------- */

static inline void
whisper_enable_other_interrupts(void) {
    int s = splhigh();
    IE1 = whisper_ie1;
    IE2 = whisper_ie2;
    P1IE = whisper_p1ie;
    P2IE = whisper_p2ie;
    // enable etimer interrupts
    TACCTL1 |= CCIE;
#if COOJA
    if (TACCTL1 & CCIFG) {
        etimer_interrupt();
    }
#endif
    DISABLE_SFD_INT();
    CLEAR_SFD_INT();
    FIFOP_INT_INIT();
    ENABLE_FIFOP_INT();
    // stop Timer B
    TBCTL = 0;
    // Timer B sourced by the 32 kHz
    TBCTL = TBSSEL0;
    // start Timer B
    TBCTL |= MC1;
    splx(s);
    watchdog_start();
}

static inline void 
whisper_disable_other_interrupts(void) {
    int s = splhigh();
    whisper_ie1 = IE1;
    whisper_ie2 = IE2;
    whisper_p1ie = P1IE;
    whisper_p2ie = P2IE;
    IE1 = 0;
    IE2 = 0;
    P1IE = 0;
    P2IE = 0;
    CACTL1 &= ~CAIE;
    DMA0CTL &= ~DMAIE;
    DMA1CTL &= ~DMAIE;
    DMA2CTL &= ~DMAIE;
    // disable etimer interrupts
    TACCTL1 &= ~CCIE;
    //TACCTL1 = 0;
    TBCCTL0 = 0;
    DISABLE_FIFOP_INT();
    CLEAR_FIFOP_INT();
    SFD_CAP_INIT(CM_BOTH);
    ENABLE_SFD_INT();
    // stop Timer B
    TBCTL = 0;
    // Reset Timer B
    TBCTL = TBCLR;
    // Timer B sourced by the DCO
    TBCTL |= TBSSEL1;
    // start Timer B
    TBCTL |= MC1;
    splx(s);
    watchdog_stop();
}

/* Whisper create packet */

uint8_t whisper_create_packet_preamble_len4(void) {
    uint16_t crc;
    uint8_t len = WHISPER_DATA_LEN; // p1
    len += 7 + WHISPER_LEN_LEN + WHISPER_DATA_LEN; // p2
    len += 7 + WHISPER_LEN_LEN + WHISPER_DATA_LEN; // p3
    len += 7 + WHISPER_LEN_LEN + WHISPER_DATA_LEN; // p4
    len += 7 + WHISPER_LEN_LEN + WHISPER_DATA_LEN; // p5
    len += 7 + WHISPER_LEN_LEN + WHISPER_DATA_LEN; // p6
    len += 7 + WHISPER_LEN_LEN + WHISPER_DATA_LEN; // p7
    len += 7 + WHISPER_LEN_LEN + WHISPER_DATA_LEN; // p8
    len += 7 + WHISPER_LEN_LEN + WHISPER_DATA_LEN; // p9
    len += 7 + WHISPER_LEN_LEN + WHISPER_DATA_LEN; // p10
    len += 7 + WHISPER_LEN_LEN + WHISPER_DATA_LEN; // p11
    len += 7 + WHISPER_LEN_LEN + WHISPER_DATA_LEN; // p12
    len += 7 + WHISPER_LEN_LEN + WHISPER_DATA_LEN; // p13
    len += 7 + WHISPER_LEN_LEN + WHISPER_DATA_LEN + FOOTER_LEN; // p14
#if !CC2420_ENABLE_TESTMODE
    len += FOOTER_LEN; // last footer
#else
    //len += 7 + WHISPER_LEN_LEN + WHISPER_DATA_LEN; // p15
    //len += 7 + WHISPER_LEN_LEN + WHISPER_DATA_LEN; // p16
#endif /* !CC2420_ENABLE_TESTMODE */

        // packet 1
#if !CC2420_ENABLE_TESTMODE
        whisper_packet[0] = len;
#else
        whisper_packet[0] = WHISPER_DATA_LEN + FOOTER_LEN;
#endif
        whisper_packet[1] = LANEFLOOD_TYPE | 0;
        crc = (crcByte(0, whisper_packet[1]));
        whisper_packet[2] = crc & 0xff;
        whisper_packet[3] = crc >> 8;

        // packet 2
        whisper_packet[4] = 0;
        whisper_packet[5] = 0;
        whisper_packet[6] = 0;
        whisper_packet[7] = 0;
        whisper_packet[8] = 0xa7;
        whisper_packet[9] = WHISPER_DATA_LEN + FOOTER_LEN;
        whisper_packet[10] = whisper_packet[1] + 1;
        crc = (crcByte(0, whisper_packet[10]));
        whisper_packet[11] = crc & 0xff;
        whisper_packet[12] = crc >> 8;

        // packet 3
        whisper_packet[13] = 0;
        whisper_packet[14] = 0;
        whisper_packet[15] = 0;
        whisper_packet[16] = 0;
        whisper_packet[17] = 0xa7;
        whisper_packet[18] = WHISPER_DATA_LEN + FOOTER_LEN;
        whisper_packet[19] = whisper_packet[10] + 1;
        crc = (crcByte(0, whisper_packet[19]));
        whisper_packet[20] = crc & 0xff;
        whisper_packet[21] = crc >> 8;

        // packet 4
        whisper_packet[22] = (uint32_t) 0x00;
        whisper_packet[26] = 0xa7;
        whisper_packet[27] = WHISPER_DATA_LEN + FOOTER_LEN;
        whisper_packet[28] = whisper_packet[19] + 1;
        crc = (crcByte(0, whisper_packet[28]));
        whisper_packet[29] = crc & 0xff;
        whisper_packet[30] = crc >> 8;
 
        // packet 5
        whisper_packet[31] = (uint32_t) 0x00;
        whisper_packet[35] = 0xa7;
        whisper_packet[36] = WHISPER_DATA_LEN + FOOTER_LEN;
        whisper_packet[37] = whisper_packet[28] + 1;
        crc = (crcByte(0, whisper_packet[37]));
        whisper_packet[38] = crc & 0xff;
        whisper_packet[39] = crc >> 8;

        // packet 6
        whisper_packet[40] = (uint32_t) 0x00;
        whisper_packet[44] = 0xa7;
        whisper_packet[45] = WHISPER_DATA_LEN + FOOTER_LEN;
        whisper_packet[46] = whisper_packet[37] + 1;
        crc = (crcByte(0, whisper_packet[46]));
        whisper_packet[47] = crc & 0xff;
        whisper_packet[48] = crc >> 8;

        // packet 7
        whisper_packet[49] = (uint32_t) 0x00;
        whisper_packet[53] = 0xa7;
        whisper_packet[54] = WHISPER_DATA_LEN + FOOTER_LEN;
        whisper_packet[55] = whisper_packet[46] + 1;
        crc = (crcByte(0, whisper_packet[55]));
        whisper_packet[56] = crc & 0xff;
        whisper_packet[57] = crc >> 8;

        // packet 8
        whisper_packet[58] = (uint32_t) 0x00;
        whisper_packet[62] = 0xa7;
        whisper_packet[63] = WHISPER_DATA_LEN + FOOTER_LEN;
        whisper_packet[64] = whisper_packet[55] + 1;
        crc = (crcByte(0, whisper_packet[64]));
        whisper_packet[65] = crc & 0xff;
        whisper_packet[66] = crc >> 8;

        // packet 9
        whisper_packet[67] = (uint32_t) 0x00;
        whisper_packet[71] = 0xa7;
        whisper_packet[72] = WHISPER_DATA_LEN + FOOTER_LEN;
        whisper_packet[73] = whisper_packet[64] + 1;
        crc = (crcByte(0, whisper_packet[73]));
        whisper_packet[74] = crc & 0xff;
        whisper_packet[75] = crc >> 8;

        // packet 10
        whisper_packet[76] = (uint32_t) 0x00;
        whisper_packet[80] = 0xa7;
        whisper_packet[81] = WHISPER_DATA_LEN + FOOTER_LEN;
        whisper_packet[82] = whisper_packet[73] + 1;
        crc = (crcByte(0, whisper_packet[82]));
        whisper_packet[83] = crc & 0xff;
        whisper_packet[84] = crc >> 8;

        // packet 11
        whisper_packet[85] = (uint32_t) 0x00;
        whisper_packet[89] = 0xa7;
        whisper_packet[90] = WHISPER_DATA_LEN + FOOTER_LEN;
        whisper_packet[91] = whisper_packet[82] + 1;
        crc = (crcByte(0, whisper_packet[91]));
        whisper_packet[92] = crc & 0xff;
        whisper_packet[93] = crc >> 8;

        // packet 12
        whisper_packet[94] = (uint32_t) 0x00;
        whisper_packet[98] = 0xa7;
        whisper_packet[99] = WHISPER_DATA_LEN + FOOTER_LEN;
        whisper_packet[100] = whisper_packet[91] + 1;
        crc = (crcByte(0, whisper_packet[100]));
        whisper_packet[101] = crc & 0xff;
        whisper_packet[102] = crc >> 8;

        // packet 13
        whisper_packet[103] = (uint32_t) 0x00;
        whisper_packet[107] = 0xa7;
        whisper_packet[108] = WHISPER_DATA_LEN + FOOTER_LEN;
        whisper_packet[109] = whisper_packet[100] + 1;
        crc = (crcByte(0, whisper_packet[109]));
        whisper_packet[110] = crc & 0xff;
        whisper_packet[111] = crc >> 8;

        // packet 14
        whisper_packet[112] = (uint32_t) 0x00;
        whisper_packet[116] = 0xa7;
        whisper_packet[117] = WHISPER_DATA_LEN + FOOTER_LEN;
        whisper_packet[118] = whisper_packet[109] + 1;
        crc = (crcByte(0, whisper_packet[118]));
        whisper_packet[119] = crc & 0xff;
        whisper_packet[120] = crc >> 8;
        
#if CC2420_ENABLE_TESTMODE && !WHISPER_LONG_WAKEUP_PACKET
        // packet 15
        whisper_packet[121] = (uint32_t) 0x00;
        whisper_packet[125] = 0xa7;
        whisper_packet[126] = WHISPER_DATA_LEN + FOOTER_LEN;
        whisper_packet[127] = whisper_packet[118] + 1;
        crc = (crcByte(0, whisper_packet[127]));
        whisper_packet[128] = crc & 0xff;
        whisper_packet[129] = crc >> 8;
        
        // packet 16
        whisper_packet[130] = (uint32_t) 0x00;
        whisper_packet[134] = 0xa7;
        whisper_packet[135] = WHISPER_DATA_LEN + FOOTER_LEN;
        whisper_packet[136] = whisper_packet[127] + 1;
        crc = (crcByte(0, whisper_packet[136]));
        whisper_packet[137] = crc & 0xff;
        whisper_packet[138] = crc >> 8;
#endif
       return len;
}

uint8_t whisper_create_packet7_preamble_len2(uint8_t packets) {
    uint16_t crc;
    uint8_t len = WHISPER_DATA_LEN; // p1
    len += 5 + WHISPER_LEN_LEN + WHISPER_DATA_LEN; // p2
    len += 5 + WHISPER_LEN_LEN + WHISPER_DATA_LEN; // p3
    len += 5 + WHISPER_LEN_LEN + WHISPER_DATA_LEN; // p4
    len += 5 + WHISPER_LEN_LEN + WHISPER_DATA_LEN; // p5
    len += 5 + WHISPER_LEN_LEN + WHISPER_DATA_LEN; // p6
    len += 5 + WHISPER_LEN_LEN + WHISPER_DATA_LEN; // p7
    len += 5 + WHISPER_LEN_LEN + WHISPER_DATA_LEN; // p8
    len += 5 + WHISPER_LEN_LEN + WHISPER_DATA_LEN; // p9
    len += 5 + WHISPER_LEN_LEN + WHISPER_DATA_LEN; // p10
    len += 5 + WHISPER_LEN_LEN + WHISPER_DATA_LEN; // p11
    len += 5 + WHISPER_LEN_LEN + WHISPER_DATA_LEN; // p12
    len += 5 + WHISPER_LEN_LEN + WHISPER_DATA_LEN; // p13
    len += 5 + WHISPER_LEN_LEN + WHISPER_DATA_LEN; // p14
    len += 5 + WHISPER_LEN_LEN + WHISPER_DATA_LEN; // p15
    len += 5 + WHISPER_LEN_LEN + WHISPER_DATA_LEN; // p16
    len += 5 + WHISPER_LEN_LEN + WHISPER_DATA_LEN; // p17
    len += 5 + WHISPER_LEN_LEN + WHISPER_DATA_LEN + FOOTER_LEN; // p18
    //len += 5 + WHISPER_LEN_LEN + WHISPER_DATA_LEN + FOOTER_LEN; // p17
    //len += 5 + WHISPER_LEN_LEN + WHISPER_DATA_LEN + FOOTER_LEN; // p16
    //len += 5 + WHISPER_LEN_LEN + WHISPER_DATA_LEN + FOOTER_LEN; // p15
    //len += 5 + WHISPER_LEN_LEN + WHISPER_DATA_LEN + FOOTER_LEN; // p14
    //len += 5 + WHISPER_LEN_LEN + WHISPER_DATA_LEN + FOOTER_LEN; // p13
    //len += 5 + WHISPER_LEN_LEN + WHISPER_DATA_LEN + FOOTER_LEN; // p12
    //len += 5 + WHISPER_LEN_LEN + WHISPER_DATA_LEN + FOOTER_LEN; // p11
#if !CC2420_ENABLE_TESTMODE
    len += FOOTER_LEN; // last footer
#endif /* !CC2420_ENABLE_TESTMODE */
    if (packets <= 14) {
        // packet 1 | counter 0
#if !CC2420_ENABLE_TESTMODE
        whisper_packet[0] = len;
#else
        whisper_packet[0] = WHISPER_DATA_LEN + FOOTER_LEN;;
#endif
        whisper_packet[1] = LANEFLOOD_TYPE | 0;
        crc = (crcByte(0, whisper_packet[1]));
        whisper_packet[2] = crc & 0xff;
        whisper_packet[3] = crc >> 8;

        // packet 2 | counter 1
        whisper_packet[4] = (uint16_t) 0x00;
//#if CC2420_ENABLE_TESTMODE
        whisper_packet[6] = 0xA7;
//#else
//        whisper_packet[6] = 0xA5;
//#endif /* CC2420_ENABLE_TESTMODE */
        whisper_packet[7] = WHISPER_DATA_LEN + FOOTER_LEN;
        whisper_packet[8] = whisper_packet[1] + 1;
        crc = (crcByte(0, whisper_packet[8]));
        whisper_packet[9] = crc & 0xff;
        whisper_packet[10] = crc >> 8;

        // packet 3 | counter 2
        whisper_packet[11] = (uint16_t) 0x00;
//#if CC2420_ENABLE_TESTMODE
        whisper_packet[13] = 0xA7;
//#else
//        whisper_packet[13] = 0xA5;
//#endif /* CC2420_ENABLE_TESTMODE */
        whisper_packet[14] = WHISPER_DATA_LEN + FOOTER_LEN;
        whisper_packet[15] = whisper_packet[8] + 1;
        crc = (crcByte(0, whisper_packet[15]));
        whisper_packet[16] = crc & 0xff;
        whisper_packet[17] = crc >> 8;

        // packet 4 | counter 3
        whisper_packet[18] = (uint16_t) 0x00;
//#if CC2420_ENABLE_TESTMODE
        whisper_packet[20] = 0xA7;
//#else
//        whisper_packet[20] = 0xA5;
//#endif /* CC2420_ENABLE_TESTMODE */
        whisper_packet[21] = WHISPER_DATA_LEN + FOOTER_LEN;
        whisper_packet[22] = whisper_packet[15] + 1;
        crc = (crcByte(0, whisper_packet[22]));
        whisper_packet[23] = crc & 0xff;
        whisper_packet[24] = crc >> 8;

        // packet 5
        whisper_packet[25] = (uint16_t) 0x00;
//#if CC2420_ENABLE_TESTMODE
        whisper_packet[27] = 0xA7;
//#else
//        whisper_packet[27] = 0xA5;
//#endif /* CC2420_ENABLE_TESTMODE */
        whisper_packet[28] = WHISPER_DATA_LEN + FOOTER_LEN;
        whisper_packet[29] = whisper_packet[22] + 1;
        crc = (crcByte(0, whisper_packet[29]));
        whisper_packet[30] = crc & 0xff;
        whisper_packet[31] = crc >> 8;

        // packet 6
        whisper_packet[32] = (uint16_t) 0x00;
//#if CC2420_ENABLE_TESTMODE
        whisper_packet[34] = 0xA7;
//#else
//        whisper_packet[34] = 0xA5;
//#endif /* CC2420_ENABLE_TESTMODE */
        whisper_packet[35] = WHISPER_DATA_LEN + FOOTER_LEN;
        whisper_packet[36] = whisper_packet[29] + 1;
        crc = (crcByte(0, whisper_packet[36]));
        whisper_packet[37] = crc & 0xff;
        whisper_packet[38] = crc >> 8;

        // packet 7
        whisper_packet[39] = (uint16_t) 0x00;
//#if CC2420_ENABLE_TESTMODE
        whisper_packet[41] = 0xA7;
//#else
//        whisper_packet[41] = 0xA5;
//#endif /* CC2420_ENABLE_TESTMODE */
        whisper_packet[42] = WHISPER_DATA_LEN + FOOTER_LEN;
        whisper_packet[43] = whisper_packet[36] + 1;
        crc = (crcByte(0, whisper_packet[43]));
        whisper_packet[44] = crc & 0xff;
        whisper_packet[45] = crc >> 8;

        // packet 8
        whisper_packet[46] = (uint16_t) 0x00;
//#if CC2420_ENABLE_TESTMODE
        whisper_packet[48] = 0xA7;
//#else
//        whisper_packet[48] = 0xA5;
//#endif /* CC2420_ENABLE_TESTMODE */
        whisper_packet[49] = WHISPER_DATA_LEN + FOOTER_LEN;
        whisper_packet[50] = whisper_packet[43] + 1;
        crc = (crcByte(0, whisper_packet[50]));
        whisper_packet[51] = crc & 0xff;
        whisper_packet[52] = crc >> 8;

        // packet 9
        whisper_packet[53] = (uint16_t) 0x00;
//#if CC2420_ENABLE_TESTMODE
        whisper_packet[55] = 0xA7;
//#else
//        whisper_packet[55] = 0xA5;
//#endif /* CC2420_ENABLE_TESTMODE */
        whisper_packet[56] = WHISPER_DATA_LEN + FOOTER_LEN;
        whisper_packet[57] = whisper_packet[50] + 1;
        crc = (crcByte(0, whisper_packet[57]));
        whisper_packet[58] = crc & 0xff;
        whisper_packet[59] = crc >> 8;

        // packet 10
        whisper_packet[60] = (uint16_t) 0x00;
//#if CC2420_ENABLE_TESTMODE
        whisper_packet[62] = 0xA7;
//#else
//        whisper_packet[62] = 0xA5;
//#endif /* CC2420_ENABLE_TESTMODE */
        whisper_packet[63] = WHISPER_DATA_LEN + FOOTER_LEN;
        whisper_packet[64] = whisper_packet[57] + 1;
        crc = (crcByte(0, whisper_packet[64]));
        whisper_packet[65] = crc & 0xff;
        whisper_packet[66] = crc >> 8;

        // packet 11
        whisper_packet[67] = (uint16_t) 0x00;
//#if CC2420_ENABLE_TESTMODE
        whisper_packet[69] = 0xA7;
//#else
//        whisper_packet[69] = 0xA5;
//#endif /* CC2420_ENABLE_TESTMODE */
        whisper_packet[70] = WHISPER_DATA_LEN + FOOTER_LEN;
        whisper_packet[71] = whisper_packet[64] + 1;
        crc = (crcByte(0, whisper_packet[71]));
        whisper_packet[72] = crc & 0xff;
        whisper_packet[73] = crc >> 8;

        // packet 12
        whisper_packet[74] = (uint16_t) 0x00;
//#if CC2420_ENABLE_TESTMODE
        whisper_packet[76] = 0xA7;
//#else
//        whisper_packet[76] = 0xA5;
//#endif /* CC2420_ENABLE_TESTMODE */
        whisper_packet[77] = WHISPER_DATA_LEN + FOOTER_LEN;
        whisper_packet[78] = whisper_packet[71] + 1;
        crc = (crcByte(0, whisper_packet[78]));
        whisper_packet[79] = crc & 0xff;
        whisper_packet[80] = crc >> 8;

        // packet 13
        whisper_packet[81] = (uint16_t) 0x00;
//#if CC2420_ENABLE_TESTMODE
        whisper_packet[83] = 0xA7;
//#else
//        whisper_packet[83] = 0xA5;
//#endif /* CC2420_ENABLE_TESTMODE */
        whisper_packet[84] = WHISPER_DATA_LEN + FOOTER_LEN;
        whisper_packet[85] = whisper_packet[78] + 1;
        crc = (crcByte(0, whisper_packet[85]));
        whisper_packet[86] = crc & 0xff;
        whisper_packet[87] = crc >> 8;

        // packet 14
        whisper_packet[88] = (uint16_t) 0x00;
//#if CC2420_ENABLE_TESTMODE
        whisper_packet[90] = 0xA7;
//#else
//        whisper_packet[90] = 0xA5;
//#endif /* CC2420_ENABLE_TESTMODE */
        whisper_packet[91] = WHISPER_DATA_LEN + FOOTER_LEN;
        whisper_packet[92] = whisper_packet[85] + 1;
        crc = (crcByte(0, whisper_packet[92]));
        whisper_packet[93] = crc & 0xff;
        whisper_packet[94] = crc >> 8;

        //---------
        // packet 15
        whisper_packet[95] = (uint16_t) 0x00;
//#if CC2420_ENABLE_TESTMODE
        whisper_packet[97] = 0xA7;
//#else
//        whisper_packet[97] = 0xA5;
//#endif /* CC2420_ENABLE_TESTMODE */
        whisper_packet[98] = WHISPER_DATA_LEN + FOOTER_LEN;
        whisper_packet[99] = whisper_packet[92] + 1;
        crc = (crcByte(0, whisper_packet[99]));
        whisper_packet[100] = crc & 0xff;
        whisper_packet[101] = crc >> 8;

        // packet 16
        whisper_packet[102] = (uint16_t) 0x00;
//#if CC2420_ENABLE_TESTMODE
        whisper_packet[104] = 0xA7;
//#else
//        whisper_packet[104] = 0xA5;
//#endif /* CC2420_ENABLE_TESTMODE */
        whisper_packet[105] = WHISPER_DATA_LEN + FOOTER_LEN;
        whisper_packet[106] = whisper_packet[99] + 1;
        crc = (crcByte(0, whisper_packet[106]));
        whisper_packet[107] = crc & 0xff;
        whisper_packet[108] = crc >> 8;

        // packet 17
        whisper_packet[109] = (uint16_t) 0x00;
//#if CC2420_ENABLE_TESTMODE
        whisper_packet[111] = 0xA7;
//#else
//        whisper_packet[111] = 0xA5;
//#endif /* CC2420_ENABLE_TESTMODE */
        whisper_packet[112] = WHISPER_DATA_LEN + FOOTER_LEN;
        whisper_packet[113] = whisper_packet[106] + 1;
        crc = (crcByte(0, whisper_packet[113]));
        whisper_packet[114] = crc & 0xff;
        whisper_packet[115] = crc >> 8;

        // packet 18
        whisper_packet[116] = (uint16_t) 0x00;
//#if CC2420_ENABLE_TESTMODE
        whisper_packet[118] = 0xA7;
//#else
//        whisper_packet[118] = 0xA5;
//#endif /* CC2420_ENABLE_TESTMODE */
        whisper_packet[119] = WHISPER_DATA_LEN + FOOTER_LEN;
        whisper_packet[120] = whisper_packet[113] + 1;
        crc = (crcByte(0, whisper_packet[120]));
        whisper_packet[121] = crc & 0xff;
        whisper_packet[122] = crc >> 8;

        return len;
    } else {
        return 0;
    }
}

uint8_t whisper_create_packet5_preamble_len2(uint8_t packets) {
    uint8_t len = WHISPER_DATA_LEN; // p1
    len += 3 + WHISPER_LEN_LEN + WHISPER_DATA_LEN; // p2
    len += 3 + WHISPER_LEN_LEN + WHISPER_DATA_LEN; // p3
    len += 3 + WHISPER_LEN_LEN + WHISPER_DATA_LEN; // p4
    len += 3 + WHISPER_LEN_LEN + WHISPER_DATA_LEN; // p5
    len += 3 + WHISPER_LEN_LEN + WHISPER_DATA_LEN; // p6
    len += 3 + WHISPER_LEN_LEN + WHISPER_DATA_LEN; // p7
    len += 3 + WHISPER_LEN_LEN + WHISPER_DATA_LEN; // p8
    len += 3 + WHISPER_LEN_LEN + WHISPER_DATA_LEN; // p9
    len += 3 + WHISPER_LEN_LEN + WHISPER_DATA_LEN; // p10
    len += 3 + WHISPER_LEN_LEN + WHISPER_DATA_LEN; // p11
    len += 3 + WHISPER_LEN_LEN + WHISPER_DATA_LEN; // p12
    len += 3 + WHISPER_LEN_LEN + WHISPER_DATA_LEN; // p13
    len += 3 + WHISPER_LEN_LEN + WHISPER_DATA_LEN; // p14
    len += 3 + WHISPER_LEN_LEN + WHISPER_DATA_LEN; // p15
    len += 3 + WHISPER_LEN_LEN + WHISPER_DATA_LEN; // p16
    len += 3 + WHISPER_LEN_LEN + WHISPER_DATA_LEN; // p17
    len += 3 + WHISPER_LEN_LEN + WHISPER_DATA_LEN; // p18
    len += 3 + WHISPER_LEN_LEN + WHISPER_DATA_LEN; // p19
    len += 3 + WHISPER_LEN_LEN + WHISPER_DATA_LEN; // p20
    len += 3 + WHISPER_LEN_LEN + WHISPER_DATA_LEN; // p21
    len += 3 + WHISPER_LEN_LEN + WHISPER_DATA_LEN; // p22
    len += 3 + WHISPER_LEN_LEN + WHISPER_DATA_LEN; // p23
    len += 3 + WHISPER_LEN_LEN + WHISPER_DATA_LEN; // p24
    len += 3 + WHISPER_LEN_LEN + WHISPER_DATA_LEN + FOOTER_LEN; // p25
    //len += FOOTER_LEN; // last footer
    if (packets <= 14) {
        // packet 1 | counter 0
        whisper_packet[0] = len;
        whisper_packet[1] = LANEFLOOD_TYPE | 0;

        // packet 2 | counter 1
        whisper_packet[2] = (uint16_t) 0x00;
        whisper_packet[4] = 0xA5;
        whisper_packet[5] = WHISPER_DATA_LEN;
        whisper_packet[6] = whisper_packet[1] + 1;

        // packet 3 | counter 2
        whisper_packet[7] = (uint16_t) 0x00;
        whisper_packet[9] = 0xA5;
        whisper_packet[10] = WHISPER_DATA_LEN;
        whisper_packet[11] = whisper_packet[6] + 1;

        // packet 4 | counter 3
        whisper_packet[12] = (uint16_t) 0x00;
        whisper_packet[14] = 0xA5;
        whisper_packet[15] = WHISPER_DATA_LEN;
        whisper_packet[16] = whisper_packet[11] + 1;

        // packet 5
        whisper_packet[17] = (uint16_t) 0x00;
        whisper_packet[19] = 0xA5;
        whisper_packet[20] = WHISPER_DATA_LEN;
        whisper_packet[21] = whisper_packet[16] + 1;

        // packet 6
        whisper_packet[22] = (uint16_t) 0x00;
        whisper_packet[24] = 0xA5;
        whisper_packet[25] = WHISPER_DATA_LEN;
        whisper_packet[26] = whisper_packet[21] + 1;

        // packet 7
        whisper_packet[27] = (uint16_t) 0x00;
        whisper_packet[29] = 0xA5;
        whisper_packet[30] = WHISPER_DATA_LEN;
        whisper_packet[31] = whisper_packet[26] + 1;

        // packet 8
        whisper_packet[32] = (uint16_t) 0x00;
        whisper_packet[34] = 0xA5;
        whisper_packet[35] = WHISPER_DATA_LEN;
        whisper_packet[36] = whisper_packet[31] + 1;

        // packet 9
        whisper_packet[37] = (uint16_t) 0x00;
        whisper_packet[39] = 0xA5;
        whisper_packet[40] = WHISPER_DATA_LEN;
        whisper_packet[41] = whisper_packet[36] + 1;

        // packet 10
        whisper_packet[42] = (uint16_t) 0x00;
        whisper_packet[44] = 0xA5;
        whisper_packet[45] = WHISPER_DATA_LEN;
        whisper_packet[46] = whisper_packet[41] + 1;

        // packet 11
        whisper_packet[47] = (uint16_t) 0x00;
        whisper_packet[49] = 0xA5;
        whisper_packet[50] = WHISPER_DATA_LEN;
        whisper_packet[51] = whisper_packet[46] + 1;

        // packet 12
        whisper_packet[52] = (uint16_t) 0x00;
        whisper_packet[54] = 0xA5;
        whisper_packet[55] = WHISPER_DATA_LEN;
        whisper_packet[56] = whisper_packet[51] + 1;

        // packet 13
        whisper_packet[57] = (uint16_t) 0x00;
        whisper_packet[59] = 0xA5;
        whisper_packet[60] = WHISPER_DATA_LEN;
        whisper_packet[61] = whisper_packet[56] + 1;

        // packet 14
        whisper_packet[62] = (uint16_t) 0x00;
        whisper_packet[64] = 0xA5;
        whisper_packet[65] = WHISPER_DATA_LEN;
        whisper_packet[66] = whisper_packet[61] + 1;

        // packet 15
        whisper_packet[67] = (uint16_t) 0x00;
        whisper_packet[69] = 0xA5;
        whisper_packet[70] = WHISPER_DATA_LEN;
        whisper_packet[71] = whisper_packet[66] + 1;

        // packet 16
        whisper_packet[72] = (uint16_t) 0x00;
        whisper_packet[74] = 0xA5;
        whisper_packet[75] = WHISPER_DATA_LEN;
        whisper_packet[76] = whisper_packet[71] + 1;

        // packet 17
        whisper_packet[77] = (uint16_t) 0x00;
        whisper_packet[79] = 0xA5;
        whisper_packet[80] = WHISPER_DATA_LEN;
        whisper_packet[81] = whisper_packet[76] + 1;

        // packet 18
        whisper_packet[82] = (uint16_t) 0x00;
        whisper_packet[84] = 0xA5;
        whisper_packet[85] = WHISPER_DATA_LEN;
        whisper_packet[86] = whisper_packet[81] + 1;
        
        // packet 19
        whisper_packet[87] = (uint16_t) 0x00;
        whisper_packet[89] = 0xA5;
        whisper_packet[90] = WHISPER_DATA_LEN;
        whisper_packet[91] = whisper_packet[86] + 1;
        
        // packet 20
        whisper_packet[92] = (uint16_t) 0x00;
        whisper_packet[94] = 0xA5;
        whisper_packet[95] = WHISPER_DATA_LEN;
        whisper_packet[96] = whisper_packet[91] + 1;
      
        // packet 21
        whisper_packet[97] = (uint16_t) 0x00;
        whisper_packet[99] = 0xA5;
        whisper_packet[100] = WHISPER_DATA_LEN;
        whisper_packet[101] = whisper_packet[96] + 1;
        
        // packet 22
        whisper_packet[102] = (uint16_t) 0x00;
        whisper_packet[104] = 0xA5;
        whisper_packet[105] = WHISPER_DATA_LEN;
        whisper_packet[106] = whisper_packet[101] + 1;
        
        // packet 23
        whisper_packet[107] = (uint16_t) 0x00;
        whisper_packet[109] = 0xA5;
        whisper_packet[110] = WHISPER_DATA_LEN;
        whisper_packet[111] = whisper_packet[107] + 1;
        
        // packet 24
        whisper_packet[112] = (uint16_t) 0x00;
        whisper_packet[114] = 0xA5;
        whisper_packet[115] = WHISPER_DATA_LEN;
        whisper_packet[116] = whisper_packet[111] + 1;
        
        // packet 25
        whisper_packet[117] = (uint16_t) 0x00;
        whisper_packet[119] = 0xA5;
        whisper_packet[120] = WHISPER_DATA_LEN;
        whisper_packet[121] = whisper_packet[116] + 1;

        return len;
    } else {
        return 0;
    }
}

/* --------------------------- Main interface ----------------------- */
void whisper_start(uint8_t initiator_, uint8_t tx_packet_len, uint8_t tx_max_,
        rtimer_clock_t t_stop_,  rtimer_clock_t t_start_, uint8_t sync_, rtimer_callback_t cb_,
        struct rtimer *rtimer_, void *ptr_) {

    // copy function arguments to the respective Whisper variables
    whisper_initiator = initiator_;
    whisper_tx_max = tx_max_;
    whisper_t_stop = t_stop_;
    whisper_t_start = t_start_;
    whisper_sync = sync_;
    whisper_cb = cb_;
    whisper_rtimer = rtimer_;
    whisper_ptr = ptr_;
    // the reference time has not been updated yet
    t_ref_l_updated = 0;
    // disable all interrupts that may interfere with Whisper
    whisper_disable_other_interrupts();
    // initialize Whisper variables
    whisper_tx_cnt = 0;
    whisper_cca_valid = 0;
    whisper_cur_countdown = -1;
    whisper_got_packet = 0;
    whisper_t_rx_start = 0;
    whisper_t_rx_stop = 0;
    countdown_bak = 0;
#if WHISPER_DIRECTION_AWARE_SAMPLING
    whisper_avg_counter_index = 0;
#endif
#if !COOJA
    // resynchronize the DCO
    msp430_sync_dco();
#endif /* COOJA */
    // flush radio buffers
    radio_flush_rx();
    radio_flush_tx();

    // Set the preamble length
#if WHISPER_PREAMBLE_LENGTH == 4
    cc2420_set_preamble_length(4);
#elif WHISPER_PREAMBLE_LENGTH == 2
    cc2420_set_preamble_length(2);
#else
    #error [ERROR:] Strange preamble length
#endif
    
    if (whisper_initiator) {
        // set Whisper state
        whisper_state = WHISPER_STATE_TRANSMITTED;
           
#if WHISPER_DIRECTION_AWARE_SAMPLING && WHISPER_DIRECTION_MODE == 2 // Collection
        if (!WHISPER_COLLECTION_IS_BOOTSTRAPPING()) {
            // Send as the node where in the next hop
            // TODO: Also do for other NTX and preamble length
            whisper_avg_start_t[whisper_avg_counter_index] = whisper_t_start + whisper_avg_start[whisper_avg_counter_index] + (2 * WHISPER_PACKET_LENGTH);
            whisper_t_start = whisper_avg_start_t[whisper_avg_counter_index];
            whisper_avg_counter_index++;
        } else {
            // We are bootstrapping
            whisper_cur_countdown = 0;
            whisper_packet_len_tmp = tx_packet_len;
            whisper_packet_len = whisper_packet_len_tmp;
            whisper_cca_valid = 1;
            whisper_offset = 0;

#if WHISPER_PREAMBLE_LENGTH == 4
            whisper_tx_cnt = whisper_packet[118] & (~LANEFLOOD_TYPE);
#elif WHISPER_PREAMBLE_LENGTH == 2
#if WHISPER_PACKET_LENGTH == 7
            whisper_tx_cnt = whisper_packet[120] & (~LANEFLOOD_TYPE); //packet18
#endif  /* WHISPER_PACKET_LENGTH */
#else /* WHISPER_PREAMBLE_LENGTH */
            #error [ERROR:] Strange preamble length
#endif  /* WHISPER_PREAMBLE_LENGTH */
        }
             
#else /* WHISPER_DIRECTION_MODE */
       
        whisper_packet_len_tmp = tx_packet_len;
        whisper_packet_len = whisper_packet_len_tmp;
#if WHISPER_PREAMBLE_LENGTH == 4
        whisper_tx_cnt = whisper_packet[118] & (~LANEFLOOD_TYPE);
#elif WHISPER_PREAMBLE_LENGTH == 2
#if WHISPER_PACKET_LENGTH == 7
        whisper_tx_cnt = whisper_packet[120] & (~LANEFLOOD_TYPE); //packet18
        //whisper_tx_cnt = whisper_packet[113] & (~LANEFLOOD_TYPE); //packet 17
        //whisper_tx_cnt = whisper_packet[106] & (~LANEFLOOD_TYPE); //packet 16
        //whisper_tx_cnt = whisper_packet[99] & (~LANEFLOOD_TYPE); //packet15
        //whisper_tx_cnt = whisper_packet[92] & (~LANEFLOOD_TYPE); //packet14
        //whisper_tx_cnt = whisper_packet[85] & (~LANEFLOOD_TYPE); //packet13
        //whisper_tx_cnt = whisper_packet[78] & (~LANEFLOOD_TYPE); //packet12
        //whisper_tx_cnt = whisper_packet[71] & (~LANEFLOOD_TYPE); //packet11
#endif       
#else /* WHISPER_PREAMBLE_LENGTH */
        printf("[ERROR:] Strange preamble length \n");
#endif /* WHISPER_PREAMBLE_LENGTH */
#endif
        // ------------------------ initiator: write to radio ------------------------ //
#if WHISPER_DIRECTION_AWARE_SAMPLING
#if WHISPER_LONG_WAKEUP_PACKET
        whisper_radio_write_tx(whisper_offset);
#elif WHISPER_NTX
        FASTSPI_WRITE_FIFO(whisper_packet + whisper_offset, ((WHISPER_NTX + 2) * WHISPER_PACKET_LENGTH) - 1);
#endif
#else /* !WHISPER_DIRECTION_AWARE_SAMPLING */
#if WHISPER_LONG_WAKEUP_PACKET
        whisper_radio_write_tx(0);
#elif WHISPER_NTX
        FASTSPI_WRITE_FIFO(whisper_packet, ((WHISPER_NTX + 2) * WHISPER_PACKET_LENGTH) - 1);
#endif /* WHISPER_LONG_WAKEUP_PACKET */
#endif /* WHISPER_DIRECTION_AWARE_SAMPLING */
    } else { // ----------- receivers ----------- //
        whisper_packet_len = 0;
        whisper_state = WHISPER_STATE_WAITING;
#if WHISPER_DIRECTION_AWARE_SAMPLING
#if WHISPER_DIRECTION_MODE == 1
        // calculate the time for p_min
        if (whisper_avg_start[whisper_avg_counter_index] >= WHISPER_PACKET_LENGTH) {
            // Wake up one packlet earlier
            whisper_avg_start_t[whisper_avg_counter_index] = whisper_t_start + (whisper_avg_start[whisper_avg_counter_index] - WHISPER_PACKET_LENGTH);
            whisper_t_start = whisper_avg_start_t[whisper_avg_counter_index];
        } else {
            whisper_avg_start_t[whisper_avg_counter_index] = whisper_t_start + whisper_avg_start[whisper_avg_counter_index];
            whisper_t_start = whisper_avg_start_t[whisper_avg_counter_index];
        }
        // calculate the time for p_max
        if (whisper_avg_counter_len > 1) {
            whisper_avg_start_t[whisper_avg_counter_len - 1] =  whisper_t_start + whisper_avg_start[whisper_avg_counter_len - 1];
        }
        whisper_avg_counter_index++;
#elif WHISPER_DIRECTION_MODE == 2
        if (!WHISPER_COLLECTION_IS_BOOTSTRAPPING()) {
            if (whisper_avg_start[whisper_avg_counter_index] >= WHISPER_PACKET_LENGTH) {
                // Wake up one packlet earlier
                whisper_avg_start_t[whisper_avg_counter_index] = whisper_t_start + (whisper_avg_start[whisper_avg_counter_index] - WHISPER_PACKET_LENGTH);
                whisper_t_start = whisper_avg_start_t[whisper_avg_counter_index];
            } else {
                whisper_avg_start_t[whisper_avg_counter_index] = whisper_t_start + whisper_avg_start[whisper_avg_counter_index];
                whisper_t_start = whisper_avg_start_t[whisper_avg_counter_index];
            }
            // calculate the time for p_max
            if (whisper_avg_counter_len > 1) {
                whisper_avg_start_t[whisper_avg_counter_len - 1] = whisper_t_start + whisper_avg_start[whisper_avg_counter_len - 1];
            }
            whisper_avg_counter_index++;
        }
#endif /* WHISPER_DIRECTION_MODE */
#endif /* WHISPER_DIRECTION_AWARE_SAMPLING */
    }
    // ------------------------ now the radio ------------------------ //
#if CC2420_ENABLE_TESTMODE
        whisper_set_tx_mode_looping();
#endif /* CC2420_ENABLE_TESTMODE */
    // Wait with turning on the radio
    while(RTIMER_CLOCK_LT(RTIMER_NOW(), whisper_t_start));
    // Ok, now start
    if (whisper_initiator) {
         // write the packet to the TXFIFO
        whisper_radio_start_tx();
        whisper_cca_valid = 1;
    } else {
        radio_on();
#if MEASURE_IDLE_LISTENING
       whisper_t_radio_start_l = RTIMER_NOW();
#endif /* MEASURE_IDLE_LISTENING */
    }
#if LANEFLOOD_DEBUG_PINS
#if COOJA || LOCAL_NODES
        leds_on(LEDS_RED);
#endif /* COOJA */
#if FLOCKLAB || LOCAL_NODES
        SET_PIN_ADC7;
#endif /* FLOCKLAB */
#endif /* LANEFLOOD_DEBUG_PINS */
    // ------------------------ activate busy waiting ------------------------ //
    // activate the whisper busy waiting process
    process_poll(&whisper_process);
}

uint8_t whisper_stop(void) {
    whisper_tbiv = TBIV;
    // turn off the radio
    radio_off();
#if LANEFLOOD_DEBUG_PINS
#if COOJA || LOCAL_NODES
    leds_off(LEDS_RED);
    leds_off(LEDS_BLUE);
    leds_off(LEDS_GREEN);
#endif /* COOJA */
#if FLOCKLAB || LOCAL_NODES
    UNSET_PIN_ADC2;
    UNSET_PIN_ADC7;
    UNSET_PIN_ADC6;
#endif /* FLOCKLAB */
#endif /* LANEFLOOD_DEBUG_PINS */
#if MEASURE_IDLE_LISTENING
    whisper_t_radio_stop_l = RTIMER_NOW();
#endif /* MEASURE_IDLE_LISTENING */
    
    //whisper_set_default_syncword();
    //cc2420_set_cca_mode(3);
    cc2420_set_preamble_length(4);
    //cc2420_set_tx_power(CC2420_DEFAULT_TX_POWER);
#if CC2420_ENABLE_TESTMODE
    whisper_set_tx_mode(0);
    cc2420_set_autocrc(1);
#endif /* CC2420_ENABLE_TESTMODE */
    
    // flush radio whisper_packets
    radio_flush_rx();
    radio_flush_tx();

    whisper_state = WHISPER_STATE_OFF;
    // re-enable non Whisper-related interrupts
    whisper_enable_other_interrupts();
    // return the number of times the packet has been received
    return 1;
}

uint8_t get_whisper_tx_cnt(void) {
    return whisper_tx_cnt;
}

uint8_t get_whisper_packet_len(void) {
    return whisper_packet_len_tmp;
}

uint8_t *get_whisper_packet(void) {
    return whisper_packet_in;
}

uint8_t get_whisper_state(void) {
    return whisper_state;
}

uint16_t get_whisper_transmitted(void) {
    return whisper_transmitted;
}

uint8_t get_whisper_cca_valid(void) {
    return whisper_cca_valid;
}

int8_t get_whisper_cur_countdown(void) {
    return whisper_cur_countdown;
}

uint16_t get_whisper_T_rx(void) {
    return (unsigned int) (whisper_t_rx_stop - whisper_t_rx_start);
}

uint16_t get_whisper_delta_sfd_wait(void) {
    return (unsigned int) (whisper_t_tx_start - whisper_t_rx_stop);
}

uint16_t get_whisper_delta_sfd_tx(void) {
    return (unsigned int) (whisper_t_tx_stop - whisper_t_tx_start);
}

uint8_t get_countdown_bak(void) {
    return countdown_bak;
}

rtimer_clock_t get_T_ref_l(void) {
    return T_ref_l;
}

#if MEASURE_IDLE_LISTENING
uint16_t get_whisper_active(void) {
        return (whisper_t_radio_stop_l - whisper_t_rx_start_l);
}
uint16_t get_whisper_passive(void) {
        return (whisper_t_rx_start_l - whisper_t_radio_start_l);
}
#endif /* MEASURE_IDLE_LISTENING */

uint8_t get_whisper_T_irq(void) {
    return whisper_T_irq;
}

uint8_t get_whisper_nops_to_add(void) {
    return whisper_nops_to_add;
}

/* ----------------------- Whisper interrupt functions ---------------------- */
inline void whisper_begin_rx(void) {
    whisper_got_packet = 1;
    whisper_t_rx_start = TBCCR1;
#if MEASURE_IDLE_LISTENING
    whisper_t_rx_start_l = RTIMER_NOW();
#endif /* MEASURE_IDLE_LISTENING */
#if LANEFLOOD_DEBUG_PINS
#if COOJA || LOCAL_NODES
    leds_on(LEDS_GREEN);
#endif /* COOJA || LOCAL_NODES */
#if FLOCKLAB || LOCAL_NODES
    SET_PIN_ADC6;
#endif /* FLOCKLAB */
#endif /* LANEFLOOD_DEBUG_PINS */
    whisper_state = WHISPER_STATE_RECEIVING;
    // Rx timeout: packet duration + 200 us
    // (packet duration: 32 us * packet_length, 1 DCO tick ~ 0.23 us)
    // The packet is 4 byte long...
    whisper_t_rx_timeout = whisper_t_rx_start + (4 * 35 + 200) * 4;
    // wait until the FIFO pin is 1 (i.e., until the first byte is received)
    while (!FIFO_IS_1) {
        if (!RTIMER_CLOCK_LT(RTIMER_NOW_DCO(), whisper_t_rx_timeout)) {
            radio_abort_rx_whisper();
#if LANEFLOOD_DEBUG_PINS
#if COOJA || LOCAL_NODES
            leds_off(LEDS_GREEN);
#endif /* COOJA */
#if FLOCKLAB || LOCAL_NODES
            UNSET_PIN_ADC6;
#endif /* FLOCKLAB */
#endif /* LANEFLOOD_DEBUG_PINS */
#if LANEFLOOD_DEBUG
            whisper_rx_timeout++;
#endif /* LANEFLOOD_DEBUG */
                return;
        }
    };
    // read the first byte (i.e., the len field) from the RXFIFO
    FASTSPI_READ_FIFO_BYTE(lf_header_whisper->whisper_len_field);
    // keep receiving only if it has the right length
    if (lf_header_whisper->whisper_len_field != (WHISPER_DATA_LEN + FOOTER_LEN)) {
        // packet with a wrong length: abort packet reception
        // if packet > 3, it might be the first packet and we have to ignore it
        radio_abort_rx_whisper();
#if LANEFLOOD_DEBUG_PINS
#if COOJA || LOCAL_NODES
        leds_off(LEDS_GREEN);
#endif /* COOJA */
#if FLOCKLAB || LOCAL_NODES
        UNSET_PIN_ADC6;
#endif /* FLOCKLAB */
#endif /* LANEFLOOD_DEBUG_PINS */
#if LANEFLOOD_DEBUG
        whisper_bad_length++;
#endif /* LANEFLOOD_DEBUG */
        return;
    }
    whisper_bytes_read = 1;
    whisper_packet_len_tmp = lf_header_whisper->whisper_len_field;
    whisper_schedule_rx_timeout();
    // read the first byte (i.e., the len field) from the RXFIFO
    FASTSPI_READ_FIFO_BYTE(lf_header_whisper->whisper_data);
    whisper_bytes_read++;
}

inline void whisper_end_rx(void) {
    whisper_t_rx_stop = TBCCR1;
    // read the remaining bytes from the RXFIFO 
    FASTSPI_READ_FIFO_NO_WAIT(&whisper_packet_in[whisper_bytes_read], whisper_packet_len_tmp - whisper_bytes_read + 1);
    
    if ((WHISPER_CRC_FIELD & FOOTER1_CRC_OK)
            && ((lf_header_whisper->whisper_data & LANEFLOOD_TYPE) == LANEFLOOD_TYPE)) {

        // packet correctly received
        whisper_tx_cnt = lf_header_whisper->whisper_data & (~LANEFLOOD_TYPE);
        whisper_cur_countdown = (int8_t) whisper_tx_cnt;
        if ((whisper_tx_cnt + 2) >= whisper_tx_max) {
            radio_abort_tx();
            radio_off();
#if LANEFLOOD_DEBUG_PINS
#if COOJA || LOCAL_NODES
            leds_off(LEDS_RED);
#endif /* COOJA */
#if FLOCKLAB || LOCAL_NODES
            UNSET_PIN_ADC7;
#endif /* FLOCKLAB */
#endif /* LANEFLOOD_DEBUG_PINS */
            whisper_state = WHISPER_STATE_OFF;
        } else {
#if WHISPER_PREAMBLE_LENGTH == 4
            whisper_offset = (WHISPER_PACKET_LENGTH * (whisper_tx_cnt + 2));
#if !CC2420_ENABLE_TESTMODE
            whisper_packet_len_tmp = 122 - whisper_offset;
#else
#if WHISPER_LONG_WAKEUP_PACKET
            whisper_packet_len_tmp = 120 - whisper_offset;
#else
            whisper_packet_len_tmp = WHISPER_PACKET_LENGTH * (WHISPER_NTX + 2);
#endif            
#endif            
            whisper_tx_cnt = whisper_packet[118] & (~LANEFLOOD_TYPE);
#elif WHISPER_PREAMBLE_LENGTH == 2
#if WHISPER_PACKET_LENGTH == 7
            whisper_offset = (WHISPER_PACKET_LENGTH * (whisper_tx_cnt + 2));
#if !CC2420_ENABLE_TESTMODE
            whisper_packet_len_tmp = 124 - whisper_offset;
#else
#if WHISPER_LONG_WAKEUP_PACKET
            whisper_packet_len_tmp = 122 - whisper_offset;
#else
            whisper_packet_len_tmp = WHISPER_PACKET_LENGTH * (WHISPER_NTX + 2);
#endif            
#endif 
            whisper_tx_cnt = whisper_packet[120] & (~LANEFLOOD_TYPE);
#endif
#else
            printf("[ERROR:] Strange preamble length %u\n", cc2420_get_preamble_length());
#endif
#if !CC2420_ENABLE_TESTMODE
            whisper_packet[whisper_offset] = whisper_packet_len_tmp;
#else
            whisper_packet[whisper_offset] = WHISPER_DATA_LEN + FOOTER_LEN;
#endif
            whisper_bytes_wrote = 0;
#if !WHISPER_LONG_WAKEUP_PACKET
            FASTSPI_WRITE_FIFO_START_PART(whisper_packet + whisper_offset, WHISPER_PACKET_LENGTH - 1, whisper_bytes_wrote);
#else
            whisper_radio_write_tx(whisper_offset);
#endif
            whisper_state = WHISPER_STATE_TRANSMITTED;
            whisper_packet_len = whisper_packet_len_tmp;
        }
    } else {
        // packet corrupted, abort the transmission before it actually starts
        radio_abort_tx();
        whisper_state = WHISPER_STATE_WAITING;
#if LANEFLOOD_DEBUG
        whisper_bad_crc++;
#endif /* LANEFLOOD_DEBUG */
    }
#if LANEFLOOD_DEBUG_PINS
#if COOJA || LOCAL_NODES
    leds_off(LEDS_GREEN);
#endif /* COOJA */
#if FLOCKLAB || LOCAL_NODES
    UNSET_PIN_ADC6;
#endif /* FLOCKLAB */
#endif /* LANEFLOOD_DEBUG_PINS */
}

inline void whisper_begin_tx(void) {
    whisper_t_tx_start = TBCCR1;
#if LANEFLOOD_DEBUG_PINS
#if COOJA || LOCAL_NODES
    leds_on(LEDS_BLUE);
#endif /* COOJA */
#if FLOCKLAB || LOCAL_NODES
    SET_PIN_ADC2;
#endif /* FLOCKLAB */
#endif /* LANEFLOOD_DEBUG_PINS */
#if COOJA
    whisper_t_cap_l = RTIMER_NOW();
    whisper_t_cap_h = RTIMER_NOW_DCO();
#else
    // capture the next low-frequency clock tick
    CAPTURE_NEXT_CLOCK_TICK(whisper_t_cap_h, whisper_t_cap_l);
#endif /* COOJA */
    whisper_state = WHISPER_STATE_TRANSMITTING;
#if CC2420_ENABLE_TESTMODE 
#if !WHISPER_LONG_WAKEUP_PACKET
#if WHISPER_PACKET_LENGTH == 7
#if WHISPER_NTX == 3
        // Transmit 3 packlets before turning the radio off
        whisper_t_cancle = whisper_t_cap_l + 23;
#elif WHISPER_NTX == 2
        whisper_t_cancle = whisper_t_cap_l + 15;       
#endif
#elif WHISPER_PACKET_LENGTH == 9
#if WHISPER_NTX == 5
        whisper_t_cancle = whisper_t_cap_l + 48;
#elif WHISPER_NTX == 4
        whisper_t_cancle = whisper_t_cap_l + 38;
#elif WHISPER_NTX == 3
        whisper_t_cancle = whisper_t_cap_l + 29;
#elif WHISPER_NTX == 2
        whisper_t_cancle = whisper_t_cap_l + 19;
#elif WHISPER_NTX == 1
        whisper_t_cancle = whisper_t_cap_l + 10;
#endif /* WHISPER_NTX */
#endif /* WHISPER_PACKET_LENGTH */
#endif /* !WHISPER_LONG_WAKEUP_PACKET */
#endif /*  CC2420_ENABLE_TESTMODE */
#if !WHISPER_LONG_WAKEUP_PACKET
    if (!whisper_initiator) {
        whisper_offset += whisper_bytes_wrote;
        if (whisper_packet_len_tmp != whisper_bytes_wrote) {
            FASTSPI_WRITE_FIFO_STOP_PART(whisper_packet + whisper_offset, (whisper_packet_len_tmp - whisper_bytes_wrote - 1), whisper_bytes_wrote);
        }
    } 
#endif
#if !WITH_GLOSSY
        if (!whisper_initiator && whisper_sync && whisper_cur_countdown >= 0) {
            whisper_compute_sync_reference_time();
        }
#endif
}

inline void whisper_end_tx(void) {
    if (energest_on) {
        ENERGEST_OFF(ENERGEST_TYPE_TRANSMIT);
        ENERGEST_ON(ENERGEST_TYPE_LISTEN);
    }
    // stop Whisper if tx_cnt reached tx_max
    if ((whisper_tx_cnt + 1) >= whisper_tx_max) {
        radio_off();        
#if LANEFLOOD_DEBUG_PINS
#if COOJA || LOCAL_NODES
        leds_off(LEDS_RED);
        leds_off(LEDS_BLUE);
#endif /*COOJA*/
#if FLOCKLAB || LOCAL_NODES
        UNSET_PIN_ADC7;
        UNSET_PIN_ADC2;
#endif /* FLOCKLAB */
#endif /* LANEFLOOD_DEBUG_PINS */
        whisper_state = WHISPER_STATE_OFF;
    } else {
        whisper_state = WHISPER_STATE_TRANSMITTED;
    }
#if LANEFLOOD_DEBUG_PINS
#if COOJA || LOCAL_NODES
    leds_off(LEDS_BLUE);
#endif /* COOJA */
#if FLOCKLAB || LOCAL_NODES
    UNSET_PIN_ADC2;
#endif /* FLOCKLAB */
#endif /* LANEFLOOD_DEBUG_PINS */
}

/* ------------------------------ Timeouts -------------------------- */
inline void whisper_schedule_rx_timeout(void) {
    TBCCR5 = whisper_t_rx_timeout;
    TBCCTL5 = CCIE;
}

inline void whisper_stop_rx_timeout(void) {
    TBCCTL5 = 0;
}

