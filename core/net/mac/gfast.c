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

/**
 * \file
 *         Redirects the interrupt either to Glossy or to Whisper.
 * \author
 *         Martina Brachmann <martina.brachmann@tu-dresden.de>
 */



#include "gfast.h"

static uint8_t modus;

const whisper_action whisper_interrupt_table[2] =
{
    whisper_interrupt,
    glossy_interrupt
};

/* --------------------------- SFD interrupt ------------------------ */
interrupt(TIMERB1_VECTOR) __attribute__((section(".gfast")))
timerb1_interrupt(void) {
    whisper_interrupt_table[modus]();
}

uint8_t get_gfast_modus(void) {
    return modus;
}

void set_gfast_modus(uint8_t modus_) {
    modus = modus_;
}

uint8_t is_t_ref_l_updated(void) {
    return t_ref_l_updated;
}

rtimer_clock_t get_t_ref_l(void) {
    return t_ref_l;
}

void set_t_ref_l(rtimer_clock_t t) {
    t_ref_l = t;
}

void reset_t_ref_l_updated(void) {
    t_ref_l_updated = 0;
}

void set_t_ref_l_updated(uint8_t updated) {
    t_ref_l_updated = updated;
}
