/**
 * \addtogroup rt
 * @{
 */

/**
 * \file
 *         Implementation of the architecture-agnostic parts of the real-time timer module.
 * \author
 *         Adam Dunkels <adam@sics.se>
 *
 */


/*
 * Copyright (c) 2005, Swedish Institute of Computer Science
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
 * This file is part of the Contiki operating system.
 *
 * @(#)$Id: rtimer.c,v 1.7 2010/01/19 13:08:24 adamdunkels Exp $
 */

#include "sys/rtimer.h"
#include "contiki.h"
#include "dev/leds.h"

#define DEBUG	0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

static struct rtimer *next_rtimer;
//static struct rtimer32 *next_rtimer32;

/*---------------------------------------------------------------------------*/
void
rtimer_init(void) {
	rtimer_arch_init();
}
/*---------------------------------------------------------------------------*/
//int
//rtimer32_set(struct rtimer32 *rtimer32_, rtimer32_clock_t time_,
//		rtimer_clock_t duration,
//		rtimer_callback_t func, void *ptr) {
//	int first = 0;
//
//	PRINTF("rtimer32_set time %lu, now %lu, cb %p \n", time_, RTIMER32_NOW(), func);
//
//	if(next_rtimer32 == NULL) {
//		first = 1;
//	}
//
//	rtimer32_->func = func;
//	rtimer32_->ptr = ptr;
//
//	rtimer32_->time_upper = (uint16_t) (time_ >> 16);
//	rtimer32_->time_lower = (uint16_t) time_;
//
//	next_rtimer32 = rtimer32_;
//
//	if(first == 1) {
//		rtimer32_arch_schedule(time_);
//	}
//	return RTIMER_OK;
//}
/*---------------------------------------------------------------------------*/
//void
//rtimer32_run_next(void) {
//	struct rtimer32 *t;
//	if(next_rtimer32 == NULL) {
//		return;
//	}
//	t = next_rtimer32;
//	next_rtimer32 = NULL;
//	t->func(t, t->ptr);
//	if(next_rtimer != NULL) {
//		rtimer32_arch_schedule((((rtimer32_clock_t) next_rtimer32->time_upper) << 16) | next_rtimer32->time_lower);
//	}
//	return;
//}
/*---------------------------------------------------------------------------*/
int
rtimer_set(struct rtimer *rtimer, rtimer_clock_t time,
	   rtimer_clock_t duration,
	   rtimer_callback_t func, void *ptr)
{
	int first = 0;

	PRINTF("rtimer_set time %u, now %u, cb %p \n", time, RTIMER_NOW(), func);
	//PRINTF("rtimer_set time %lu, now %lu, cb %p \n", time, RTIMER_NOW(), func);

	if(next_rtimer == NULL) {
		first = 1;
	}

	rtimer->func = func;
	rtimer->ptr = ptr;

	rtimer->time = time;
	next_rtimer = rtimer;

	if(first == 1) {
		rtimer_arch_schedule(time);
	}
	return RTIMER_OK;
}
/*---------------------------------------------------------------------------*/
void
rtimer_run_next(void)
{
	struct rtimer *t;
	if(next_rtimer == NULL) {
		return;
	}
	t = next_rtimer;
	next_rtimer = NULL;
	t->func(t, t->ptr);
	if(next_rtimer != NULL) {
		rtimer_arch_schedule(next_rtimer->time);
	}
	return;
}
/*---------------------------------------------------------------------------*/
