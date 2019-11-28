/*
 * Copyright (c) 2007, Swedish Institute of Computer Science
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
 * @(#)$Id: cc2420.c,v 1.51 2010/04/08 18:23:24 adamdunkels Exp $
 */
/*
 * This code is almost device independent and should be easy to port.
 */

#include "dev/cc2420.h"

#include <stdio.h>
#include <string.h>

#include "../net/mac/whisper/whisper.h"
#include "contiki.h"

#include "dev/leds.h"
#include "dev/spi.h"

#include "dev/cc2420_const.h"
#include "glossy.h"

/*---------------------------------------------------------------------------*/
#define AUTOACK                 (1 << 4)
#define ADR_DECODE              (1 << 11)
#define RXFIFO_PROTECTION       (1 << 9)
#define CORR_THR(n)             (((n) & 0x1f) << 6)
#define FIFOP_THR(n)            ((n) & 0x7f)
#define RXBPF_LOCUR             (1 << 13);
#define CCA_MODE(reg, n)        ((reg & 0xff3f) | (((n) & 0x3) << 6))
#define TX_TURNAROUND           (1 << 13)
#define CCA_THR(reg, n)         ((reg & 0x00ff) | ((((n) + 45) & 0xff) << 8))
#define AUTOCRC                 (1 << 5)
#define PREAMBLE_LENGTH(reg, n)      ((reg & 0xfff0) | ((n) & 0xf))
/*---------------------------------------------------------------------------*/
uint8_t
cc2420_init(void) {
    uint16_t reg;
    {
        int s = splhigh();
        spi_init();

        /* all input by default, set these as output */
        P4DIR |= BV(CSN) | BV(VREG_EN) | BV(RESET_N);

        SPI_DISABLE(); /* Unselect radio. */
        DISABLE_FIFOP_INT();
        FIFOP_INT_INIT();
        splx(s);
    }

    /* Turn on voltage regulator and reset. */
    SET_VREG_ACTIVE();
    SET_RESET_ACTIVE();
    clock_delay(127);
    SET_RESET_INACTIVE();

    /* Turn on the crystal oscillator. */
    FASTSPI_STROBE(CC2420_SXOSCON);

    /* Change CCA mode to 1 (clear channel when RSSI below certain threshold)*/
    //cc2420_set_cca_mode(1);
    /* Turn off automatic packet acknowledgment and address decoding. */
    FASTSPI_GETREG(CC2420_MDMCTRL0, reg);
    reg &= ~(AUTOACK | ADR_DECODE);
    FASTSPI_SETREG(CC2420_MDMCTRL0, reg);

    /* Change default values as recomended in the data sheet, */
    /* correlation threshold = 20, RX bandpass filter = 1.3uA. */
    FASTSPI_SETREG(CC2420_MDMCTRL1, CORR_THR(20));
    FASTSPI_GETREG(CC2420_RXCTRL1, reg);
    reg |= RXBPF_LOCUR;
    FASTSPI_SETREG(CC2420_RXCTRL1, reg);

    /* Set the FIFOP threshold to maximum. */
    FASTSPI_SETREG(CC2420_IOCFG0, FIFOP_THR(127));

    /* Turn off "Security enable" (page 32). */
    FASTSPI_GETREG(CC2420_SECCTRL0, reg);
    reg &= ~RXFIFO_PROTECTION;
    FASTSPI_SETREG(CC2420_SECCTRL0, reg);
      
//    /* Change TX power */
//    FASTSPI_GETREG(CC2420_TXCTRL, reg);
//    reg &= (0xFFE0);
//    //reg |= (0x1F & 0xFF);		// 0dBm     31
//    //reg |= (0x1F & 0xF7);		// -3dBm
//    //reg |= (0x1F & 0xEF);		// -7dBm
//    //reg |= (0x1F & 0xEB);             // -10dBm  11
//    //reg |= (0x1F & 0xE7);		// -15dBm
//    reg |= (0x1F & 0xE3);		// -25dBm   3
//    //reg |= (0x1F & 0xE2);		// -XXdBm   2
//    //reg |= (0x1F & 0xE1);		// -XXdBm   1
//    reg |= (0x1F & 0xE0);		// -XXdBm   0
//    FASTSPI_SETREG(CC2420_TXCTRL, reg);
    cc2420_set_tx_power(CC2420_DEFAULT_TX_POWER);

    radio_flush_rx();
    return 1;
}
/*---------------------------------------------------------------------------*/
void
cc2420_set_channel(uint8_t c)
{
  uint16_t f;

  /*
   * Subtract the base channel (11), multiply by 5, which is the
   * channel spacing. 357 is 2405-2048 and 0x4000 is LOCK_THR = 1.
   */

  //f = 5 * (c - 11) + 357 + 0x4000;
  //f = 5 * (23 - 11) + 357 + 0x4000;
  //f = 5 * (27 - 11) + 357 + 0x4000;
  f = 434 + 0x4000;
  /*
   * Writing RAM requires crystal oscillator to be stable.
   */
  while(!(radio_status() & (BV(CC2420_XOSC16M_STABLE))));

  /* Wait for any transmission to end. */
  while(radio_status() & BV(CC2420_TX_ACTIVE));

  FASTSPI_SETREG(CC2420_FSCTRL, f);
}

void
cc2420_set_tx_turnaround(uint8_t t) {
    uint16_t reg;
    FASTSPI_GETREG(CC2420_TXCTRL, reg);
    if (t) {
        reg |= TX_TURNAROUND;
    } else {
        reg &= ~TX_TURNAROUND;
    }
    FASTSPI_SETREG(CC2420_TXCTRL, reg);
}

void
cc2420_set_cca_mode(uint8_t m) {
    uint16_t reg;
    FASTSPI_GETREG(CC2420_MDMCTRL0, reg);
    FASTSPI_SETREG(CC2420_MDMCTRL0, CCA_MODE(reg, m));
}

int8_t
cc2420_get_rssi(void) {
    uint16_t reg;
    FASTSPI_GETREG(CC2420_RSSI, reg);
    return (int8_t) (0xff & reg);
}

int8_t
cc2420_get_cca_thr(void) {
    uint16_t reg;
    FASTSPI_GETREG(CC2420_RSSI, reg);
    return (int8_t) ((0xff00 & reg) >> 8);
}

void
cc2420_set_cca_thr(int8_t t) {
    uint16_t reg;
    FASTSPI_GETREG(CC2420_RSSI, reg);
    FASTSPI_SETREG(CC2420_RSSI, CCA_THR(reg, t));
}

void
cc2420_set_autocrc(uint8_t a) {
    uint16_t reg;
    FASTSPI_GETREG(CC2420_MDMCTRL0, reg);
    if (a) {
        reg |= AUTOCRC; 
    } else {
        reg &= ~AUTOCRC;
    }
    FASTSPI_SETREG(CC2420_MDMCTRL0, reg);
}

int16_t
cc2420_get_syncword(void) {
    uint16_t reg;
    FASTSPI_GETREG(CC2420_SYNCWORD, reg);
    return reg;
}

void
cc2420_set_syncword(uint16_t syncword) {
    FASTSPI_SETREG(CC2420_SYNCWORD, syncword);
}

void
cc2420_set_default_syncword(void) {
    FASTSPI_SETREG(CC2420_SYNCWORD, 0xA70F);
}

uint8_t
cc2420_get_preamble_length(void) {
    uint16_t reg;
    FASTSPI_GETREG(CC2420_MDMCTRL0, reg);
    return (int8_t) (0x000f & reg);
}

int8_t
cc2420_set_preamble_length(uint8_t l) {
    uint16_t reg;
    FASTSPI_GETREG(CC2420_MDMCTRL0, reg);
    if (l <= 1) {
        printf("Preamble must be greater than 1 as it requires one leading zero + one zero from syncword \n");
        return -1;
    } else {
        l = l - 2;
    }
    FASTSPI_SETREG(CC2420_MDMCTRL0, PREAMBLE_LENGTH(reg, l));
    return l;
}

void
cc2420_set_tx_power(uint8_t power) {
    uint16_t reg;
    FASTSPI_GETREG(CC2420_TXCTRL, reg);
    reg = (reg & 0xffe0) | (power & 0x1f);
    FASTSPI_SETREG(CC2420_TXCTRL, reg);
}
