/*
 * Copyright (c) 2004, Adam Dunkels and the Swedish Institute of
 * Computer Science.
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
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.  
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  
 *
 * This file is part of the uIP TCP/IP stack and the Contiki operating system.
 *
 *
 */


#include "net/ip/uip.h"
#include "net/ip/uiplib.h"
#include <string.h>

#define DEBUG DEBUG_NONE
#include "net/ip/uip-debug.h"

/*-----------------------------------------------------------------------------------*/
/* Parse a IPv4-address from a string. Returns the number of characters read 
 * for the address. */
int
uiplib_ip4addrconv(const char *addrstr, uip_ip4addr_t *ipaddr)
{
  unsigned char tmp;
  char c;
  unsigned char i, j;
  uint8_t charsread = 0;

  tmp = 0;

  for(i = 0; i < 4; ++i) {
    j = 0;
    do {
      c = *addrstr;
      ++j;
      if(j > 4) {
        return 0;
      }
      if(c == '.' || c == 0 || c == ' ') {
        ipaddr->u8[i] = tmp;
        tmp = 0;
      } else if(c >= '0' && c <= '9') {
      	tmp = (tmp * 10) + (c - '0');
      } else {
        return 0;
      }
      ++addrstr;
      ++charsread;
    } while(c != '.' && c != 0 && c != ' ');

  }
  return charsread-1;
}
/*-----------------------------------------------------------------------------------*/
