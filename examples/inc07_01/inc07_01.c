/*
 * Copyright (c) 2006, Swedish Institute of Computer Science.
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
 */

/**
 * \file
 *         A very simple Contiki application showing how Contiki programs look
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#include "contiki.h"
#include <stdio.h> /* For printf() */
#include "sys/etimer.h"
#include "dev/leds.h"
#include "button-sensor.h"

#define LED_PING_EVENT (44)
#define LED_PONG_EVENT (45)

static struct etimer et_hello;
static struct etimer et_blink;
static struct etimer et_3;

/*---------------------------------------------------------------------------*/
PROCESS(inc07_01_process, "inc07_01 process");
PROCESS(blink_process, "blink process");
PROCESS(terceiro_process, "terceiro process");
PROCESS(pong_process, "pong process");
PROCESS(read_button_process,"read button process");
AUTOSTART_PROCESSES( &pong_process, &read_button_process);
//&inc07_01_process, &blink_process, &terceiro_process,
/*---------------------------------------------------------------------------*/

PROCESS_THREAD(inc07_01_process, ev, data)
{
  PROCESS_BEGIN();
  etimer_set(&et_hello, 4*CLOCK_SECOND);
  
  while(1) {
      PROCESS_WAIT_EVENT();
      if (ev == PROCESS_EVENT_TIMER) {
          printf("oi samuel!\n");
          etimer_reset(&et_hello);
          printf("inc07_01_process enviando ping!\n");
          process_post(&pong_process, LED_PING_EVENT, (void*)(&inc07_01_process));
      }else if(ev == LED_PONG_EVENT){
          printf("inc07_01_process: mensagem pong recebido\n");
      }
  }

  PROCESS_END();
}

//==============================================================================================

PROCESS_THREAD(blink_process, ev, data)
{
  PROCESS_BEGIN();
  etimer_set(&et_blink, 2*CLOCK_SECOND);

  while(1) {
      PROCESS_WAIT_EVENT();
      if (ev == PROCESS_EVENT_TIMER) {
          leds_toggle(LEDS_GREEN);
          etimer_reset(&et_blink);
          printf("blink_process enviando ping!\n");
          process_post(&pong_process, LED_PING_EVENT, (void*)(&blink_process));
      }else if(ev == LED_PONG_EVENT){
          printf("blink_process: mensagem pong recebido\n");
      }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/

PROCESS_THREAD(terceiro_process, ev, data)
{
  PROCESS_BEGIN();
  etimer_set(&et_3, 10*CLOCK_SECOND);

  while(1) {
      PROCESS_WAIT_EVENT();
      if (ev == PROCESS_EVENT_TIMER) {
          printf("oi samuel, passou dez segundos!\n");
          etimer_reset(&et_3);
          printf("terceiro_process enviando ping!\n");
          process_post(&pong_process, LED_PING_EVENT, (void*)(&terceiro_process));
      }else if(ev == LED_PONG_EVENT){
          printf("terceiro_process: mensagem pong recebido\n");
      }
  }

  PROCESS_END();
}


//===========================================================================

PROCESS_THREAD(pong_process, ev, data)
{
  PROCESS_BEGIN();


  while(1) {
      PROCESS_WAIT_EVENT();
      if (ev == LED_PING_EVENT) {
          printf("Pong: Recebido ping do processo %s\n", ((struct process*)data)->name);
          process_post((struct process*)data, LED_PONG_EVENT, NULL);
      }
  }

  PROCESS_END();
}

PROCESS_THREAD(read_button_process, ev, data)
{
    PROCESS_BEGIN();
    printf("Read Button Demo\n");

    while(1){
        PROCESS_YIELD();

        if(ev == sensors_event){
            if(data == &button_left_sensor){
                printf("Left Button!\n");
                leds_toggle(LEDS_RED);
            }
            else if(data == &button_right_sensor){
                leds_toggle(LEDS_GREEN);
                printf("Right Button!\n");
            }
            printf("read_button_process enviando ping\n ");
            process_post(&pong_process, LED_PING_EVENT, (void*)(&read_button_process));
        }else if(ev == LED_PONG_EVENT){
            printf("read_button_process: mensagem pong recebido\n");
        }
    }
    PROCESS_END();
}
