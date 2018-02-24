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
#include "dev/button-sensor.h"
#include "dev/leds.h"
#include "sys/etimer.h"

static struct etimer et;
static struct timer t;
/*---------------------------------------------------------------------------*/
PROCESS(Lab_3_process, "Lab 3 process");
AUTOSTART_PROCESSES(&Lab_3_process);
/*---------------------------------------------------------------------------*/

PROCESS_THREAD(Lab_3_process, ev, data)
{
  PROCESS_BEGIN();

  SENSORS_ACTIVATE(button_sensor);

  static int iteracoes = 0;
  static int acertos = 0;
  static bool LED_GREEN = 1;//0 = RED, 1 = GREEN
  leds_on(LEDS_GREEN);

  //seta o timer para 3 segundos
  etimer_set(&et, CLOCK_SECOND * 3);
  timer_set(&t, CLOCK_SECOND * 2.5);

  while(iteracoes < 10){
      PROCESS_YIELD();

      if(ev == sensors_event) {
          //Recupera o valor do timer
          bool demorou = timer_expired(&t);
          if(data == &button_left_sensor){
              if (!LED_GREEN){
                  //acertou
                  acertos++;
                  printf("Botao correto para o LED vermelho!\n");
                  if (demorou)
                      printf("Demorou mais de 2s para pressionar!\n");
              }else {
                  printf("Botao errado para o LED vermelho!\n");
              }
          }else if (data == &button_right_sensor){
              if (LED_GREEN){
                  //acertou
                  acertos++;
                  printf("Botao correto para o LED verde!\n");
                  if (demorou)
                      printf("Demorou mais de 2s para pressionar!\n");
              }else {
                  printf("Botao errado para o LED verde!\n");
              }
          }

      }else if (ev == PROCESS_EVENT_TIMER){//fim dos 3s
          //dispara os leds alterando vermelho e verde a cada 3s
          leds_toggle(LEDS_RED);
          leds_toggle(LEDS_GREEN);

          //inverte o status do led verde
          LED_GREEN = !LED_GREEN;
          iteracoes++;
          etimer_reset(&et);
          timer_reset(&t);
      }

  }
  printf("Encerrando apos 10 iteracoes! Voce fez %d pontos.\n", acertos);

  SENSORS_DEACTIVATE(button_sensor);
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
