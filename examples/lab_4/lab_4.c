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
#include <random.h>
#include "dev/button-sensor.h"
#include "dev/leds.h"
#include "sys/ctimer.h"
#include "sys/etimer.h"

#include "project-conf.h"
/*---------------------------------------------------------------------------*/
PROCESS(Lab_4_process, "Lab 4 process");
AUTOSTART_PROCESSES(&Lab_4_process);
/*---------------------------------------------------------------------------*/


static struct etimer et;
PROCESS_THREAD(Lab_4_process, ev, data)
{
  PROCESS_BEGIN();


  static int i = 0;
  static int s, b, c;
  static bool errou = 0;
  static char user[5];
  static char btn;
  static int acertos = 0;
  static char sequencia[5];

  etimer_set(&et, CLOCK_SECOND * 1);
  PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);


  SENSORS_ACTIVATE(button_sensor);
  while(1){
      //GERAR SEQUENCIA DO TAMANHO DA ITERACAO
      //PISCAR OS LEDS (1s) DE ACORDO COM A SEQUENCIA GERADA

      for (s = 0; s <= i; s++){
          char led = gerarLed();
          sequencia[s] = led;
          if (led == 'R'){
              leds_on(LEDS_RED);

              etimer_set(&et, CLOCK_SECOND * 1);
              PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);

              leds_off(LEDS_RED|LEDS_GREEN);
          }else {
              leds_on(LEDS_GREEN);
              etimer_set(&et, CLOCK_SECOND * 1);
              PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);
              leds_off(LEDS_RED|LEDS_GREEN);

              etimer_set(&et, CLOCK_SECOND * 0.3);
              PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);

          }
      }

      errou = 0;

      for (b = 0; b <= i; b++){
          PROCESS_WAIT_EVENT_UNTIL(ev == sensors_event);
          if(data == &button_left_sensor){
              btn = 'R';
          }else if (data == &button_right_sensor){
              btn = 'G';
          }
          printf("Pressionei %c\n", btn);
          user[b] = btn;
      }

      //comparar posicoes das sequencias
      for (c = 0; c <= i; c++){
          if (sequencia[c] != user[c]){
              //piscar os leds
              //se o usuario errou, apresenta os pontos e a sequencia correta
              leds_on(LEDS_ALL);
              printf("Errou! Voce fez %d pontos. A sequencia correta eh: \n",acertos);
              for (int p = 0;p<=i;p++)
                  printf("%c, ", sequencia[p]);
              errou = 1;
              break;
          }else {
              acertos++;
          }
      }

      if (errou){
          break;
      }else if (acertos == 5){
          printf("acertou a sequencia completa!\n");
          //pisca os leds
          break;
      }else {
          printf("Acertando, continua...\n");
          leds_off(LEDS_ALL);
          etimer_set(&et, CLOCK_SECOND * 1);
          PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);
          i++;
      }
  }


  SENSORS_DEACTIVATE(button_sensor);
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
