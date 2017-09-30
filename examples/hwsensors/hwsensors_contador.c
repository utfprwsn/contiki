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
#include "sys/etimer.h"
#include "sys/ctimer.h"
#include "dev/leds.h"
#include "dev/watchdog.h"
#include "random.h"
#include "button-sensor.h"
#include "board-peripherals.h"

#include "ti-lib.h"

#include <stdio.h>
#include <stdint.h>

static struct etimer et;
static int contador;
/*---------------------------------------------------------------------------*/
PROCESS(hwsensors_process, "hwsensors process");
AUTOSTART_PROCESSES(&hwsensors_process);

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(hwsensors_process, ev, data)
{
    PROCESS_BEGIN();
    printf("Iniciando contador...\n");

    //configura os pinos DIO25 e DIO26 como saída
    IOCPinTypeGpioOutput(IOID_25);
    IOCPinTypeGpioOutput(IOID_26);

    contador = 0;
    etimer_set(&et, CLOCK_SECOND * 1);

    while(1){
        PROCESS_YIELD();

        if(ev == PROCESS_EVENT_TIMER){
            contador++;
            printf("Contador: %d\n", contador);
            //Se o contador chegar a 4, zera:
            if (contador == 4){
                contador = 0;
            }

            //aplica o valor do contador nos pinos do GPIO
            GPIO_writeDio(25, contador & 0x1);
            GPIO_writeDio(26, contador >> 1);
            leds_toggle(LEDS_GREEN);

            etimer_reset(&et);
        }
    }
    PROCESS_END();
}
