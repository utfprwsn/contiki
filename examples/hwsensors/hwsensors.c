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

#include "dev/adc-sensor.h"
#include "lib/sensors.h"

static struct sensors_sensor *sensor;
static struct etimer et;

/*---------------------------------------------------------------------------*/
PROCESS(hwsensors_process, "hwsensors process");
AUTOSTART_PROCESSES(&hwsensors_process);

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(hwsensors_process, ev, data)
{
    PROCESS_BEGIN();

    printf("Iniciando ADC...\n");

    //seta o timer para 1s
    etimer_set(&et, CLOCK_SECOND);

    sensor = sensors_find(ADC_SENSOR);

    SENSORS_ACTIVATE(*sensor);

    sensor->configure(ADC_SENSOR_SET_CHANNEL, ADC_COMPB_IN_AUXIO4);

    while(1){
        PROCESS_YIELD();

        if(ev == PROCESS_EVENT_TIMER){
            leds_toggle(LEDS_GREEN);
            printf("Sensor: %d\n", sensor->value(ADC_SENSOR_VALUE));
            etimer_reset(&et);
        }
    }

    SENSORS_DEACTIVATE(*sensor);
    PROCESS_END();
}
