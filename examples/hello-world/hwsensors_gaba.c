/*
 * Copyright (c) 2014, Texas Instruments Incorporated - http://www.ti.com/
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
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*---------------------------------------------------------------------------*/
/**
 * \addtogroup cc26xx-platforms
 * @{
 *
 * \defgroup cc26xx-examples CC26xx Example Projects
 *
 * Example projects for CC26xx-based platforms.
 * @{
 *
 * \defgroup cc26xx-demo CC26xx Demo Project
 *
 *   Example project demonstrating the CC13xx/CC26xx platforms
 *
 *   This example will work for the following boards:
 *   - srf06-cc26xx: SmartRF06EB + CC13xx/CC26xx EM
 *   - CC2650 and CC1350 SensorTag
 *   - CC1310, CC1350, CC2650 LaunchPads
 *
 *   This is an IPv6/RPL-enabled example. Thus, if you have a border router in
 *   your installation (same RDC layer, same PAN ID and RF channel), you should
 *   be able to ping6 this demo node.
 *
 *   This example also demonstrates CC26xx BLE operation. The process starts
 *   the BLE beacon daemon (implemented in the RF driver). The daemon will
 *   send out a BLE beacon periodically. Use any BLE-enabled application (e.g.
 *   LightBlue on OS X or the TI BLE Multitool smartphone app) and after a few
 *   seconds the cc26xx device will be discovered.
 *
 * - etimer/clock : Every CC26XX_DEMO_LOOP_INTERVAL clock ticks the LED defined
 *                  as CC26XX_DEMO_LEDS_PERIODIC will toggle and the device
 *                  will print out readings from some supported sensors
 * - sensors      : Some sensortag sensors are read asynchronously (see sensor
 *                  documentation). For those, this example will print out
 *                  readings in a staggered fashion at a random interval
 * - Buttons      : CC26XX_DEMO_SENSOR_1 button will toggle CC26XX_DEMO_LEDS_BUTTON
 *                - CC26XX_DEMO_SENSOR_2 turns on LEDS_REBOOT and causes a
 *                  watchdog reboot
 *                - The remaining buttons will just print something
 *                - The example also shows how to retrieve the duration of a
 *                  button press (in ticks). The driver will generate a
 *                  sensors_changed event upon button release
 * - Reed Relay   : Will toggle the sensortag buzzer on/off
 *
 * @{
 *
 * \file
 *     Example demonstrating the cc26xx platforms
 */
#include "contiki.h"
#include "sys/etimer.h"
#include "sys/ctimer.h"

#include "dev/leds.h"
#include "dev/watchdog.h"
#include "dev/adc-sensor.h"

#include "random.h"
#include "button-sensor.h"
#include "board-peripherals.h"
#include "contiki-net.h"
#include "lib/sensors.h"
#include "batmon-sensor.h"
#include "ti-lib.h"


#include "lpm.h"

#include <stdio.h>
#include <stdint.h>
/*---------------------------------------------------------------------------*/
#define CC26XX_DEMO_LOOP_INTERVAL       (CLOCK_SECOND * 20)
#define CC26XX_DEMO_LEDS_PERIODIC       LEDS_YELLOW
#define CC26XX_DEMO_LEDS_BUTTON         LEDS_RED
#define CC26XX_DEMO_LEDS_REBOOT         LEDS_ALL
/*---------------------------------------------------------------------------*/
#define CC26XX_DEMO_SENSOR_NONE         (void *)0xFFFFFFFF


/*---------------------------------------------------------------------------*/
static struct etimer et;
static struct etimer et_adc;
static struct etimer et_adc_sensor;
static struct etimer et_gpio;
static uint16_t single_adc_sample;

/*---------------------------------------------------------------------------*/
PROCESS(cc26xx_demo_process, "cc26xx demo process");

//não ativar as duas junto para uma não interferir na outra
PROCESS(adc_process, "adc bare metal process");
PROCESS(adc_process_sensor, "adc driver process");

PROCESS(gpio_process, "gpio bare metal process");




AUTOSTART_PROCESSES(&cc26xx_demo_process, &adc_process_sensor,&gpio_process);
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*
 * Update sensor readings in a staggered fashion every SENSOR_READING_PERIOD
 * ticks + a random interval between 0 and SENSOR_READING_RANDOM ticks
 */
#define SENSOR_READING_PERIOD (CLOCK_SECOND * 20)
#define SENSOR_READING_RANDOM (CLOCK_SECOND << 4)


 uint8_t pwm_request_max_pm(void)
 {
     return LPM_MODE_DEEP_SLEEP;
 }

 void sleep_enter(uint8_t mode)
 {
     leds_on(LEDS_RED);
 }

 void sleep_leave(void)
 {
     leds_off(LEDS_RED);

 }

 LPM_MODULE(pwmdrive_module, pwm_request_max_pm, sleep_enter, sleep_leave, LPM_DOMAIN_PERIPH);

int16_t pwminit(int32_t freq)
{

    uint32_t load=0;

    ti_lib_ioc_pin_type_gpio_output(IOID_21);
    leds_off(LEDS_RED);

    /* Enable GPT0 clocks under active, sleep, deep sleep */
    ti_lib_prcm_peripheral_run_enable(PRCM_PERIPH_TIMER0);
    ti_lib_prcm_peripheral_sleep_enable(PRCM_PERIPH_TIMER0);
    ti_lib_prcm_peripheral_deep_sleep_enable(PRCM_PERIPH_TIMER0);
    ti_lib_prcm_load_set();
    while(!ti_lib_prcm_load_get());

    /*
     * Register ourself with LPM. This will keep the PERIPH PD powered on
     * during deep sleep, allowing the pwm to keep working while the chip is
     * being power-cycled
     */
    //supostamente, mas pelo jeito não está implementado no driver
    //a solução foi desabilitar os Low Power Modes da implementação
    //arquivo /contiki/cpu/cc26xx-cc13xx/lpm.h, substituir a linha
    //#define LPM_MODE_MAX_SUPPORTED LPM_MODE_DEEP_SLEEP para
    //#define LPM_MODE_MAX_SUPPORTED LPM_MODE_AWAKE
    lpm_register_module(&pwmdrive_module);

    /* Drive the I/O ID with GPT0 / Timer A */
    ti_lib_ioc_port_configure_set(IOID_21, IOC_PORT_MCU_PORT_EVENT0, IOC_STD_OUTPUT);

    /* GPT0 / Timer A: PWM, Interrupt Enable */
    //HWREG(GPT0_BASE + GPT_O_TAMR) = (TIMER_CFG_A_PWM & 0xFF) | GPT_TAMR_TAPWMIE;
    ti_lib_timer_configure(GPT0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM | TIMER_CFG_B_PWM);


    /* Stop the timer */
    ti_lib_timer_disable(GPT0_BASE, TIMER_A);
    ti_lib_timer_disable(GPT0_BASE, TIMER_B);

    if(freq > 0)
    {
        load = (GET_MCU_CLOCK / freq);

        ti_lib_timer_load_set(GPT0_BASE, TIMER_A, load);
        ti_lib_timer_match_set(GPT0_BASE, TIMER_A, 1);

        /* Start */
        ti_lib_timer_enable(GPT0_BASE, TIMER_A);
    }
    return load;

}


void pwmdeinit()
{
    /*
       * Unregister the buzzer module from LPM. This will effectively release our
       * lock for the PERIPH PD allowing it to be powered down (unless some other
       * module keeps it on)
       */
      lpm_unregister_module(&pwmdrive_module);

      /* Stop the timer */
      ti_lib_timer_disable(GPT0_BASE, TIMER_A);

      /*
       * Stop the module clock:
       *
       * Currently GPT0 is in use by clock_delay_usec (GPT0/TB) and by this
       * module here (GPT0/TA).
       *
       * clock_delay_usec
       * - is definitely not running when we enter here and
       * - handles the module clock internally
       *
       * Thus, we can safely change the state of module clocks here.
       */
      ti_lib_prcm_peripheral_run_disable(PRCM_PERIPH_TIMER0);
      ti_lib_prcm_peripheral_sleep_disable(PRCM_PERIPH_TIMER0);
      ti_lib_prcm_peripheral_deep_sleep_disable(PRCM_PERIPH_TIMER0);
      ti_lib_prcm_load_set();
      while(!ti_lib_prcm_load_get());

      /* Un-configure the pin */
      ti_lib_ioc_pin_type_gpio_input(IOID_21);
      ti_lib_ioc_io_input_set(IOID_21, IOC_INPUT_DISABLE);
}

/*---------------------------------------------------------------------------*/

PROCESS_THREAD(cc26xx_demo_process, ev, data)
{
    static int16_t current_duty = 0;
    static int16_t loadvalue;

    PROCESS_BEGIN();

    printf("PWM demo\n");

    etimer_set(&et, CC26XX_DEMO_LOOP_INTERVAL);

    loadvalue = pwminit(5000);

    while(1)
    {

        PROCESS_YIELD();

        if(ev == sensors_event)
        {
            if(data == &button_left_sensor)
            {
                int32_t ticks;
                if(current_duty > 0)
                {
                    current_duty -= 10;
                }
                ticks = (current_duty * loadvalue) / 100;
                if(ticks==0)
                {
                    ticks++;
                }
                if(ticks==loadvalue)
                {
                    ticks=loadvalue--;
                }
                printf("Duty = %d\n", current_duty);
                ti_lib_timer_match_set(GPT0_BASE, TIMER_A, ticks);

            }
            else if(data == &button_right_sensor)
            {
                int32_t ticks;
                if(current_duty < 100)
                {
                    current_duty += 10;
                }
                ticks = (current_duty * loadvalue) / 100;
                if(ticks==0)
                {
                    ticks++;
                }
                if(ticks==loadvalue)
                {
                    ticks=loadvalue--;
                }


                printf("Duty = %d\n", current_duty);
                ti_lib_timer_match_set(GPT0_BASE, TIMER_A, ticks);

            }
        }
        PROCESS_END();
    }
    return 0;
}
/*---------------------------------------------------------------------------*/

PROCESS_THREAD(adc_process, ev, data)
{
  PROCESS_BEGIN();

  etimer_set(&et_adc, CLOCK_SECOND * 4);

  while(1) {

    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et_adc));

    /* initialization of ADC */
    ti_lib_aon_wuc_aux_wakeup_event(AONWUC_AUX_WAKEUP);
    while(!(ti_lib_aon_wuc_power_status_get() & AONWUC_AUX_POWER_ON));

    /*
     * Enable clock for ADC digital and analog interface (not currently enabled
     * in driver)
     */
    ti_lib_aux_wuc_clock_enable(AUX_WUC_ADI_CLOCK | AUX_WUC_ANAIF_CLOCK |
                                AUX_WUC_SMPH_CLOCK);
    while(ti_lib_aux_wuc_clock_status(AUX_WUC_ADI_CLOCK | AUX_WUC_ANAIF_CLOCK |
                                      AUX_WUC_SMPH_CLOCK)
          != AUX_WUC_CLOCK_READY);

    /* Connect AUX IO7 (DIO23, but also DP2 on XDS110) as analog input. */
    ti_lib_aux_adc_select_input(ADC_COMPB_IN_AUXIO7);

    /* Set up ADC range, AUXADC_REF_FIXED = nominally 4.3 V */
    ti_lib_aux_adc_enable_sync(AUXADC_REF_FIXED, AUXADC_SAMPLE_TIME_2P7_US,
                               AUXADC_TRIGGER_MANUAL);

    /* Trigger ADC converting */
    ti_lib_aux_adc_gen_manual_trigger();

    /* Read value */
    single_adc_sample = ti_lib_aux_adc_read_fifo();

    printf("ADC value: %d\n", single_adc_sample);

    /* Shut the adc down */
    ti_lib_aux_adc_disable();

    etimer_reset(&et_adc);
  }

  PROCESS_END();
}


PROCESS_THREAD(gpio_process, ev, data)
{
    static int8_t counter = 0;
    PROCESS_BEGIN();

    etimer_set(&et_gpio, CLOCK_SECOND * 1);

    /* initialization of GPIO output */
    ti_lib_rom_ioc_pin_type_gpio_output(IOID_27);
    ti_lib_rom_ioc_pin_type_gpio_output(IOID_30);
    ti_lib_gpio_clear_multi_dio(1<<IOID_27 | 1<<IOID_30);


    while(1) {

        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et_gpio));
        counter++;

        if(counter & 0x1)
        {
            ti_lib_gpio_set_dio(IOID_27);
        }
        else
        {
            ti_lib_gpio_clear_dio(IOID_27);
        }

        if(counter & 0x2)
        {
            ti_lib_gpio_set_dio(IOID_30);
        }
        else
        {
            ti_lib_gpio_clear_dio(IOID_30);
        }
        etimer_reset(&et_gpio);
    }

    PROCESS_END();
}



PROCESS_THREAD(adc_process_sensor, ev, data)
{
  static struct sensors_sensor *sensor;
  static int rv;
  PROCESS_BEGIN();
  //atualizar /contiki/platform/srf06-cc26xx/launchpad/launchpad-sensors.c para incluir o adc-sensor.h
  //#include "dev/adc-sensor.h"
  //SENSORS(&button_left_sensor, &button_right_sensor, &bmp_280_sensor, &adc_sensor);

  etimer_set(&et_adc_sensor, CLOCK_SECOND * 4);
  while(1) {

      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et_adc_sensor));

      sensor = (struct sensors_sensor *) sensors_find(ADC_SENSOR);
      if(sensor) {
          SENSORS_ACTIVATE(*sensor);
          sensor->configure(ADC_SENSOR_SET_CHANNEL,ADC_COMPB_IN_AUXIO7);
          rv = sensor->value(ADC_SENSOR_VALUE);
          SENSORS_DEACTIVATE(*sensor);
          printf("ADC value is %d\n", rv);
      }
      else
      {
          printf("Sensor not found!\n");

      }
      etimer_reset(&et_adc_sensor);
  }

  PROCESS_END();
}






/**
 * @}
 * @}
 * @}
 */
