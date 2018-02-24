/**
 * \file
 *        Sensor de temperatura que exibe a faixa de temperatura numa escala de LEDs:
 *        - LED AZUL:       até 25°C
 *        - LED VERDE:      até 30°C
 *        - LED VERMELHO:   acima de 30°C
 *        Na faixa entre 25 e 30°C, o sensor aciona o cooler em velocidade V1 (baixa).
 *        Acima de 30°C, o sensor aciona o cooler em velocidade V2 (alta).
 *
 * \authors
 *         Marco Antonio
 *         Rodrigo
 *         Samuel
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
#include "cpu/cc26xx-cc13xx/lpm.h"

#define TEMPERATURA_EVENT 	(44)
#define TEMP_FAIXA_MEDIA    (25)
#define TEMP_FAIXA_ALTA     (30)
#define LED_AZUL        	(IOID_30)
#define LED_VERDE       	(IOID_29)
#define LED_VERMELHO    	(IOID_28)
#define PWM_FREQ			(5000)
#define PWM_DUTYCYCLE_V1	(50)
#define PWM_DUTYCYCLE_V2	(90)

//Estrutura para ser utilizada na comunicação entre os processos, tendo com valor a temperatura lida via ADC: 
struct temperatura {
	int32_t valor;
};

//Configuração do LPM e PWM:
uint8_t pwm_request_max_pm(void){
    return LPM_MODE_DEEP_SLEEP;
}
void sleep_enter(void){
    leds_on(LEDS_RED);
}
void sleep_leave(void){
    leds_off(LEDS_RED);
}
LPM_MODULE(pwmdrive_module, pwm_request_max_pm, sleep_enter, sleep_leave, LPM_DOMAIN_PERIPH);

uint32_t pwminit(int32_t freq){
    /* Register with LPM. This will keep the PERIPH PD powered on
    * during deep sleep, allowing the pwm to keep working while the chip is
    * being power-cycled */

    lpm_register_module(&pwmdrive_module);
    uint32_t load = 0;
    ti_lib_ioc_pin_type_gpio_output(IOID_21);
    leds_off(LEDS_RED);

    /* Enable GPT0 clocks under active mode */
    ti_lib_prcm_peripheral_run_enable(PRCM_PERIPH_TIMER0);
    ti_lib_prcm_peripheral_sleep_enable(PRCM_PERIPH_TIMER0);
    ti_lib_prcm_peripheral_deep_sleep_enable(PRCM_PERIPH_TIMER0);
    ti_lib_prcm_load_set();
    while(!ti_lib_prcm_load_get());

    /* Enable GPT0 clocks under active, sleep, deep sleep */
    ti_lib_prcm_peripheral_run_enable(PRCM_PERIPH_TIMER0);
    ti_lib_prcm_peripheral_sleep_enable(PRCM_PERIPH_TIMER0);
    ti_lib_prcm_peripheral_deep_sleep_enable(PRCM_PERIPH_TIMER0);
    ti_lib_prcm_load_set();
    while(!ti_lib_prcm_load_get());

    /* Drive the I/O ID with GPT0 / Timer A */
    ti_lib_ioc_port_configure_set(IOID_21, IOC_PORT_MCU_PORT_EVENT0, IOC_STD_OUTPUT);

    /* GPT0 / Timer A: PWM, Interrupt Enable */
    ti_lib_timer_configure(GPT0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM | TIMER_CFG_B_PWM);

    /* Stop the timers */
    ti_lib_timer_disable(GPT0_BASE, TIMER_A);
    ti_lib_timer_disable(GPT0_BASE, TIMER_B);

    if(freq > 0) {
        load = (GET_MCU_CLOCK / freq);
        ti_lib_timer_load_set(GPT0_BASE, TIMER_A, load);
        ti_lib_timer_match_set(GPT0_BASE, TIMER_A, load - 1);
        /* Start */
        ti_lib_timer_enable(GPT0_BASE, TIMER_A);
    }
    return load;
}

/*---------------------------------------------------------------------------*/
PROCESS(tempsensor_process, "tempsensor process");
PROCESS(ledindicator_process, "ledindicator process");
PROCESS(cooler_process, "cooler process");
AUTOSTART_PROCESSES(&tempsensor_process, &ledindicator_process, &cooler_process);

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(tempsensor_process, ev, data)
{
    PROCESS_BEGIN();
    static const struct sensors_sensor *lm35;
    static struct etimer et;
    static int32_t valorAD = 0;
	static struct temperatura sTemp;
	
    printf("Iniciando Sensor de temperatura no canal A4 do ADC...\n");

    //Timer de 1s
    etimer_set(&et, CLOCK_SECOND);

    //Referência do sensor ADC
    lm35 = sensors_find(ADC_SENSOR);

    while(1){
        PROCESS_YIELD();

        if(ev == PROCESS_EVENT_TIMER){
            SENSORS_ACTIVATE(*lm35);
            lm35->configure(ADC_SENSOR_SET_CHANNEL, ADC_COMPB_IN_AUXIO4);
            //Recupera o valor de A4 (DIO 26)
            valorAD = lm35->value(ADC_SENSOR_VALUE);
            //O LM35 aumenta em 10mV a cada 1°C:
            sTemp.valor = (int32_t)(valorAD/10000);
            printf("Sensor: %d, Temperatura: %d\n" , valorAD, sTemp.valor);
            SENSORS_DEACTIVATE(*lm35);

            //Envia temperatura para os processos do led e do cooler
            process_post_synch(&ledindicator_process, TEMPERATURA_EVENT, (void*)(&sTemp));
			process_post_synch(&cooler_process, TEMPERATURA_EVENT, (void*)(&sTemp));
			
            //Reinicia o timer:
            etimer_reset(&et);
        }
    }
    PROCESS_END();
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(ledindicator_process, ev, data)
{
    PROCESS_BEGIN();
    //Configura os pinos como saída:
    IOCPinTypeGpioOutput(LED_AZUL);
    IOCPinTypeGpioOutput(LED_VERDE);
    IOCPinTypeGpioOutput(LED_VERMELHO);
	
	static int32_t t;
	
    while(1){
        PROCESS_WAIT_EVENT();

        if (ev == TEMPERATURA_EVENT){
			t = ((struct temperatura*)data)->valor;
            printf("Indicador LED - Temperatura recebida: %d\n", t);
            if (t <= TEMP_FAIXA_MEDIA){
                GPIO_setDio(LED_AZUL);
                GPIO_clearDio(LED_VERDE);
                GPIO_clearDio(LED_VERMELHO);
            }else if (t <= TEMP_FAIXA_ALTA){
                GPIO_clearDio(LED_AZUL);
                GPIO_setDio(LED_VERDE);
                GPIO_clearDio(LED_VERMELHO);
            }else {
                GPIO_clearDio(LED_AZUL);
                GPIO_clearDio(LED_VERDE);
                GPIO_setDio(LED_VERMELHO);
            }
        }
    }


    PROCESS_END();
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(cooler_process, ev, data)
{
    static uint32_t current_duty = 0;
    static uint32_t loadvalue;
    static uint32_t ticks;
	static int32_t t;
	
    PROCESS_BEGIN();

	//inicializa o PWM com carga mínima (desligado?)
    loadvalue = pwminit(PWM_FREQ);

    while (1){
        PROCESS_WAIT_EVENT();

        if (ev == TEMPERATURA_EVENT){
			t = ((struct temperatura*)data)->valor;
            printf("COOLER - Temperatura recebida: %d\n", t);
			
            if (t <= TEMP_FAIXA_MEDIA){
                //cooler off
				current_duty = 1;
            }else if (t <= TEMP_FAIXA_ALTA){
                current_duty = PWM_DUTYCYCLE_V1;
            }else {
                current_duty = PWM_DUTYCYCLE_V2;
            }
			
            ticks = (current_duty * loadvalue) / 100;
            printf("COOLER PWM currenty_duty = %lu, ticks = %lu\n", current_duty, ticks);
            ti_lib_timer_match_set(GPT0_BASE, TIMER_A, loadvalue - ticks);
        }

    }

    PROCESS_END();
}
