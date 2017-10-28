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
#include "contiki-lib.h"
#include "contiki-net.h"
#include "sys/etimer.h"
#include "sys/ctimer.h"
#include "dev/adc-sensor.h"
#include "dev/leds.h"
#include "dev/watchdog.h"
#include "random.h"
#include "button-sensor.h"
#include "board-peripherals.h"
#include "ti-lib.h"
#include "net/ip/resolv.h"
#include "net/ip/uip-debug.h"
#include "lib/sensors.h"
#include "cpu/cc26xx-cc13xx/lpm.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#define DEBUG DEBUG_PRINT
#define MAX_PAYLOAD_LEN		120
#define CONN_PORT     8802

#define UIP_UDP_BUF  ((struct uip_udp_hdr *)&uip_buf[UIP_LLH_LEN + UIP_IPH_LEN])
#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

//Enderaçamento do indicador de LEDs
#define LED_AZUL        	(IOID_30)
#define LED_VERDE       	(IOID_29)
#define LED_VERMELHO    	(IOID_28)
//Frequência do PWM
#define PWM_FREQ			(5000)

//Protocolo da aplicação:
#define TEMP_READING (0x59)
#define LEDS_INDICATOR (0x5A)
#define LEDS_STATE (0x5B)
#define COOLER_PWM_DC (0x5C)
#define PWM_STATE (0x5D)

//Conexão UDP
//static char buf[MAX_PAYLOAD_LEN];
static struct uip_udp_conn *client_conn;

struct sensorRequest {
    char request;
    int32_t dado;
} __attribute__((packed));

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

static int32_t ler_temperatura(){
    static const struct sensors_sensor *lm35;
    static int32_t valorAD = 0;
	

    //Referência do sensor ADC
    lm35 = sensors_find(ADC_SENSOR);

    SENSORS_ACTIVATE(*lm35);
    lm35->configure(ADC_SENSOR_SET_CHANNEL, ADC_COMPB_IN_AUXIO4);
    //Recupera o valor de A4 (DIO 26)
    //O LM35 aumenta em 10mV a cada 1°C:
    valorAD = (int32_t)((lm35->value(ADC_SENSOR_VALUE))/10000);
    printf("Temperatura: %d\n" , valorAD);
    SENSORS_DEACTIVATE(*lm35);
    return valorAD;

}

/*---------------------------------------------------------------------------*/
static void acender_led(int azul, int verde, int vermelho){
    printf("led azul: %d, led verde: %d, led vermelho: %d", azul, verde, vermelho);
    //Configura os pinos como saída:
    IOCPinTypeGpioOutput(LED_AZUL);
    IOCPinTypeGpioOutput(LED_VERDE);
    IOCPinTypeGpioOutput(LED_VERMELHO);
	
    GPIO_clearDio(LED_AZUL);
    GPIO_clearDio(LED_VERDE);
    GPIO_clearDio(LED_VERMELHO);

    if (azul)
        GPIO_setDio(LED_AZUL);
    else if (verde)
        GPIO_setDio(LED_VERDE);
    else if (vermelho)
        GPIO_setDio(LED_VERMELHO);

}

/*---------------------------------------------------------------------------*/
static void ligar_cooler(int velocidade){

    static uint32_t loadvalue;
    static uint32_t ticks;

    //inicializa o PWM com carga mínima (desligado?)
    loadvalue = pwminit(PWM_FREQ);

    ticks = (velocidade * loadvalue) / 100;
    printf("COOLER PWM currenty_duty = %lu, ticks = %lu\n", velocidade, ticks);
    ti_lib_timer_match_set(GPT0_BASE, TIMER_A, loadvalue - ticks);
}

static void tcpip_handler(void){
    if(uip_newdata()) {
        struct sensorRequest* req = (struct sensorRequest*)uip_appdata;
        struct sensorRequest reply;
        printf("Response from the server: '%c'\n", req->request);

        switch(req->request){
        case LEDS_INDICATOR:
			PRINTF("Recebido LEDS_INDICATOR de [");
            PRINT6ADDR(&client_conn->ripaddr);
            PRINTF("]: %u\n", UIP_HTONS(client_conn->rport));

            acender_led(req->dado&0x04,req->dado&0x02,req->dado&0x01);

            //Devolve o state do indicador de leds:
            uip_ipaddr_copy(&client_conn->ripaddr, &UIP_IP_BUF->srcipaddr);
            client_conn->rport = UIP_UDP_BUF->destport;
            reply.request = LEDS_STATE;
            reply.dado = req->dado;
            PRINTF("Enviando LEDS_STATE %d para [", reply.dado);
            PRINT6ADDR(&client_conn->ripaddr);
            PRINTF("]: %u\n", UIP_HTONS(client_conn->rport));
            uip_udp_packet_send(client_conn, (void*)&reply, sizeof(struct sensorRequest));
            break;
        case COOLER_PWM_DC:
            PRINTF("Recebido COOLER_PWM_DC de [");
            PRINT6ADDR(&client_conn->ripaddr);
            PRINTF("]: %u\n", UIP_HTONS(client_conn->rport));

            ligar_cooler(req->dado);

            //Devolve o state do pwm:
            uip_ipaddr_copy(&client_conn->ripaddr, &UIP_IP_BUF->srcipaddr);
            client_conn->rport = UIP_UDP_BUF->destport;
            reply.request = PWM_STATE;
            reply.dado = req->dado;
            PRINTF("Enviando PWM_STATE %c para [", reply.dado);
            PRINT6ADDR(&client_conn->ripaddr);
            PRINTF("]: %u\n", UIP_HTONS(client_conn->rport));
            uip_udp_packet_send(client_conn, (void*)&reply, sizeof(struct sensorRequest));
            break;
        default:
            PRINTF("Comando inválido!");
        }
    }
}

static void timeout_handler(void){
    struct sensorRequest req;
    req.request = TEMP_READING;
	req.dado = ler_temperatura();

    if(uip_ds6_get_global(ADDR_PREFERRED) == NULL) {
      PRINTF("Aguardando auto-configuracao de IP\n");
      return;
    }

    PRINTF("Cliente para [");
    PRINT6ADDR(&client_conn->ripaddr);
    PRINTF("]: %u\n", UIP_HTONS(client_conn->rport));

    uip_udp_packet_send(client_conn, (void*)&req, sizeof(struct sensorRequest));
}

/*---------------------------------------------------------------------------*/
static void print_local_addresses(void){
  int i;
  uint8_t state;

  PRINTF("Client IPv6 addresses: ");
  for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
    state = uip_ds6_if.addr_list[i].state;
    if(uip_ds6_if.addr_list[i].isused &&
       (state == ADDR_TENTATIVE || state == ADDR_PREFERRED)) {
      PRINT6ADDR(&uip_ds6_if.addr_list[i].ipaddr);
      PRINTF("\n");
    }
  }
}

/*
 *monitorar sensor temperatura de tempo em tempo. e enviar pacote.
 * tratar pacotes recebidos para ligar e desligar leds e acionar cooler
 */
PROCESS(tempsensor_process, "tempsensor process");
AUTOSTART_PROCESSES(&tempsensor_process);

PROCESS_THREAD(tempsensor_process, ev, data) {
	PROCESS_BEGIN();
	
	static struct etimer et;
	uip_ipaddr_t ipaddr;

	print_local_addresses();
	
	//IPv6 do servidor: 2804:7f4:3b80:c6e8:b8c4:e25b:5426:844
	uip_ip6addr(&ipaddr, 0x2804, 0x7f4, 0x3b80, 0xc6e8, 0xb8c4, 0xe25b, 0x5426, 0x844);

	//Nova conexão com o servidor:
	client_conn = udp_new(&ipaddr, UIP_HTONS(CONN_PORT), NULL);
	udp_bind(client_conn, UIP_HTONS(CONN_PORT));

	PRINT6ADDR(&client_conn->ripaddr);
	PRINTF(" local/remote port %u/%u\n", UIP_HTONS(client_conn->lport), UIP_HTONS(client_conn->rport));
	
	printf("Iniciando Sensor de temperatura no canal A4 do ADC...\n");
	
	//Timer de 1s
    etimer_set(&et, CLOCK_SECOND);
	
	while(1){
        PROCESS_YIELD();
		if(ev == PROCESS_EVENT_TIMER){
			timeout_handler();
			//Reinicia o timer:
            etimer_reset(&et);
		}else if (ev == tcpip_event){
			tcpip_handler();
		}
	}
	
	PROCESS_END();
}
