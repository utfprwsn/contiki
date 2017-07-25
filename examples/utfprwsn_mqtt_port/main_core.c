/**
  Licensed to the Apache Software Foundation (ASF) under one
  or more contributor license agreements.  See the NOTICE file
  distributed with this work for additional information
  regarding copyright ownership.  The ASF licenses this file
  to you under the Apache License, Version 2.0 (the
  "License"); you may not use this file except in compliance
  with the License.  You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing,
  software distributed under the License is distributed on an
  "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
  KIND, either express or implied.  See the License for the
  specific language governing permissions and limitations
  under the License.

 *******************************************************************************
 * @file main_core.c
 * @author Ânderson Ignácio da Silva
 * @date 19 Ago 2016
 * @brief Arquivo principal do porte do MQTT-SN, utilizado para testar
 * @see http://www.aignacio.com
 * @license Este projeto está sendo liberado pela licença APACHE 2.0.
 */

#include "contiki.h"
#include "lib/random.h"
#include "clock.h"
#include "sys/ctimer.h"
#include "net/ip/uip.h"
#include "net/ipv6/uip-ds6.h"
#include "mqtt_sn.h"
#include "dev/leds.h"
#include "net/rime/rime.h"
#include "net/ip/uip.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static uint16_t udp_port = 1883;
static uint16_t keep_alive = 5;
static uint16_t broker_address[] = {0xbbbb, 0, 0, 0, 0x6066, 0xcf93, 0xa440, 0xd672};
static struct   etimer time_poll;
//static uint16_t tick_process = 0;
static char     pub_test[20];
static char     device_id[17];
static char     topic_hw[15];
static char     *topics_mqtt[] = {"/topic_1",
                                  "/topic_2",
                                  "/topic_3",
                                  "/topic_4",
                                  "/topic_5",
                                  "/topic_6"};
//static char     *will_topic = "/homestark/offline";
//static char     *will_message = "O dispositivo esta offline";
//Estes tópicos pré-registrados serão mais rápidos de publicar/receber publicações

mqtt_sn_con_t mqtt_sn_connection;

void mqtt_sn_callback(char *topic, char *message){
  printf("\nMensagem recebida:");
  printf("\nTopico:%s Mensagem:%s",topic,message);
}

void init_broker(void){
  char *all_topics[ss(topics_mqtt)+1];
  // Copiando o endereço do hardware
  sprintf(device_id,"%02X%02X%02X%02X%02X%02X%02X%02X",
          linkaddr_node_addr.u8[0],linkaddr_node_addr.u8[1],
          linkaddr_node_addr.u8[2],linkaddr_node_addr.u8[3],
          linkaddr_node_addr.u8[4],linkaddr_node_addr.u8[5],
          linkaddr_node_addr.u8[6],linkaddr_node_addr.u8[7]);
  sprintf(topic_hw,"/homestark_%02X%02X",linkaddr_node_addr.u8[6],linkaddr_node_addr.u8[7]);

  mqtt_sn_connection.client_id     = device_id;
  mqtt_sn_connection.udp_port      = udp_port;
  mqtt_sn_connection.ipv6_broker   = broker_address;
  mqtt_sn_connection.keep_alive    = keep_alive;
  //mqtt_sn_connection.will_topic    = will_topic;   // Configurar como sendo 0x00 senão quiser utilizar
  //mqtt_sn_connection.will_message  = will_message; // Configurar como sendo 0x00 senão quiser utilizar
  mqtt_sn_connection.will_topic    = 0x00; // Configurar como sendo 0x00 senão quiser utilizar
  mqtt_sn_connection.will_message  = 0x00; // Configurar como sendo 0x00 senão quiser utilizar

  mqtt_sn_init();   // Inicializa alocação de eventos e a principal PROCESS_THREAD do MQTT-SN

  size_t i;
  for(i=0;i<ss(topics_mqtt);i++)
    all_topics[i] = topics_mqtt[i];
  all_topics[i] = topic_hw;

  mqtt_sn_create_sck(mqtt_sn_connection,
                     all_topics,
                     ss(all_topics),
                     mqtt_sn_callback);
  mqtt_sn_sub(topic_hw,0);
  // mqtt_sn_sub("/topic_6",0);
  // mqtt_sn_sub("/topic_5",0);
  // mqtt_sn_sub("/topic_4",0);

}

/*---------------------------------------------------------------------------*/
PROCESS(init_system_process, "[Contiki-OS] Iniciando sistema operacional");
AUTOSTART_PROCESSES(&init_system_process);
/*---------------------------------------------------------------------------*/

PROCESS_THREAD(init_system_process, ev, data)
{
  PROCESS_BEGIN();

  debug_os("Inicio do processo MQTT_SN_DEMO");

  init_broker();

  etimer_set(&time_poll, CLOCK_SECOND);

  while(1) {
      PROCESS_WAIT_EVENT();
      // debug_os("Execucao[%d]",tick_process++);
      // sprintf(pub_test,"Execucao %d",tick_process);
      sprintf(pub_test,"%s",topic_hw);

      //mqtt_sn_pub(topic_hw,pub_test,true,0);
      mqtt_sn_pub("/topic_1","CATA",true,0);
      mqtt_sn_pub("/topic_5","BICHA",true,0);
      mqtt_sn_pub("/topic_4","DO",true,0);
      mqtt_sn_pub("/topic_3","CARALHO",true,0);
      mqtt_sn_pub("/topic_2","!!!",true,0);

      //mqtt_sn_check_queue();
      //print_g_topics();
      debug_os("Estado do MQTT:%s",mqtt_sn_check_status_string());
      if (etimer_expired(&time_poll))
        etimer_reset(&time_poll);
  }
  PROCESS_END();
}
