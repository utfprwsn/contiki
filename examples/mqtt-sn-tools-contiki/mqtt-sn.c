/*
  Basic MQTT-SN client library
  Copyright (C) 2013 Nicholas Humfrey

  Permission is hereby granted, free of charge, to any person obtaining
  a copy of this software and associated documentation files (the
  "Software"), to deal in the Software without restriction, including
  without limitation the rights to use, copy, modify, merge, publish,
  distribute, sublicense, and/or sell copies of the Software, and to
  permit persons to whom the Software is furnished to do so, subject to
  the following conditions:

  The above copyright notice and this permission notice shall be
  included in all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
  LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
  WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

  Modifications:
  Copyright (C) 2013 Adam Renner
*/

#include <stdio.h>
//#include <unistd.h>
#include <stdlib.h>
#include "simple-udp.h"
#include "net/ip/uip.h"
#include "list.h"


#include <string.h>
#include <stdint.h>

//#include <unistd.h>
//#include <sys/types.h>
//#include <sys/socket.h>
//#include <netdb.h>
//#include <time.h>
//#include <errno.h>

#include "mqtt-sn.h"


#ifndef AI_DEFAULT
#define AI_DEFAULT (AI_ADDRCONFIG|AI_V4MAPPED)
#endif

static uint8_t debug = FALSE;

topic_map_t *topic_map = NULL;

/*MQTT_SN events*/
process_event_t mqtt_sn_request_event;
//Connack recieved
static process_event_t connack_event;
static process_event_t disconnect_event;
static process_event_t receive_timeout_event;
static process_event_t send_timeout_event;

PROCESS(mqtt_sn_process, "MQTT_SN process");
void manage_response(struct mqtt_sn_connection *mqc, const uip_ipaddr_t *source_addr,
                 const uint8_t *data, uint16_t datalen);
static int manage_request(struct mqtt_sn_request *req, struct mqtt_sn_connection *mqc,
               const char* topic_name, uint8_t qos,clock_time_t time_out);
static void request_timer_callback(void *req);

#if 1
void mqtt_sn_set_debug(uint8_t value)
{
    debug = value;
}
#endif

#if 1
static void
mqtt_sn_receiver(struct simple_udp_connection *sock, const uip_ipaddr_t *sender_addr, uint16_t sender_port,
         const uip_ipaddr_t *receiver_addr, uint16_t receiver_port, const uint8_t *data, uint16_t datalen)
{
  uint8_t msg_type;
  struct mqtt_sn_connection *mqc = (struct mqtt_sn_connection *)sock;
  if (mqc->keep_alive > 0 && mqc->stat == MQTTSN_CONNECTED){
    ctimer_restart((&(mqc->receive_timer)));
    printf("recieve timer reset\n");
  }
  if (datalen >= 2)
  {
    msg_type = data[1];
    switch(msg_type) {
//        case MQTT_SN_TYPE_ADVERTISE:
//        case MQTT_SN_TYPE_SEARCHGW:
//        case MQTT_SN_TYPE_GWINFO:
//        case MQTT_SN_TYPE_CONNECT:
        case MQTT_SN_TYPE_CONNACK:
          {
            memcpy(&(mqc->last_connack),data,sizeof(connack_packet_t));
            process_post(&mqtt_sn_process, connack_event, mqc);
            if(mqc->mc->connack_recv != NULL) {
              mqc->mc->connack_recv(mqc, receiver_addr, data, datalen);
            }
            break;
          }
//        case MQTT_SN_TYPE_WILLTOPICREQ:
//        case MQTT_SN_TYPE_WILLTOPIC:
//        case MQTT_SN_TYPE_WILLMSGREQ:
//        case MQTT_SN_TYPE_WILLMSG:
//        case MQTT_SN_TYPE_REGISTER:
        case MQTT_SN_TYPE_REGACK:
          {
            if(mqc->mc->regack_recv != NULL) {
              mqc->mc->regack_recv(mqc, receiver_addr, data, datalen);
            }
            manage_response(mqc, receiver_addr, data, datalen);
            break;
          }
        case MQTT_SN_TYPE_PUBLISH:
          {
            if(mqc->mc->pub_recv != NULL) {
              mqc->mc->pub_recv(mqc, receiver_addr, data, datalen);
            }
            break;
          }
        case MQTT_SN_TYPE_PUBACK:
          {
            if(mqc->mc->puback_recv != NULL) {
              mqc->mc->puback_recv(mqc, receiver_addr, data, datalen);
            }
            break;
          }
//        case MQTT_SN_TYPE_PUBCOMP:
//        case MQTT_SN_TYPE_PUBREC:
//        case MQTT_SN_TYPE_PUBREL:
//        case MQTT_SN_TYPE_SUBSCRIBE:
        case MQTT_SN_TYPE_SUBACK:
          {
            if(mqc->mc->suback_recv != NULL) {
              mqc->mc->suback_recv(mqc, receiver_addr, data, datalen);
            }
            manage_response(mqc, receiver_addr, data, datalen);
            break;
          }
//        case MQTT_SN_TYPE_UNSUBSCRIBE:
//        case MQTT_SN_TYPE_UNSUBACK:
        case MQTT_SN_TYPE_PINGREQ:
          {
            mqtt_sn_send_pingresp(mqc);
            if(mqc->mc->pingreq_recv != NULL) {
              mqc->mc->pingreq_recv(mqc, receiver_addr, data, datalen);
            }
            break;
          }
        case MQTT_SN_TYPE_PINGRESP:
          {
            if(mqc->mc->pingresp_recv != NULL) {
              mqc->mc->pingresp_recv(mqc, receiver_addr, data, datalen);
            }
            break;
          }
        case MQTT_SN_TYPE_DISCONNECT:
          {
            process_post(&mqtt_sn_process, disconnect_event, mqc);
            if(mqc->mc->disconnect_recv != NULL) {
              mqc->mc->disconnect_recv(mqc, receiver_addr, data, datalen);
            }
            break;
          }
//        case MQTT_SN_TYPE_WILLTOPICUPD:
//        case MQTT_SN_TYPE_WILLTOPICRESP:
//        case MQTT_SN_TYPE_WILLMSGUPD:
//        case MQTT_SN_TYPE_WILLMSGRESP:
        default:
          {
            if (debug) {
              printf(" unrecognized received packet...\n");
            }
            break;
          }
    }
  }
}
#endif

#if 1
int mqtt_sn_create_socket(struct mqtt_sn_connection *mqc, uint16_t local_port, uip_ipaddr_t *remote_addr, uint16_t remote_port)
{
  simple_udp_register(&(mqc->sock), local_port, remote_addr, remote_port, mqtt_sn_receiver);
  mqc->stat = MQTTSN_DISCONNECTED;
  mqc->keep_alive=0;
  mqc->next_message_id = 1;
  mqc->connection_retries = 0;
  LIST_STRUCT_INIT(mqc,requests);
  mqtt_sn_request_event = process_alloc_event();
  process_start(&mqtt_sn_process, NULL);
  return 0;
}
#endif

static void receive_timer_callback(void *mqc)
{
  process_post(&mqtt_sn_process, receive_timeout_event, mqc);
}
static void send_timer_callback(void *mqc)
{
  process_post(&mqtt_sn_process, send_timeout_event, mqc);
}

PROCESS_THREAD(mqtt_sn_process, ev, data)
{
  struct mqtt_sn_connection *mqc;

  PROCESS_BEGIN();

  connack_event = process_alloc_event();
  disconnect_event = process_alloc_event();
  receive_timeout_event = process_alloc_event();
  send_timeout_event = process_alloc_event();

  while(1) {
    PROCESS_WAIT_EVENT();
    mqc = (struct mqtt_sn_connection *)data;
    if(ev == connack_event) {
      //if connection was succesful, set and or start the ctimers.
      if (mqc->last_connack.return_code == 0x00 && mqc->stat == MQTTSN_WAITING_CONNACK){
        mqc->stat = MQTTSN_CONNECTED;
        if (mqc->keep_alive > 0){
          ctimer_set(&mqc->receive_timer, mqc->keep_alive * 2, receive_timer_callback, mqc);
          ctimer_set(&mqc->send_timer,mqc->keep_alive / 2,send_timer_callback, mqc);
        }
      }
    }
    else if (ev == disconnect_event){
      //stop the timers
      if (mqc->keep_alive > 0){
        ctimer_stop(&(mqc->receive_timer));
        ctimer_stop(&(mqc->send_timer));
      }
      mqc->stat = MQTTSN_DISCONNECTED;
    }
    else if (ev == receive_timeout_event){
      //if last receive has expired we need to stop and disconnect
      printf("receive timeout event");
      mqtt_sn_send_disconnect(mqc);
      if(mqc->mc->keepalive_timeout != NULL) {
        mqc->mc->keepalive_timeout(mqc);
      }
    }
    else if (ev == send_timeout_event){
      //if last send has expired, we need to send a pingreq
      mqtt_sn_send_pingreq(mqc);
    }
  }

  PROCESS_END();
}

#if 1
static void send_packet(struct mqtt_sn_connection *mqc, char* data, size_t len)
{
  simple_udp_send(&(mqc->sock), data, len);//these datatypes should all cast fine
  if (mqc->keep_alive>0 && mqc->stat == MQTTSN_CONNECTED)
  {
    //normally we would use this to make sure that we are always sending data to keep the connection alive
    //but since the gateway does not support pubacks, we will always be relying on a
    //steady stream of pings to ensure that the connection stays alive.
    ctimer_restart(&(mqc->send_timer));
  }
}
#endif

#if 1
void mqtt_sn_send_connect(struct mqtt_sn_connection *mqc, const char* client_id, uint16_t keepalive)
{
    connect_packet_t packet;

    // Check that it isn't too long
    if (client_id && strlen(client_id) > 23) {
        printf("Error: client id is too long\n");
        return;
    }

    // Create the CONNECT packet
    packet.type = MQTT_SN_TYPE_CONNECT;
    packet.flags = MQTT_SN_FLAG_CLEAN;
    packet.protocol_id = MQTT_SN_PROTOCOL_ID;
    packet.duration = uip_htons(keepalive);

    strncpy(packet.client_id, client_id, sizeof(packet.client_id)-1);
    packet.client_id[sizeof(packet.client_id) - 1] = '\0';

    packet.length = 0x06 + strlen(packet.client_id);

    if (debug)
        printf("Sending CONNECT packet...\n");

    // Store the keep alive period
    if (keepalive) {
        mqc->keep_alive = keepalive*CLOCK_SECOND;
    }
    mqc->stat = MQTTSN_WAITING_CONNACK;
    mqc->connection_retries++;

    return send_packet(mqc, (char*)&packet, packet.length);
}
#endif
#if 1
uint16_t mqtt_sn_send_register(struct mqtt_sn_connection *mqc, const char* topic_name)
{
    register_packet_t packet;
    size_t topic_name_len = strlen(topic_name);

    if (topic_name_len > MQTT_SN_MAX_TOPIC_LENGTH) {
        printf("Error: topic name is too long\n");
        return 0;
    }

    packet.type = MQTT_SN_TYPE_REGISTER;
    packet.topic_id = 0;
    packet.message_id = uip_htons(mqc->next_message_id++);
    strncpy(packet.topic_name, topic_name, sizeof(packet.topic_name));
    packet.length = 0x06 + strlen(packet.topic_name);

    if (debug)
        printf("Sending REGISTER packet...\n");

    send_packet(mqc, (char*)&packet, packet.length);
    return packet.message_id;
}
#endif

#if 1
uint16_t mqtt_sn_send_regack(struct mqtt_sn_connection *mqc, int topic_id, int message_id)
{
    regack_packet_t packet;
    packet.type = MQTT_SN_TYPE_REGACK;
    packet.topic_id = uip_htons(topic_id);
    packet.message_id = uip_htons(message_id);
    packet.return_code = 0x00;
    packet.length = 0x07;

    if (debug)
        printf("Sending REGACK packet...\n");

    send_packet(mqc, (char*)&packet, packet.length);
    return packet.message_id;
}
#endif

static uint8_t mqtt_sn_get_qos_flag(int8_t qos)
{
    switch (qos) {
        case -1:
          return MQTT_SN_FLAG_QOS_N1;
        case 0:
          return MQTT_SN_FLAG_QOS_0;
        case 1:
          return MQTT_SN_FLAG_QOS_1;
        case 2:
          return MQTT_SN_FLAG_QOS_2;
        default:
          return 0;
    }
}
#if 1
uint16_t mqtt_sn_send_publish(struct mqtt_sn_connection *mqc, uint16_t topic_id, uint8_t topic_type, const char* data, uint16_t data_len, int8_t qos, uint8_t retain)
{
    publish_packet_t packet;

    if (data_len > sizeof(packet.data)) {
        printf("Error: payload is too big\n");
        return 0;
    }

    packet.type = MQTT_SN_TYPE_PUBLISH;
    packet.flags = 0x00;
    if (retain)
        packet.flags += MQTT_SN_FLAG_RETAIN;
    packet.flags += mqtt_sn_get_qos_flag(qos);
    packet.flags += (topic_type & 0x3);
    packet.topic_id = uip_htons(topic_id);
    packet.message_id = uip_htons(mqc->next_message_id++);
    strncpy(packet.data, data, sizeof(packet.data));
    packet.length = 0x07 + data_len;

    if (debug){
        printf("Sending PUBLISH packet...\n");
        if (ctimer_expired(&(mqc->receive_timer))){
            printf("receive timer has already expired...\n");
        }
    }

    send_packet(mqc, (char*)&packet, packet.length);
    return packet.message_id;
}
#endif
#if 1
uint16_t mqtt_sn_send_subscribe(struct mqtt_sn_connection *mqc, const char* topic_name, uint8_t qos)
{
    subscribe_packet_t packet;
    size_t topic_name_len = strlen(topic_name);

    packet.type = MQTT_SN_TYPE_SUBSCRIBE;
    packet.flags = 0x00;
    packet.flags += mqtt_sn_get_qos_flag(qos);
    if (topic_name_len == 2) {
        packet.flags += MQTT_SN_TOPIC_TYPE_SHORT;
    } else {
        packet.flags += MQTT_SN_TOPIC_TYPE_NORMAL;
    }
    packet.message_id = uip_htons(mqc->next_message_id++);
    strncpy(packet.topic_name, topic_name, sizeof(packet.topic_name));
    packet.topic_name[sizeof(packet.topic_name)-1] = '\0';
    packet.length = 0x05 + topic_name_len;

    if (debug){
        printf("Sending SUBSCRIBE packet...\n");
    }

    send_packet(mqc, (char*)&packet, packet.length);
    return packet.message_id;
}
#endif
#if 0
void mqtt_sn_send_subscribe_topic_id(struct mqtt_sn_connection *mqc, uint16_t topic_id, uint8_t qos)
{
    subscribe_packet_t packet;
    packet.type = MQTT_SN_TYPE_SUBSCRIBE;
    packet.flags = 0x00;
    packet.flags += mqtt_sn_get_qos_flag(qos);
    packet.flags += MQTT_SN_TOPIC_TYPE_PREDEFINED;
    packet.message_id = htons(mqc->next_message_id++);
    packet.topic_id = htons(topic_id);
    packet.length = 0x05 + 2;

    if (debug)
        fprintf(stderr, "Sending SUBSCRIBE packet...\n");

    return send_packet(sock, (char*)&packet, packet.length);
}
#endif
#if 1
void mqtt_sn_send_pingreq(struct mqtt_sn_connection *mqc)
{
    char packet[2];

    packet[0] = 2;
    packet[1] = MQTT_SN_TYPE_PINGREQ;

    if (debug) {
        printf("Sending ping...\n");
    }

    ctimer_restart(&(mqc->send_timer));


    return send_packet(mqc, (char*)&packet, 2);
}
#endif
#if 1
void mqtt_sn_send_pingresp(struct mqtt_sn_connection *mqc)
{
    char packet[2];

    packet[0] = 2;
    packet[1] = MQTT_SN_TYPE_PINGRESP;

    if (debug) {
        printf("Sending ping response...\n");
    }

    return send_packet(mqc, (char*)&packet, 2);
}
#endif
#if 1
void mqtt_sn_send_disconnect(struct mqtt_sn_connection *mqc)
{
    disconnect_packet_t packet;
    packet.type = MQTT_SN_TYPE_DISCONNECT;
    packet.length = 0x02;

    if (debug)
        printf("Sending DISCONNECT packet...\n");

    send_packet(mqc, (char*)&packet, packet.length);
    mqc->connection_retries=0;
    process_post(&mqtt_sn_process, disconnect_event, mqc);
}
#endif
#if 0
void mqtt_sn_recieve_connack(int sock)
{
    connack_packet_t *packet = recieve_packet(sock);

    if (packet == NULL) {
        fprintf(stderr, "Failed to connect to MQTT-S gateway.\n");
        return;
    }

    if (packet->type != MQTT_SN_TYPE_CONNACK) {
        fprintf(stderr, "Was expecting CONNACK packet but received: 0x%2.2x\n", packet->type);
        return;
    }

    // Check Connack return code
    if (debug)
        fprintf(stderr, "CONNACK return code: 0x%2.2x\n", packet->return_code);

    if (packet->return_code) {
        fprintf(stderr, "CONNECT error: %s\n", mqtt_sn_return_code_string(packet->return_code));
        exit(packet->return_code);
    }
}
#endif
#if 0
static int mqtt_sn_process_register(int sock, const register_packet_t *packet)
{
    int message_id = ntohs(packet->message_id);
    int topic_id = ntohs(packet->topic_id);
    const char* topic_name = packet->topic_name;

    // Add it to the topic map
    mqtt_sn_register_topic(topic_id, topic_name);

    // Respond to gateway with REGACK
    mqtt_sn_send_regack(sock, topic_id, message_id);

    return 0;
}
#endif
#if 0
void mqtt_sn_register_topic(int topic_id, const char* topic_name)
{
    topic_map_t **ptr = &topic_map;

    // Check topic ID is valid
    if (topic_id == 0x0000 || topic_id == 0xFFFF) {
        printf("Error: attempted to register invalid topic id: 0x%4.4x\n", topic_id);
        return;
    }

    // Check topic name is valid
    if (topic_name == NULL || strlen(topic_name) < 0) {
        printf("Error: attempted to register invalid topic name.\n");
        return;
    }

    if (debug)
        printf("Registering topic 0x%4.4x: %s\n", topic_id, topic_name);

    // Look for the topic id
    while (*ptr) {
        if ((*ptr)->topic_id == topic_id) {
            break;
        } else {
            ptr = &((*ptr)->next);
        }
    }

    // Allocate memory for a new entry, if we reached the end of the list
    if (*ptr == NULL) {
        *ptr = (topic_map_t *)malloc(sizeof(topic_map_t));
        if (!*ptr) {
            printf("Error: Failed to allocate memory for new topic map entry.\n");
            return;
        }
        (*ptr)->next = NULL;
    }

    // Copy in the name to the entry
    strncpy((*ptr)->topic_name, topic_name, MQTT_SN_MAX_TOPIC_LENGTH);
    (*ptr)->topic_id = topic_id;
}

const char* mqtt_sn_lookup_topic(int topic_id)
{
    topic_map_t **ptr = &topic_map;

    while (*ptr) {
        if ((*ptr)->topic_id == topic_id) {
            return (*ptr)->topic_name;
        }
        ptr = &((*ptr)->next);
    }

    fprintf(stderr, "Warning: failed to lookup topic id: 0x%4.4x\n", topic_id);
    return NULL;
}

uint16_t mqtt_sn_recieve_regack(int sock)
{
    regack_packet_t *packet = recieve_packet(sock);
    uint16_t received_message_id, received_topic_id;

    if (packet == NULL) {
        fprintf(stderr, "Failed to connect to register topic.\n");
        return;
    }

    if (packet->type != MQTT_SN_TYPE_REGACK) {
        fprintf(stderr, "Was expecting REGACK packet but received: 0x%2.2x\n", packet->type);
        exit(-1);
    }

    // Check Regack return code
    if (debug)
        fprintf(stderr, "REGACK return code: 0x%2.2x\n", packet->return_code);

    if (packet->return_code) {
        fprintf(stderr, "REGISTER error: %s\n", mqtt_sn_return_code_string(packet->return_code));
        exit(packet->return_code);
    }

    // Check that the Message ID matches
    received_message_id = ntohs( packet->message_id );
    if (received_message_id != next_message_id-1) {
        fprintf(stderr, "Warning: message id in Regack does not equal message id sent\n");
    }

    // Return the topic ID returned by the gateway
    received_topic_id = ntohs( packet->topic_id );
    if (debug)
        fprintf(stderr, "Topic ID: %d\n", received_topic_id);

    return received_topic_id;
}
#endif
#if 0
uint16_t mqtt_sn_recieve_suback(int sock)
{
    suback_packet_t *packet = recieve_packet(sock);
    uint16_t received_message_id, received_topic_id;

    if (packet == NULL) {
        fprintf(stderr, "Failed to subscribe to topic.\n");
        return;
    }

    if (packet->type != MQTT_SN_TYPE_SUBACK) {
        fprintf(stderr, "Was expecting SUBACK packet but received: 0x%2.2x\n", packet->type);
        exit(-1);
    }

    // Check Suback return code
    if (debug)
        fprintf(stderr, "SUBACK return code: 0x%2.2x\n", packet->return_code);

    if (packet->return_code) {
        fprintf(stderr, "SUBSCRIBE error: %s\n", mqtt_sn_return_code_string(packet->return_code));
        exit(packet->return_code);
    }

    // Check that the Message ID matches
    received_message_id = ntohs( packet->message_id );
    if (received_message_id != next_message_id-1) {
        fprintf(stderr, "Warning: message id in SUBACK does not equal message id sent\n");
        if (debug) {
            fprintf(stderr, "  Expecting: %d\n", next_message_id-1);
            fprintf(stderr, "  Actual: %d\n", received_message_id);
        }
    }

    // Return the topic ID returned by the gateway
    received_topic_id = ntohs( packet->topic_id );
    if (debug)
        fprintf(stderr, "Topic ID: %d\n", received_topic_id);

    return received_topic_id;
}
#endif

#if 1
const char* mqtt_sn_type_string(uint8_t type)
{
    switch(type) {
        case MQTT_SN_TYPE_ADVERTISE:     return "ADVERTISE";
        case MQTT_SN_TYPE_SEARCHGW:      return "SEARCHGW";
        case MQTT_SN_TYPE_GWINFO:        return "GWINFO";
        case MQTT_SN_TYPE_CONNECT:       return "CONNECT";
        case MQTT_SN_TYPE_CONNACK:       return "CONNACK";
        case MQTT_SN_TYPE_WILLTOPICREQ:  return "WILLTOPICREQ";
        case MQTT_SN_TYPE_WILLTOPIC:     return "WILLTOPIC";
        case MQTT_SN_TYPE_WILLMSGREQ:    return "WILLMSGREQ";
        case MQTT_SN_TYPE_WILLMSG:       return "WILLMSG";
        case MQTT_SN_TYPE_REGISTER:      return "REGISTER";
        case MQTT_SN_TYPE_REGACK:        return "REGACK";
        case MQTT_SN_TYPE_PUBLISH:       return "PUBLISH";
        case MQTT_SN_TYPE_PUBACK:        return "PUBACK";
        case MQTT_SN_TYPE_PUBCOMP:       return "PUBCOMP";
        case MQTT_SN_TYPE_PUBREC:        return "PUBREC";
        case MQTT_SN_TYPE_PUBREL:        return "PUBREL";
        case MQTT_SN_TYPE_SUBSCRIBE:     return "SUBSCRIBE";
        case MQTT_SN_TYPE_SUBACK:        return "SUBACK";
        case MQTT_SN_TYPE_UNSUBSCRIBE:   return "UNSUBSCRIBE";
        case MQTT_SN_TYPE_UNSUBACK:      return "UNSUBACK";
        case MQTT_SN_TYPE_PINGREQ:       return "PINGREQ";
        case MQTT_SN_TYPE_PINGRESP:      return "PINGRESP";
        case MQTT_SN_TYPE_DISCONNECT:    return "DISCONNECT";
        case MQTT_SN_TYPE_WILLTOPICUPD:  return "WILLTOPICUPD";
        case MQTT_SN_TYPE_WILLTOPICRESP: return "WILLTOPICRESP";
        case MQTT_SN_TYPE_WILLMSGUPD:    return "WILLMSGUPD";
        case MQTT_SN_TYPE_WILLMSGRESP:   return "WILLMSGRESP";
        default:                       return "UNKNOWN";
    }
}
#endif
#if 1
const char* mqtt_sn_return_code_string(uint8_t return_code)
{
    switch(return_code) {
        case 0x00: return "Accepted";
        case 0x01: return "Rejected: congestion";
        case 0x02: return "Rejected: invalid topic ID";
        case 0x03: return "Rejected: not supported";
        default:   return "Rejected: unknown reason";
    }
}
#endif
#if 0
void mqtt_sn_cleanup()
{
    topic_map_t **ptr = &topic_map;
    topic_map_t **ptr2 = NULL;

    // Walk through the topic map, deleting each entry
    while (*ptr) {
        ptr2 = ptr;
        ptr = &((*ptr)->next);
        free(*ptr2);
        *ptr2 = NULL;
    }
}
#endif
#if 1
int mqtt_sn_request_returned(struct mqtt_sn_request *req) {
  if (req->state == MQTTSN_REQUEST_COMPLETE || req->state == MQTTSN_REQUEST_FAILED) { return 1;}
  else {return 0;}
}

int mqtt_sn_request_success(struct mqtt_sn_request *req) {
  if (req->state == MQTTSN_REQUEST_COMPLETE) {return 1;}
  else {return 0;}
}

void
manage_response(struct mqtt_sn_connection *mqc, const uip_ipaddr_t *source_addr,
                 const uint8_t *data, uint16_t datalen)
{
  uint8_t msg_type;
  uint16_t msg_id = 0;
  uint8_t return_code = 0;
  uint16_t topic_id = 0;
  struct mqtt_sn_request *req;
  msg_type = data[1];
  if (msg_type == MQTT_SN_TYPE_REGACK) {
    regack_packet_t incoming_regack;
    memcpy(&incoming_regack, data, datalen);
    msg_id = incoming_regack.message_id;
    return_code = incoming_regack.return_code;
    topic_id = uip_htons(incoming_regack.topic_id);
  }
  else if ( msg_type == MQTT_SN_TYPE_SUBACK) {
    suback_packet_t incoming_suback;
    memcpy(&incoming_suback, data, datalen);
    msg_id = incoming_suback.message_id;
    return_code = incoming_suback.return_code;
    topic_id = uip_htons(incoming_suback.topic_id);
  }
  for(req = list_head(mqc->requests); req != NULL; req = req->next) {
    if(req->msg_id == msg_id) {
      req->topic_id = topic_id;
      req->return_code = return_code;
      if (return_code == 0){
        req->state = MQTTSN_REQUEST_COMPLETE;
      }
      else {
        req->state = MQTTSN_REQUEST_FAILED;
      }
      manage_request(req,mqc,NULL,0,0);
    }
  }
}

static void request_timer_callback(void *req)
{
  struct mqtt_sn_request *req2 = (struct mqtt_sn_request *)req;
  manage_request(req2,NULL,NULL,0,0);
}

static int
manage_request(struct mqtt_sn_request *req, struct mqtt_sn_connection *mqc,
               const char* topic_name, uint8_t qos,clock_time_t time_out)
{
  PT_BEGIN(&(req->pt));
  list_add(mqc->requests,req);
  ctimer_set(&(req->t), time_out, request_timer_callback, req);
  req->state = MQTTSN_REQUEST_WAITING_ACK;
  if (req->request_type == MQTTSN_REGISTER_REQUEST) {
    req->msg_id = mqtt_sn_send_register(mqc, topic_name);
  }
  if (req->request_type == MQTTSN_SUBSCRIBE_REQUEST) {
    req->msg_id = mqtt_sn_send_subscribe(mqc, topic_name, qos);
  }
  PT_YIELD(&(req->pt)); /* Wait until timer expired or response received */
  ctimer_stop(&(req->t));
  process_post(PROCESS_BROADCAST,mqtt_sn_request_event,req);
  list_remove(mqc->requests,req);
  PT_END(&(req->pt));
  return 1;
}

uint16_t mqtt_sn_register_try(mqtt_sn_register_request *req, struct mqtt_sn_connection *mqc,
                              const char* topic_name,clock_time_t time_out) {
  req->request_type = MQTTSN_REGISTER_REQUEST;
  manage_request(req,mqc,topic_name,0,time_out);
  return req->msg_id;
}

uint16_t mqtt_sn_subscribe_try(mqtt_sn_subscribe_request *req, struct mqtt_sn_connection *mqc,
                               const char* topic_name, uint8_t qos, clock_time_t time_out) {
  req->request_type = MQTTSN_SUBSCRIBE_REQUEST;
  manage_request(req,mqc,topic_name,qos,time_out);
  return req->msg_id;
}

#endif // 1


