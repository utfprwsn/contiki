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

#ifndef MQTT_SN_H
#define MQTT_SN_H

#include "simple-udp.h"
#include "clock.h"
#include "etimer.h"
#include "ctimer.h"
#include "list.h"


#ifndef FALSE
#define FALSE  (0)
#endif

#ifndef TRUE
#define TRUE  (1)
#endif

#ifndef MQTT_SN_MAX_CONNECT_RETRIES
#define MQTT_SN_MAX_CONNECT_RETRIES 3
#endif

#define MQTT_SN_MAX_PACKET_LENGTH  (255)
#define MQTT_SN_MAX_TOPIC_LENGTH   (MQTT_SN_MAX_PACKET_LENGTH-6)

#define MQTT_SN_TYPE_ADVERTISE     (0x00)
#define MQTT_SN_TYPE_SEARCHGW      (0x01)
#define MQTT_SN_TYPE_GWINFO        (0x02)
#define MQTT_SN_TYPE_CONNECT       (0x04)
#define MQTT_SN_TYPE_CONNACK       (0x05)
#define MQTT_SN_TYPE_WILLTOPICREQ  (0x06)
#define MQTT_SN_TYPE_WILLTOPIC     (0x07)
#define MQTT_SN_TYPE_WILLMSGREQ    (0x08)
#define MQTT_SN_TYPE_WILLMSG       (0x09)
#define MQTT_SN_TYPE_REGISTER      (0x0A)
#define MQTT_SN_TYPE_REGACK        (0x0B)
#define MQTT_SN_TYPE_PUBLISH       (0x0C)
#define MQTT_SN_TYPE_PUBACK        (0x0D)
#define MQTT_SN_TYPE_PUBCOMP       (0x0E)
#define MQTT_SN_TYPE_PUBREC        (0x0F)
#define MQTT_SN_TYPE_PUBREL        (0x10)
#define MQTT_SN_TYPE_SUBSCRIBE     (0x12)
#define MQTT_SN_TYPE_SUBACK        (0x13)
#define MQTT_SN_TYPE_UNSUBSCRIBE   (0x14)
#define MQTT_SN_TYPE_UNSUBACK      (0x15)
#define MQTT_SN_TYPE_PINGREQ       (0x16)
#define MQTT_SN_TYPE_PINGRESP      (0x17)
#define MQTT_SN_TYPE_DISCONNECT    (0x18)
#define MQTT_SN_TYPE_WILLTOPICUPD  (0x1A)
#define MQTT_SN_TYPE_WILLTOPICRESP (0x1B)
#define MQTT_SN_TYPE_WILLMSGUPD    (0x1C)
#define MQTT_SN_TYPE_WILLMSGRESP   (0x1D)

#define MQTT_SN_TOPIC_TYPE_NORMAL     (0x00)
#define MQTT_SN_TOPIC_TYPE_PREDEFINED (0x01)
#define MQTT_SN_TOPIC_TYPE_SHORT      (0x02)


#define MQTT_SN_FLAG_DUP     (0x1 << 7)
#define MQTT_SN_FLAG_QOS_0   (0x0 << 5)
#define MQTT_SN_FLAG_QOS_1   (0x1 << 5)
#define MQTT_SN_FLAG_QOS_2   (0x2 << 5)
#define MQTT_SN_FLAG_QOS_N1  (0x3 << 5)
#define MQTT_SN_FLAG_RETAIN  (0x1 << 4)
#define MQTT_SN_FLAG_WILL    (0x1 << 3)
#define MQTT_SN_FLAG_CLEAN   (0x1 << 2)

#define MQTT_SN_PROTOCOL_ID  (0x01)

enum mqtt_sn_return_code {
  ACCEPTED = 0,
  REGECTED_CONGESTION,
  REJECTED_INVALID_TOPIC_ID,
  REJECTED_NOT_SUPPORTED
};

typedef struct {
  uint8_t length;
  uint8_t type;
  uint8_t flags;
  uint8_t protocol_id;
  uint16_t duration;
  char client_id[23];
} connect_packet_t;

typedef struct {
  uint8_t length;
  uint8_t type;
  uint8_t return_code;
} connack_packet_t;

typedef struct {
  uint8_t length;
  uint8_t type;
  uint16_t topic_id;
  uint16_t message_id;
  char topic_name[MQTT_SN_MAX_TOPIC_LENGTH];
} register_packet_t;

typedef struct {
  uint8_t length;
  uint8_t type;
  uint16_t topic_id;
  uint16_t message_id;
  uint8_t return_code;
} regack_packet_t;

typedef struct __attribute__((packed)) {
  uint8_t length;
  uint8_t type;
  uint8_t flags;
  uint16_t topic_id;
  uint16_t message_id;
  char data[MQTT_SN_MAX_PACKET_LENGTH-7];
} publish_packet_t;

typedef struct {
  uint8_t length;
  uint8_t type;
  uint16_t topic_id;
  uint16_t message_id;
  uint8_t return_code;
} puback_packet_t;

typedef struct __attribute__((packed)) {
  uint8_t length;
  uint8_t type;
  uint8_t flags;
  uint16_t message_id;
  union {
      char topic_name[MQTT_SN_MAX_TOPIC_LENGTH];
      uint16_t topic_id;
  };
} subscribe_packet_t;

typedef struct __attribute__((packed)) {
  uint8_t length;
  uint8_t type;
  uint8_t flags;
  uint16_t topic_id;
  uint16_t message_id;
  uint8_t return_code;
} suback_packet_t;

typedef struct {
  uint8_t length;
  uint8_t type;
  uint16_t duration;
} disconnect_packet_t;

typedef struct topic_map {
  uint16_t topic_id;
  char topic_name[MQTT_SN_MAX_TOPIC_LENGTH];
  struct topic_map *next;
} topic_map_t;

struct mqtt_sn_connection;
struct mqtt_sn_callbacks {
  /** Called when a packet has been received by the mqtt_sn module or other event needs handled */
  void (* pub_recv)(struct mqtt_sn_connection *mqc, const uip_ipaddr_t *source_addr, const uint8_t *data, uint16_t datalen);
  void (* pingreq_recv)(struct mqtt_sn_connection *mqc, const uip_ipaddr_t *source_addr, const uint8_t *data, uint16_t datalen);
  void (* pingresp_recv)(struct mqtt_sn_connection *mqc, const uip_ipaddr_t *source_addr, const uint8_t *data, uint16_t datalen);
  void (* connack_recv)(struct mqtt_sn_connection *mqc, const uip_ipaddr_t *source_addr, const uint8_t *data, uint16_t datalen);
  void (* regack_recv)(struct mqtt_sn_connection *mqc, const uip_ipaddr_t *source_addr, const uint8_t *data, uint16_t datalen);
  void (* puback_recv)(struct mqtt_sn_connection *mqc, const uip_ipaddr_t *source_addr, const uint8_t *data, uint16_t datalen);
  void (* suback_recv)(struct mqtt_sn_connection *mqc, const uip_ipaddr_t *source_addr, const uint8_t *data, uint16_t datalen);
  void (* disconnect_recv)(struct mqtt_sn_connection *mqc, const uip_ipaddr_t *source_addr, const uint8_t *data, uint16_t datalen);
  void (* keepalive_timeout)(struct mqtt_sn_connection *mqc);
};


enum mqttsn_connection_status
{
  MQTTSN_DISCONNECTED =0,
  MQTTSN_WAITING_CONNACK,
  MQTTSN_CONNECTION_FAILED,
  MQTTSN_CONNECTED
};

enum topic_status {
  UNREGISTERED=0,
  WAITING_REGACK,
  REGISTERED
};

struct mqtt_sn_connection {
  struct simple_udp_connection sock;
  //simple udp receive callback, host, port will be registered with sock
  uint16_t next_message_id;
  //the keep alive mechanisms could be replaced with event timers
  struct ctimer receive_timer;
  struct ctimer send_timer;
  clock_time_t keep_alive;
  connack_packet_t last_connack;
  const char* client_id;
  enum mqttsn_connection_status stat;
  uint8_t connection_retries;
  struct process *client_process;
  const struct mqtt_sn_callbacks *mc;
  LIST_STRUCT(requests);
};

struct mqttsn_topic {
  uint16_t topic_id;
  char topic_name[MQTT_SN_MAX_TOPIC_LENGTH];
};

#endif

// Library functions
#if 1
int mqtt_sn_create_socket(struct mqtt_sn_connection *mqc, uint16_t local_port, uip_ipaddr_t *remote_addr, uint16_t remote_port);
#endif
#if 1
void mqtt_sn_send_connect(struct mqtt_sn_connection *mqc, const char* client_id, uint16_t keepalive);
#endif
#if 1
uint16_t mqtt_sn_send_register(struct mqtt_sn_connection *mqc, const char* topic_name);
#endif
#if 1
uint16_t mqtt_sn_send_publish(struct mqtt_sn_connection *mqc, uint16_t topic_id, uint8_t topic_type, const char* data, uint16_t data_len, int8_t qos, uint8_t retain);
#endif
#if 1
uint16_t mqtt_sn_send_subscribe(struct mqtt_sn_connection *mqc, const char* topic_name, uint8_t qos);
uint16_t mqtt_sn_send_subscribe_topic_id(struct mqtt_sn_connection *mqc, uint16_t topic_id, uint8_t qos);
#endif
#if 1
void mqtt_sn_send_pingreq(struct mqtt_sn_connection *mqc);
#endif
#if 1
void mqtt_sn_send_pingresp(struct mqtt_sn_connection *mqc);
#endif
#if 1
void mqtt_sn_send_disconnect(struct mqtt_sn_connection *mqc);
#endif
#if 1
const char* mqtt_sn_type_string(uint8_t type);
const char* mqtt_sn_return_code_string(uint8_t return_code);
void mqtt_sn_set_debug(uint8_t value);
#endif // 1
#if 0
void mqtt_sn_recieve_connack(struct mqtt_sn_connection *mqc);
#endif
#if 0
uint16_t mqtt_sn_recieve_regack(struct mqtt_sn_connection *mqc);
#endif
#if 0
uint16_t mqtt_sn_recieve_suback(struct mqtt_sn_connection *mqc);
#endif
#if 0
publish_packet_t* mqtt_sn_loop(int sock, int timeout);
#endif
#if 0
void mqtt_sn_register_topic(int topic_id, const char* topic_name);
const char* mqtt_sn_lookup_topic(int topic_id);
void mqtt_sn_cleanup();
#endif

#if 1
/*MQTT SN Request is an abstraction of messages that require a response
* from the broker to complete a negotiated connection
* e.g. subscription, registration, unsubscription
*/

extern process_event_t mqtt_sn_request_event;

enum mqtt_sn_request_state
{
  MQTTSN_NO_REQUEST = 0,
  MQTTSN_REQUEST_WAITING_ACK,
  MQTTSN_REQUEST_FAILED,
  MQTTSN_REQUEST_COMPLETE
};

enum mqtt_sn_request_type
{
  MQTTSN_REGISTER_REQUEST = 0,
  MQTTSN_SUBSCRIBE_REQUEST,
  MQTTSN_UNSUBSCRIBE_REQUEST
};

struct mqtt_sn_request
{
  struct mqtt_sn_request *next;
  enum mqtt_sn_request_state state;
  enum mqtt_sn_request_type request_type;
  uint16_t msg_id;
  uint16_t topic_id;
  uint8_t return_code;
  struct pt pt;
  struct ctimer t;
};

typedef struct mqtt_sn_request mqtt_sn_register_request;
typedef struct mqtt_sn_request mqtt_sn_subscribe_request;
//typedef struct mqtt_sn_request mqtt_sn_unsubscribe_request;

//uint16_t mqtt_sn_request_try(struct mqtt_sn_connection *mqc,struct mqtt_sn_request *req, void (*message_function)(void *msg_fct), const char* topic_name,uint8_t time_out, uint8_t retries);
int mqtt_sn_request_returned(struct mqtt_sn_request *req);
int mqtt_sn_request_success(struct mqtt_sn_request *req);

uint16_t mqtt_sn_register_try(mqtt_sn_register_request *req, struct mqtt_sn_connection *mqc, const char* topic_name,clock_time_t time_out);

uint16_t mqtt_sn_subscribe_try(mqtt_sn_subscribe_request *req, struct mqtt_sn_connection *mqc, const char* topic_name, uint8_t qos, clock_time_t time_out);

//uint16_t mqtt_sn_unsubscribe_try(mqtt_sn_usubscribe_request *req; struct mqtt_sn_connection *mqc, const char* topic_name,uint8_t time_out, uint8_t retries);

#endif // 1






