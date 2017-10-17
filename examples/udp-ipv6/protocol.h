/*
 * protocol.h
 *
 *  Created on: Sep 28, 2017
 *      Author: mint
 */

#ifndef PROTOCOL_H_
#define PROTOCOL_H_

#include <stdbool.h>
#include <stdint.h>

#define ABS_P(x) (x>0?x:-1*x)

#define LED_TOGGLE_REQUEST (0x79)
#define LED_SET_STATE (0x7A)
#define LED_GET_STATE (0x7B)
#define LED_STATE (0x7C)
#define CONN_PORT (8802)

#define OP_SUBTRACT (0x25)
#define OP_RESULT (0x6F)
#define OP_REQUEST (0x6E)
#define OP_MULTIPLY (0x22)
#define OP_DIVIDE (0x23)
#define OP_SUM (0x24)


struct mathopreq {
  uint8_t opRequest;
  int32_t op1;
  uint8_t operation;
  int32_t op2;
  float fc;
}  __attribute__((packed));

struct mathopreply {
  uint8_t opResult;
  int32_t intPart;
  uint32_t fracPart;
  float fpResult;
  uint8_t crc;
}  __attribute__((packed));

char * operator(uint8_t op);
void printRequest(struct mathopreq *req);
void printReply(struct mathopreply *req);


#endif /* PROTOCOL_H_ */
