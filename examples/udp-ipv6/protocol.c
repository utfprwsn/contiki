/*
 * protocol.c
 *
 *  Created on: Sep 28, 2017
 *      Author: mint
 */

#include "protocol.h"
#define DEBUG DEBUG_PRINT
#include "net/ip/uip-debug.h"


/*---------------------------------------------------------------------------*/
void printRequest(struct mathopreq *req)
{
    int32_t intPart;
    uint32_t fracPart;
    intPart = (int32_t)req->fc;
    fracPart = ABS_P((int32_t)((req->fc - intPart)*10000));
    PRINTF("(%ld%s%ld)*%ld.%lu",req->op1,operator(req->operation),req->op2,intPart,fracPart);
}

void printReply(struct mathopreply *req)
{
    int i=0;
    int32_t intPart;
    uint32_t fracPart;
    intPart = (int32_t)req->fpResult;
    uint8_t* buffer = (uint8_t*)req;
    fracPart = ABS_P((int32_t)((req->fpResult - intPart)*10000));
    PRINTF("%ld.%lu (%ld.%lu): ",req->intPart,req->fracPart,intPart,fracPart);
    uint8_t crc=0;
    for(i=0;i<sizeof(struct mathopreply)-1;i++)
    {
        crc+=buffer[i];
    }
    PRINTF("CRC calc: 0x%x, exp: 0x%x -> %s ",crc,req->crc,crc==req->crc?"OK":"ERR");
}

char * operator(uint8_t op)
{
    switch(op)
    {
        case OP_SUM:
            return "+";
        case OP_SUBTRACT:
            return "-";
        case OP_MULTIPLY:
            return "*";
        case OP_DIVIDE:
            return "/";
        default: return "?";
    }
}
