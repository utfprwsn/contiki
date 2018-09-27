/*
 * Copyright (c) 2015, SICS Swedish ICT.
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

#ifndef __TSCH_H__
#define __TSCH_H__

/********** Includes **********/

#include "contiki.h"
#include "net/mac/mac.h"
#include "net/mac/tsch/tsch-security.h"

#define TX_INIT 1
#define TS_TX_OFFSET 2
#define TS_TX_OFFSET_AFTER_TRANSMIT 3
#define TS_RX_ACK_DELAY 4
#define TS_ACK_WAIT 5
#define ACK_RECEIVED 6
#define RADIO_OFF_AFTER_ACK_RECEIVED 7
#define RADIO_OFF_END_TX_SLOT 8
#define TX_END 9
#define RX_INIT (0x0A)
#define TS_RX_OFFSET (0x0B)
#define RX_IDLE (0x0C)
#define RX_IDLE_RX_OFF (0x0D)
#define PACKET_DETECTED (0x0E)
#define PACKET_RECEIVED (0x0F)
#define RX_OFF_AFTER_PACKET_RECEIVED (0x10)
#define RX_ACK_SEND (0x11)
#define RX_END (0x12)
#define SLOT_START (0x13)
#define SLOT_START_TURN_RADIO_ON (0x14)
#define SLOT_START_RADIO_IS_ON (0x15)
#define SLOT_END (0x16)
#define SLOT_SCHEDULE (0x17)
#define SLOT_OPERATION_END (0x18)


#define TSCH_DEBUG(x) do {                   \
    uint32_t j=0,k[5]={0,1,12,15,21};                               \
    for(uint32_t i=0; i<5; i++) {                                               \
      if(1<<i & x) {            \
        j |= 1<<k[i];                                           \
      }                                                         \
    }                                                           \
    ti_lib_gpio_clear_multi_dio(1<<IOID_0 | 1<<IOID_1 | 1<<IOID_12 | 1<<IOID_15 | 1<<IOID_21); \
    ti_lib_gpio_set_multi_dio(j); \
  } while(0)




/******** Configuration *******/

/* Max time before sending a unicast keep-alive message to the time source */
#ifdef TSCH_CONF_KEEPALIVE_TIMEOUT
#define TSCH_KEEPALIVE_TIMEOUT TSCH_CONF_KEEPALIVE_TIMEOUT
#else
/* Time to desynch assuming a drift of 40 PPM (80 PPM between two nodes) and guard time of +/-1ms: 12.5s. */
#define TSCH_KEEPALIVE_TIMEOUT (12 * CLOCK_SECOND)
#endif

/* With TSCH_ADAPTIVE_TIMESYNC enabled: keep-alive timeout used after reaching
 * accurate drift compensation. */
#ifdef TSCH_CONF_MAX_KEEPALIVE_TIMEOUT
#define TSCH_MAX_KEEPALIVE_TIMEOUT TSCH_CONF_MAX_KEEPALIVE_TIMEOUT
#else
#define TSCH_MAX_KEEPALIVE_TIMEOUT (60 * CLOCK_SECOND)
#endif

/* Max time without synchronization before leaving the PAN */
#ifdef TSCH_CONF_DESYNC_THRESHOLD
#define TSCH_DESYNC_THRESHOLD TSCH_CONF_DESYNC_THRESHOLD
#else
#define TSCH_DESYNC_THRESHOLD (2 * TSCH_MAX_KEEPALIVE_TIMEOUT)
#endif

/* Period between two consecutive EBs */
#ifdef TSCH_CONF_EB_PERIOD
#define TSCH_EB_PERIOD TSCH_CONF_EB_PERIOD
#else
#define TSCH_EB_PERIOD (16 * CLOCK_SECOND)
#endif

/* Max Period between two consecutive EBs */
#ifdef TSCH_CONF_MAX_EB_PERIOD
#define TSCH_MAX_EB_PERIOD TSCH_CONF_MAX_EB_PERIOD
#else
#define TSCH_MAX_EB_PERIOD (50 * CLOCK_SECOND)
#endif

/* Max acceptable join priority */
#ifdef TSCH_CONF_MAX_JOIN_PRIORITY
#define TSCH_MAX_JOIN_PRIORITY TSCH_CONF_MAX_JOIN_PRIORITY
#else
#define TSCH_MAX_JOIN_PRIORITY 32
#endif

/* Start TSCH automatically after init? If not, the upper layers
 * must call NETSTACK_MAC.on() to start it. Useful when the
 * application needs to control when the nodes are to start
 * scanning or advertising.*/
#ifdef TSCH_CONF_AUTOSTART
#define TSCH_AUTOSTART TSCH_CONF_AUTOSTART
#else
#define TSCH_AUTOSTART 1
#endif

/* Join only secured networks? (discard EBs with security disabled) */
#ifdef TSCH_CONF_JOIN_SECURED_ONLY
#define TSCH_JOIN_SECURED_ONLY TSCH_CONF_JOIN_SECURED_ONLY
#else
/* By default, set if LLSEC802154_ENABLED is also non-zero */
#define TSCH_JOIN_SECURED_ONLY LLSEC802154_ENABLED
#endif

/* By default, join any PAN ID. Otherwise, wait for an EB from IEEE802154_PANID */
#ifdef TSCH_CONF_JOIN_MY_PANID_ONLY
#define TSCH_JOIN_MY_PANID_ONLY TSCH_CONF_JOIN_MY_PANID_ONLY
#else
#define TSCH_JOIN_MY_PANID_ONLY 0
#endif

/* The radio polling frequency (in Hz) during association process */
#ifdef TSCH_CONF_ASSOCIATION_POLL_FREQUENCY
#define TSCH_ASSOCIATION_POLL_FREQUENCY TSCH_CONF_ASSOCIATION_POLL_FREQUENCY
#else
#define TSCH_ASSOCIATION_POLL_FREQUENCY 500
#endif

/* When associating, check ASN against our own uptime (time in minutes)..
 * Useful to force joining only with nodes started roughly at the same time.
 * Set to the max number of minutes acceptable. */
#ifdef TSCH_CONF_CHECK_TIME_AT_ASSOCIATION
#define TSCH_CHECK_TIME_AT_ASSOCIATION TSCH_CONF_CHECK_TIME_AT_ASSOCIATION
#else
#define TSCH_CHECK_TIME_AT_ASSOCIATION 0
#endif

/* By default: initialize schedule from EB when associating, using the
 * slotframe and links Information Element */
#ifdef TSCH_CONF_INIT_SCHEDULE_FROM_EB
#define TSCH_INIT_SCHEDULE_FROM_EB TSCH_CONF_INIT_SCHEDULE_FROM_EB
#else
#define TSCH_INIT_SCHEDULE_FROM_EB 1
#endif

/* An ad-hoc mechanism to have TSCH select its time source without the
 * help of an upper-layer, simply by collecting statistics on received
 * EBs and their join priority. Disabled by default as we recomment
 * mapping the time source on the RPL preferred parent
 * (via tsch_rpl_callback_parent_switch) */
#ifdef TSCH_CONF_AUTOSELECT_TIME_SOURCE
#define TSCH_AUTOSELECT_TIME_SOURCE TSCH_CONF_AUTOSELECT_TIME_SOURCE
#else
#define TSCH_AUTOSELECT_TIME_SOURCE 0
#endif /* TSCH_CONF_EB_AUTOSELECT */

/*********** Callbacks *********/

/* Called by TSCH when joining a network */
#ifdef TSCH_CALLBACK_JOINING_NETWORK
void TSCH_CALLBACK_JOINING_NETWORK();
#endif

/* Called by TSCH when leaving a network */
#ifdef TSCH_CALLBACK_LEAVING_NETWORK
void TSCH_CALLBACK_LEAVING_NETWORK();
#endif

/***** External Variables *****/

/* Are we coordinator of the TSCH network? */
extern int tsch_is_coordinator;
/* Are we associated to a TSCH network? */
extern int tsch_is_associated;
/* Is the PAN running link-layer security? */
extern int tsch_is_pan_secured;
/* The TSCH MAC driver */
extern const struct mac_driver tschmac_driver;

/********** Functions *********/

/* The the TSCH join priority */
void tsch_set_join_priority(uint8_t jp);
/* The period at which EBs are sent */
void tsch_set_eb_period(uint32_t period);
/* The keep-alive timeout */
void tsch_set_ka_timeout(uint32_t timeout);
/* Set the node as PAN coordinator */
void tsch_set_coordinator(int enable);
/* Set the pan as secured or not */
void tsch_set_pan_secured(int enable);

#endif /* __TSCH_H__ */
