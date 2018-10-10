all: udp-client-rpl-localization example
#all: example

WITH_UIP6=1
UIP_CONF_IPV6=1
CFLAGS+= -DUIP_CONF_IPV6_RPL
CONTIKI_WITH_IPV6 = 1
CFLAGS += -DUIP_CONF_ND6_SEND_NS=1
CFLAGS += -DPROJECT_CONF_H=\"project-conf.h\"
CFLAGS += -g

CONTIKI=../..
PROJECT_SOURCEFILES += mqtt-sn.c cetic-6lbr-client.c

include $(CONTIKI)/Makefile.include
