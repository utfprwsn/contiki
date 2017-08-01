all: example
PROJECT_SOURCEFILES += mqtt-sn.c

WITH_UIP6=1
UIP_CONF_IPV6=1
CFLAGS+= -DUIP_CONF_IPV6_RPL
CFLAGS += -DPROJECT_CONF_H=\"project-conf.h\"
CFLAGS += -g

CONTIKI=../..
include $(CONTIKI)/Makefile.include