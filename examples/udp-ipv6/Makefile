#all: udp-server udp-client udp-server-atividade4
all: udp-server-atividade4 udp-client-atividade4 udp-client-atividade4-gabarito udp-client-atividadeAdvanced udp-server-atividadeAdvanced udp-client-rpl-localization-anchor
#all: udp-server-RSSI-LQI udp-client-RSSI-LQI

CFLAGS += -DPROJECT_CONF_H=\"project-conf.h\"
CONTIKI = ../..
CONTIKI_WITH_IPV6 = 1
CFLAGS += -DUIP_CONF_ND6_SEND_NS=1
CFLAGS += -g
DEBUG = 1
APPS += powertrace
PROJECT_SOURCEFILES += protocol.c cetic-6lbr-client.c 

include $(CONTIKI)/Makefile.include
