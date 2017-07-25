#!/bin/bash
echo "Script utilizado quando o numero de devices ACM ultrapassa o limite, resetando o modulo de driver acm"
cd /lib/modules/3.16.0-4-amd64/
rmmod -f cdc_acm
lsmod | grep acm # make sure it's not there
insmod ./kernel/drivers/usb/class/cdc-acm.ko






