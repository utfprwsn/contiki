#!/bin/bash
echo $1
/projects/contiki/tools/tunslip6 -s $1 aaaa::1/64
