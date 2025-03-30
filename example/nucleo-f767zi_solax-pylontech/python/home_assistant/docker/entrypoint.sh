#!/bin/bash

set -e

export PYTHONPATH=/:.

STLINK_SERIAL=066FFF544856846687084917
SERIAL_PATH=/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_${STLINK_SERIAL}-if02 #via usb nucleo

while [ true ] 
do
	echo "Starting solax bms mqtt bridge..."
	python3 solax_bms_mqtt.py ${SERIAL_PATH}
	sleep 0.1
done 
