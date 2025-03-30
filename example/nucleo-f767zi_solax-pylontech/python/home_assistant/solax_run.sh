#!/bin/bash

while [ true ]
do
	python3 solax_bms_mqtt.py /dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066FFF544856846687084917-if02
	sleep 0.5
done

