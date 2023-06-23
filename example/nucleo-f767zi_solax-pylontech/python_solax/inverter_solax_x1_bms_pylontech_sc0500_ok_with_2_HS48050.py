"""
*******************************************************************************
*   IOBridge python
*   (c) 2022 Olivier TOMAZ
*
*  Licensed under the Apache License, Version 2.0 (the "License");
*  you may not use this file except in compliance with the License.
*  You may obtain a copy of the License at
*
*      http://www.apache.org/licenses/LICENSE-2.0
*
*  Unless required by applicable law or agreed to in writing, software
*  distributed under the License is distributed on an "AS IS" BASIS,
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*  See the License for the specific language governing permissions and
*  limitations under the License.
********************************************************************************
"""

if __name__ == '__main__':
	import serial
	import argparse
	import sys
	import binascii
	import random
	import time
	import math
	import traceback
	from iobridge.iobridge import IOBridge, IOBridgeException
	import logging
	import os
	import struct

	LOGLEVEL = os.environ.get('LOGLEVEL', 'DEBUG').upper()
	LOGFORMAT = '%(asctime)s %(levelname)s %(threadName)s %(message)s'
	logging.basicConfig(level=LOGLEVEL, format=LOGFORMAT)

	log = logging.getLogger("")


	def auto_int(x):
		return int(x, 0)

	parser = argparse.ArgumentParser(description="CAN bus man-in-the-middle example")
	parser.add_argument("--port", default="/dev/ttyACM0", help="""IO bridge interface to the SOLAX""")
	parser.add_argument("--baudrate", default="921600", help="IOBridge USART speed", type=auto_int)
	parser.add_argument("--port2", default="/dev/ttyACM1", help="""IO bridge interface to the Pylontech""")
	parser.add_argument("--baudrate2", default="921600", help="IOBridge USART speed", type=auto_int)
	args = parser.parse_args()

	s = serial.Serial(port=args.port, baudrate=args.baudrate)
	solax = IOBridge(s)

	s2 = serial.Serial(port=args.port2, baudrate=args.baudrate2)
	pylon = IOBridge(s2)

	nextwakeuptime = time.time() + 10
	pylon.can_tx(0x35C, 'e', b'\xC0\x00')
	bms_adopted = False
	bms_counter = 0
	current = 0

	#flush background messages
	while solax.can_available() > 0:
		solax.can_rx()
	while pylon.can_available() > 0:
		pylon.can_rx()

	#start transmission
	while True:
		try:
			if solax.can_available() > 0:
				cid, cidkind, data = solax.can_rx()
				log.info("solax> " + hex(cid) + " " + cidkind + " " + binascii.hexlify(data).decode("utf8"))
				# INV_request broadcast?
				if cid == 0x1871 and data[0] == 0x01:
					if not bms_adopted:
						# bms_announce
						log.info(">solax " + hex(0x0100A000) + " " + 'e' + " " + binascii.hexlify(b'').decode("utf8"))
						solax.can_tx(0x0100A000, 'e', b'')
						#reply with received data
						log.info(">solax " + hex(0x1801) + " " + 'e' + " " + binascii.hexlify(data).decode("utf8"))
						solax.can_tx(0x1801, 'e', data)
						#bms_adopted=True
				if cid == 0x1871 and data[0] == 0x03:
					log.info("+ adoption request")
					datarep = b'\x02\x00\x01\x00\x01\x00\x00\x00'
					log.info(">solax " + hex(0x1801) + " " + 'e' + " " + binascii.hexlify(datarep).decode("utf8"))
					solax.can_tx(0x1801, 'e', datarep)
					bms_adopted = True
				# detect disconnection
				if cid == 0x1871 and data[0] == 0x02:
					bms_adopted=False
				log.info(">pylon " + hex(cid) + " " + cidkind + " " + binascii.hexlify(data).decode("utf8"))
				pylon.can_tx(cid, cidkind, data)
		except IOBridgeException as e:
			traceback.print_exc()

		try:
			if pylon.can_available() > 0:
				cid, cidkind, data = pylon.can_rx()
				log.info("pylon> " + hex(cid) + " " + cidkind + " " + binascii.hexlify(data).decode("utf8"))
				if cid == 0x1872:
					vmax, vmin, ich, idisch = struct.unpack_from("<hhhh", data)
					# fake the max charge and discharge currents
					# OTO: this is taken into account, the inverter respects it
					data = struct.pack("<hhhh", 400*10, 80*10, 5*10, 5*10)
				if cid == 0x1873:
					vcur, icur = struct.unpack_from("<hh", data)
				## wipe BMS_PackData Current
				#if cid == 0x1873:
				#	data = data [0:2] + b'\x00\x00' + data[4:8]
				#	#data = b'\x00\x00\x00\x00' + data[4:8]
				#	log.info("~ changed: " + hex(cid) + " " + cidkind + " " + binascii.hexlify(data).decode("utf8"))
				if cid == 0x1877:
					#data = data[0:4] + b'\x52' + data[5:6]
					# data = b'\x00\x00\x00\x00\x52\x00'
					# if bms_counter == 0:
					# 	data += b'\x00\x02'
					# else:
					# 	data += b'\x22' + struct.pack(">B", bms_counter * 16)
					# bms_counter=(bms_counter+1)%5
					# fixed value
					data = b'\x00\x00\x00\x00\x52\x00\x22\x40'
					#log.info("~ changed: " + hex(cid) + " " + cidkind + " " + binascii.hexlify(data).decode("utf8"))
				#if cid in (0x1872, 0x1874, 0x1875, 0x1876, 0x1877):
				forward=True
				if bms_adopted:
					if cid in (0x1872, 0x1875):
						forward = True
				else: 
					if cid in (0x1872, 0x1873, 0x1874, 0x1875, 0x1876, 0x1877):
						forward = True
				if forward:
					log.info(">solax " + hex(cid) + " " + cidkind + " " + binascii.hexlify(data).decode("utf8"))
					solax.can_tx(cid, cidkind, data)
				else:
					log.info("  dropped")
		except IOBridgeException as e:
			traceback.print_exc()

		#periodic pylon wakeup
		if time.time() > nextwakeuptime :
			nextwakeuptime = time.time() + 10
			pylon.can_tx(0x35C, 'e', b'\xC0\x00')
			log.info("wakeup: " + hex(0x35C) + " " + 'e' + " " + binascii.hexlify(b'\xc0\x00').decode("utf8"))

	s.close()
	sys.stdout.flush()
