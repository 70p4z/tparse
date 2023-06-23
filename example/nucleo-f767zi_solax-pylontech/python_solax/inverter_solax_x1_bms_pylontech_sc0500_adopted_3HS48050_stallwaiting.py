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
	parser.add_argument("--port", default="/dev/ttyACM0", help="""IO bridge interface to the INVerter""")
	parser.add_argument("--baudrate", default="921600", help="IOBridge USART speed", type=auto_int)
	parser.add_argument("--port2", default="/dev/ttyACM1", help="""IO bridge interface to the BMS""")
	parser.add_argument("--baudrate2", default="921600", help="IOBridge USART speed", type=auto_int)
	parser.add_argument("--vmax", default="400", help="Max voltage", type=auto_int)
	parser.add_argument("--vmin", default="80", help="Min voltage", type=auto_int)
	parser.add_argument("--chmax", default="25", help="Charge max current", type=auto_int)
	parser.add_argument("--dischmax", default="25", help="Discharge max current", type=auto_int)
	#parser.add_argument("--bmskind", default="54", help="Hex BMS kind for packet 0x1877")
	args = parser.parse_args()

	s = serial.Serial(port=args.port, baudrate=args.baudrate)
	inv = IOBridge(s)

	s2 = serial.Serial(port=args.port2, baudrate=args.baudrate2)
	bms = IOBridge(s2)

	bms_adoption_state = 0
	bms_last_activity = 
	bms_fault = False
	vmax = vmin = chmax = dischmax = vcur = icur = capacity = soh = 0

	#flush background messages
	while inv.can_available() > 0:
		inv.can_rx()
	while bms.can_available() > 0:
		bms.can_rx()

	def to_inv(cid, cidkind, data, print_suffix=""):
		log.info(">inv " + hex(cid) + " " + cidkind + " " + binascii.hexlify(data).decode("utf8") + print_suffix)
		inv.can_tx(cid, cidkind, data)

	def to_bms(cid, cidkind, data, print_suffix=""):
		log.info(">bms " + hex(cid) + " " + cidkind + " " + binascii.hexlify(data).decode("utf8") + print_suffix)
		bms.can_tx(cid, cidkind, data)

	#start transmission
	while True:
		try:
			if inv.can_available() > 0:
				cid, cidkind, data = inv.can_rx()
				forward=True
				log.info("inv> " + hex(cid) + " " + cidkind + " " + binascii.hexlify(data).decode("utf8"))
				# INV_request broadcast?
				if cid == 0x1871 and data[0] == 0x01:
					# bms_hannounce
					#reply with received data
					#log.info(">inv " + hex(0x1801) + " " + 'e' + " " + binascii.hexlify(data).decode("utf8"))
					#inv.can_tx(0x1801, 'e', data)
				if cid == 0x1871 and data[0] == 0x03:
					log.info("+ adoption request")
					#datarep = b'\x02\x00\x01\x00\x01\x00\x00\x00'
					#log.info(">inv " + hex(0x1801) + " " + 'e' + " " + binascii.hexlify(datarep).decode("utf8"))
					#inv.can_tx(0x1801, 'e', datarep)
					bms_adopted = False #True
				# detect disconnection
				if cid == 0x1871 and data[0] == 0x02:
					if bms_adoption_state = 1:
						to_inv(0x1801, 'e', b'\x02\x00\x01\x00\x01\x00\x00\x00')
						bms_adoption_state = 2
				if cid == 0x1871 and data[0] != 0x01:
					forward=False
				if forward:
					to_bms(cid, cidkind, data)
		except IOBridgeException as e:
			traceback.print_exc()

		try:
			if bms.can_available() > 0:
				cid, cidkind, data = bms.can_rx()
				msg = ""
				forward=True
				log.info("bms> " + hex(cid) + " " + cidkind + " " + binascii.hexlify(data).decode("utf8"))
				if cid == 0x1871:
					if data[0] == 0x05:
						data = b'\x02' + data[1:]
				if cid == 0x1872:
					# # bms name and version
					# name = b'\x00' + b'PTBR' + b'\x00\x00\x00'
					# version = b'\x00' + b'1.0' + b'\x00\x00\x00\x00'
					# log.info(">inv " + hex(0x1881) + " " + 'e' + " " + binascii.hexlify(name).decode("utf8"))
					# inv.can_tx(0x1881, 'e', name)
					# log.info(">inv " + hex(0x1882) + " " + 'e' + " " + binascii.hexlify(version).decode("utf8"))
					# inv.can_tx(0x1882, 'e', version)

					vmax, vmin, chmax, dischmax = struct.unpack_from("<hhhh", data)
					if chmax == 0:
						# needs to execute a manual reset sequence
						bms_fault = True
					# fake the max charge and discharge currents
					# OTO: this is taken into account, the inverter respects it
					try:
						_vmax = vmax/10.0
						if args.vmax != 0:
							_vmax = args.vmax
						_vmin = vmin/10.0
						if args.vmin != 0:
							_vmin = args.vmin
						_chmax = chmax/10.0
						if args.chmax != 0:
							_chmax = args.chmax
						_dischmax = dischmax/10.0
						if args.dischmax != 0:
							_dischmax = args.dischmax
						data = struct.pack("<HHHH", int(_vmax*10), int(_vmin*10), int(_chmax*10), int(_dischmax*10))
					except:
						traceback.print_exc()
						forward=False
				if cid == 0x1873:
					vcur, icur, capacity, soh = struct.unpack_from("<HhHH", data)
					msg = " V=" + str(vcur/10.0) + " I=" + str(icur/10.0)
					# early capacity cap to avoid troubles of the BMS bms
					if capacity < 100: # 1 volt of margin to maxout the capacity
						data = struct.pack("<HhHH", vcur, icur, capacity, soh)
				## wipe BMS_PackData Current
				#if cid == 0x1873:
				#	data = data [0:2] + b'\x00\x00' + data[4:8]
				#	#data = b'\x00\x00\x00\x00' + data[4:8]
				if cid == 0x1875:
					if data[4] != 1: # contactor not engaged
						bms_fault = 1
					#data = data[0:2] + b'\x04\x00\x00\x00\x01\x00' 
				if cid == 0x1877:
					if data[0] == 0x04:
						# needs to execute a manual reset sequence
						bms_fault = True 
					#data = data[0:4] + b'\x52' + data[5:6]
					# data = b'\x00\x00\x00\x00\x52\x00'
					# if bms_counter == 0:
					# 	data += b'\x00\x02'
					# else:
					# 	data += b'\x22' + struct.pack(">B", bms_counter * 16)
					# bms_counter=(bms_counter+1)%5
					# fixed value
					#data = b'\x00\x00\x00\x00\x54\x00\x22\x40'
					data = b'\x00\x00\x00\x00\x54\x00\x22\x40'
				# only forward known packet for known interactions :)
				if not cid in (0x1871, 0x1872, 0x1873, 0x1874, 0x1875, 0x1876, 0x1877):
					forward=False
				if forward:
					to_inv(cid, cidkind, data)
					if (cid == 0x1877):
						# append additional 1878 message
						data = struct.pack("<HHI", vmax, 0, soh)
						to_inv(0x1878, 'e', data)
						# mark the whole list of BMS data as being sent
						if bms_adoption_state = 0:
							to_inv(0x0100A000, 'e', b'')
							bms_adoption_state = 1
				else:
					log.info("  dropped")
		except IOBridgeException as e:
			traceback.print_exc()

		# #periodic BMS wakeup
		# if time.time() > nextwakeuptime :
		# 	nextwakeuptime = time.time() + 10
		# 	bms.can_tx(0x35C, 'e', b'\xC0\x00')
		# 	log.info("wakeup: " + hex(0x35C) + " " + 'e' + " " + binascii.hexlify(b'\xc0\x00').decode("utf8"))

	s.close()
	sys.stdout.flush()



"""
0x4200 =>
2022-05-22 11:41:55,705 INFO MainThread bms> 0x7310 e 010010020502341c
2022-05-22 11:41:55,705 INFO MainThread   dropped
2022-05-22 11:41:55,708 INFO MainThread bms> 0x7320 e 2d00030f90003200
2022-05-22 11:41:55,709 INFO MainThread   dropped
2022-05-22 11:41:55,712 INFO MainThread bms> 0x7330 e 50594c4f4e544543
2022-05-22 11:41:55,712 INFO MainThread   dropped
2022-05-22 11:41:55,715 INFO MainThread bms> 0x7340 e 4800000000000000
2022-05-22 11:41:55,715 INFO MainThread   dropped



"""
