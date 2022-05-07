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

import sys
import binascii
import random

class IOBridge:
	def __init__(self, serial):
		self.serial = serial
		# ungarble and flush incoming data
		rng = binascii.hexlify(random.randbytes(4))
		self.serial.write(b'\ninfo ' + rng + b'\n')
		while True:
			l = self.serial.readline()
			if l.startswith(b'INFO:' + rng):
				break;

	def exchange(self, cmd):
		print(">"+cmd)
		self.serial.write((cmd + "\n").encode("utf8"))
		while True:
			rline = self.serial.readline()
			rline = rline.rstrip(b'\n').decode("utf8")
			if rline == None or len(rline) == 0 or not rline.startswith('OK:'):
				if not rline:
					rline = b''
				if rline.startswith('#'):
					print(rline)
					continue
				print("Error: " + cmd)
				print("       " + rline)
				sys.exit(-1)
			break
		print("<"+rline)
		reply = rline.split(':')[1]
		return reply
	def atr(self):
		return binascii.unhexlify(self.exchange("atr"))
	def apdu_t0(self, capdu):
		rbin = binascii.unhexlify(self.exchange("t0 " + binascii.hexlify(capdu).decode("utf8")))
		# decode status words
		if len(rbin) >= 2:
			# some smartcard do not respect get response exact parameters, and send 6100 while not expecting a 256 byte get response (C0)
			if rbin[-2] == 0x61:
				pass
			elif rbin[-2] == 0x6C:
				pass
			elif rbin[-2]&0xF0 != 0x90:
				print("Error: CAPDU:" + line)
				print("       RAPDU:" + rline.decode('utf8'))
				sys.exit(-1)
		return rbin
	def off(self):
		self.exchange("off")
	def on(self):
		self.exchange("on")
	def i2c_write(self, addr, data):
		self.exchange("i2cw " + hex(addr) + " " + binascii.hexlify(data).decode("utf8"))
	def i2c_read(self, addr, maxlen):
		return binascii.unhexlify(self.exchange("i2cr " + hex(addr) + " " + hex(maxlen)))
	def i2c_wait_interrupt(self):
		self.exchange("i2ciwait")
