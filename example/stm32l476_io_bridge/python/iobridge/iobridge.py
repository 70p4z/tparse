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
from iobridge.usartiface import UsartIface

import logging
import os

LOGLEVEL = os.environ.get('LOGLEVEL', 'DEBUG').upper()
LOGFORMAT = '%(asctime)s %(levelname)s %(threadName)s %(message)s'
logging.basicConfig(level=LOGLEVEL, format=LOGFORMAT)

default_logger = logging.getLogger("")


class IOBridgeException(BaseException):
	pass

class IOBridge(UsartIface):
	def __init__(self, serial, logger=None):
		super().__init__(serial)
		self.logger = logger
		if not logger:
			self.logger = default_logger

	def exchange(self, cmd):
		if self.logger:
			self.logger.debug(">"+cmd)
		self.serial.write((cmd + "\n").encode("utf8"))
		while True:
			rline = self.serial.readline()
			rline = rline.rstrip(b'\n').decode("utf8")
			if rline == None or len(rline) == 0 or not rline.startswith('OK:'):
				if not rline:
					rline = b''
				if rline.startswith('#'):
					if self.logger:
						self.logger.debug(rline)
					continue
				if self.logger:
					self.logger.error("Error: " + cmd)
					self.logger.error("       " + rline)
				raise IOBridgeException()
			break
		if self.logger:
			self.logger.debug("<"+rline)
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
				if self.logger:
					self.logger.error("Error: CAPDU:" + line)
					self.logger.error("       RAPDU:" + rline.decode('utf8'))
				raise IOBridgeException()
		return rbin
	def off(self):
		self.exchange("off")
	def on(self):
		self.exchange("on")
	def i2c_write(self, addr, data):
		return self.exchange("i2cw " + hex(addr) + " " + binascii.hexlify(data).decode("utf8"))
	def i2c_read(self, addr, maxlen):
		return binascii.unhexlify(self.exchange("i2cr " + hex(addr) + " " + hex(maxlen)))
	def i2c_wait_interrupt(self):
		self.exchange("i2ciwait")
	def gpio_set(self, port, pin, state):
		self.exchange("gpo " + hex(port) + " " + hex(pin) + " " + hex(state))
	def gpio_get(self, port, pin):
		return self.exchange("gpi " + hex(port) + " " + hex(pin)) == "01"
	def can_available(self):
		rep = self.exchange("cavail")
		return int(rep, 16)
	#Return CAN identifier, CAN identifier kind, CAN frame data
	def can_rx(self):
		packet = self.exchange("crx")
		packetfield = packet.split(",")
		return int(packetfield[0], 16), packetfield[1], binascii.unhexlify(packetfield[2])
	def can_tx(self, cid, cidkind, data):
		if cidkind == 'extended' or cidkind == 'e' or cidkind == 'ex' or cidkind == 'x' or cidkind == "29":
			cidkind = 'e'
		elif cidkind == 'standard' or cidkind == 's' or cidkind == 'std' or cidkind == '11': 
			cidkind = 's'
		else:
			raise IOBridgeException()
		self.exchange("ctx " + hex(cid) + " " + cidkind + " " + binascii.hexlify(data).decode("utf8"))

