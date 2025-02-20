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

LOGLEVEL = os.environ.get('LOGLEVEL', 'INFO').upper()
LOGFORMAT = '%(asctime)s %(levelname)s %(threadName)s %(message)s'
logging.basicConfig(level=LOGLEVEL, format=LOGFORMAT)
default_logger = logging.getLogger("")
default_logger.setLevel(LOGLEVEL)

class IOBridgeException(BaseException):
	pass

class IOBridge(UsartIface):
	def __init__(self, serial, logger=None):
		super().__init__(serial)
		self.logger = logger
		if not logger:
			self.logger = default_logger
		self.logger.debug("IOBridge init")

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
				if self.logger:
					self.logger.debug(rline)
				if rline.startswith('#'):
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
	def atr(self, ta1=0):
		return binascii.unhexlify(self.exchange("atr " + hex(ta1)))
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
					self.logger.error(f"Error: CAPDU: {binascii.hexlify(capdu).decode()}")
					self.logger.error(f"       RAPDU: {binascii.hexlify(rbin).decode()}")
				raise IOBridgeException()
		return rbin
	def isocfg(self, smartcard_clock_hz):
		self.exchange("isocfg " + hex(smartcard_clock_hz))
	def off(self):
		self.exchange("off")
	def on(self):
		self.exchange("on")
	def i2c_strobe(self, addr):
		return self.exchange("i2cs " + hex(addr))
	def i2c_write(self, addr, data, retry=5):
		return self.exchange("i2cw " + hex(addr) + " " + binascii.hexlify(data).decode("utf8") + " " + hex(retry))
	def i2c_read(self, addr, maxlen, retry=5):
		return binascii.unhexlify(self.exchange("i2cr " + hex(addr) + " " + hex(maxlen) + " " + hex(retry)))
	def i2c_write_cache(self, addr, data, retry=5):
		return self.exchange("i2cwc " + hex(addr) + " " + binascii.hexlify(data).decode("utf8") + " " + hex(retry))
	def i2c_write_cache_last(self):
		return self.exchange("i2cwclast")
	def i2c_read_xfer(self, addr, port=0, pin=9, timeout_ms=30000, format=0, retry=5):
		return binascii.unhexlify(self.exchange("i2crxfer " + hex(addr) + " " + hex(port) + " " + hex(pin) + " " + hex(timeout_ms) + " " + hex(format) + " " + hex(retry)))
	def i2c_wait_interrupt(self, port=0, pin=9):
		self.exchange("i2ciwait " + hex(port) + " " + hex(pin))
	def i2c_is_interrupt(self, port=0, pin=9):
		return self.exchange("gpi " + hex(port) + " " + hex(pin)) != "01"
	def gpio_set(self, port, pin, state):
		self.exchange("gpo " + hex(port) + " " + hex(pin) + " " + hex(state))
	def gpio_get(self, port, pin):
		return self.exchange("gpi " + hex(port) + " " + hex(pin)) == "01"
	def gpio_cfg_input(self, port, pin):
		self.exchange("cfgi " + hex(port) + " " + hex(pin))
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
	def spi_xfer(self, data):
		return binascii.unhexlify(self.exchange("spix " + binascii.hexlify(data).decode("utf8")))
