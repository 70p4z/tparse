"""
*******************************************************************************
*  Usart Interface synchronizer 
*  (c) 2022 Olivier TOMAZ
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

import binascii
import random

class UsartIface:

	def send_challenge(self):
		try:
			rng = binascii.hexlify(random.randbytes(4))
		except:
			rng = b'\x01\x02\x03\x04'
		self.serial.write(b'\ninfo ' + rng + b'\n')
		return rng

	def __init__(self, serial):
		self.serial = serial
		# ungarble and flush incoming data
		lastrng = rng = self.send_challenge()
		while True:
			l = self.serial.readline()
			if l.startswith(b'INFO:' + rng) or l.startswith(b'INFO:' + lastrng):
				break;
