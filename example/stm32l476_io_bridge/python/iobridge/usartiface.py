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
import threading

class UsartIface:

	def get_challenge(self):
		try:
			rng = binascii.hexlify(random.randbytes(4))
		except:
			rng = b'\x01\x02\x03\x04'
		return rng

	# after 5 seconds, the challenge must have been accepted
	def __init__(self, serial, max_timeout=5):
		self.serial = serial
		# ungarble and flush incoming data
		rng = self.get_challenge()
		self.challenge_received = False
		def recv_challenge():
			while True:
				l = self.serial.readline()
				if l.startswith(b'INFO:' + rng):
					self.challenge_received = True
					break
		recv_thread = threading.Thread(target=recv_challenge)
		recv_thread.start()
		timeout = time.time() + max_timeout
		retry_time = time.time() + max_timeout/10

		# resend the challenge until received
		self.serial.write(b'\ninfo ' + rng + b'\n')
		while time.time() < timeout and not self.challenge_received:
			if time.time() >= retry_time:
				retry_time = time.time() + max_timeout/10
				self.serial.write(b'\ninfo ' + rng + b'\n')
			time.sleep(0.05)

		recv_thread.join()