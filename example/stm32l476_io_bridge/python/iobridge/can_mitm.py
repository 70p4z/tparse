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

	def auto_int(x):
		return int(x, 0)

	parser = argparse.ArgumentParser(description="CAN bus man-in-the-middle example")
	parser.add_argument("--port", default="/dev/ttyACM0", help="""Serial interface to use""")
	parser.add_argument("--baudrate", default="921600", help="IOBridge USART speed", type=auto_int)
	parser.add_argument("--port2", default="/dev/ttyACM0", help="""Serial interface to use""")
	parser.add_argument("--baudrate2", default="921600", help="IOBridge USART speed", type=auto_int)
	args = parser.parse_args()

	s = serial.Serial(port=args.port, baudrate=args.baudrate)
	iob = IOBridge(s)

	s2 = serial.Serial(port=args.port2, baudrate=args.baudrate2)
	iob2 = IOBridge(s2)

	while True:
		#port => port2
		try:
			if iob.can_available() > 0:
				cid, cidkind, data = iob.can_rx()
				iob2.can_tx(cid, cidkind, data)
		except IOBridgeException as e:
			traceback.print_exc()

		#port2 => port
		try:
			if iob2.can_available() > 0:
				cid, cidkind, data = iob2.can_rx()
				iob.can_tx(cid, cidkind, data)
		except IOBridgeException as e:
			traceback.print_exc()

	s.close()
	sys.stdout.flush()
