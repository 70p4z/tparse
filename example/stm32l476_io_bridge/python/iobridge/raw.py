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
	def auto_int(x):
		return int(x, 0)


	parser = argparse.ArgumentParser(description="Wrap STDIN as ISO7816 T=0 commands, handling REISSUE/GET RESPONSE.")
	parser.add_argument("--port", default="/dev/ttyACM0", help="""Serial interface to use""")
	parser.add_argument("--baudrate", default="921600", help="IOBridge USART speed", type=auto_int)
	args = parser.parse_args()

	s = serial.Serial(port=args.port, baudrate=args.baudrate)
	# ungarble and flush incoming data
	rng = binascii.hexlify(random.randbytes(4))
	s.write(b'\ninfo ' + rng + b'\n')
	while True:
		l = s.readline()
		if l.startswith(b'INFO:' + rng):
			break;
	for line in sys.stdin:
		line = line.rstrip("\n").rstrip("\r")
		print(">" + line)
		start = time.time();
		s.write((line + "\n").encode("utf8"))
		while True:
			rline = s.readline()
			stop = time.time();
			if rline == None or len(rline) == 0 or not rline.startswith(b'OK:'):
				if not rline:
					rline = b''
				if rline.startswith(b'#'):
					print(rline)
					continue
				print("Error: Request:" + line)
				print("       Reply:  " + rline.decode("utf8"))
				sys.exit(-1)
			break
		rline = rline.rstrip(b'\n')
		print("<" + rline.decode("utf8") + " (time:"+str(math.ceil((stop-start)*1000)/1000.0)+"s)")
	s.close()
	sys.stdout.flush()
