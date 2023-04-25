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
import serial
import argparse
import sys
import binascii
import time
import math
import struct
from iobridge.iobridge import IOBridge

INS_REQUEST_MTU  = 0x1 # U2BE:HOST_MTU
INS_REPLY_MTU    = 0x2 # U2BE:MIN(HOST_MTU,SE_MTU)
INS_STATUS_IDLE  = 0x3 # none
INS_STATUS_BUSY  = 0x4 # U4BE:remaining_time_ms
INS_SUSPEND      = 0x5 # none
INS_SUSPEND_CTX  = 0x6 # BA:suspend_ctx
INS_RESUME       = 0x7 # BA:suspend_ctx
INS_GC_GET_ID    = 0x8 # none
INS_ID           = 0x9 # BA[8]:SE_ID
INS_SET_ADDRESS  = 0xA # BA[8]:SE_ID U1:7bit_I2C@
INS_BOOT_INFO    = 0xB # BA[8]:BOOT_INFO
INS_CARGO        = 0xC # BA:upper layer payload
INS_MASTER_INFO  = 0xD # BA:MCU_INFO[Ver_major,Ver_minor,Ver_patch,RFU]
INS_FLASHBACK    = 0xF # none

def i2c_negociate_mtu(iob, mtu_req=255, addr=0x78, i2c_int_port=0, i2c_int_pin=9):
  iob.i2c_write(addr, struct.pack(">BHH", INS_REQUEST_MTU, 2, mtu_req))
  time.sleep(0.01)
  data = iob.i2c_read(addr, 3)
  if data[0] != INS_REPLY_MTU or struct.unpack_from(">BH", data)[1] != 2:
    raise BaseException("MTU request not replied with a MTU reply")
  data = iob.i2c_read(addr, 2)
  return struct.unpack_from(">H", data)[0]

def fragment(packet, mtu=128):
  to_split = struct.pack(">BH", INS_CARGO, len(packet)) + packet
  f = []
  while len(to_split) > 0:
    f.append(to_split[:mtu])
    to_split = to_split[mtu:]
  return f

def i2c_write_packet(packet, iob, addr, mtu):
  chunks = fragment(packet, mtu)
  for c in chunks:
    iob.i2c_write_cache(addr, c)
  iob.i2c_write_cache_last()

def cargo_size(rarray):
  if len(rarray) < 3:
    return 0
  return struct.unpack_from(">BH", rarray)[1]

def reassemble(rarray):
  nb = cargo_size(rarray)
  if len(rarray) >= 3+nb:
    d = rarray[3:3+nb]
    #if rarray[0] == INS_CARGO:
    return d
  return None

def i2c_read_packet(iob, addr, mtu):
  rarray = iob.i2c_read(addr, 3)
  nb = cargo_size(rarray)
  while nb:
    si = min(nb, mtu)
    rarray += iob.i2c_read(addr, si)
    nb -= si
  return reassemble(rarray)

def i2c_exchange(iob, data, mtu=255, addr=0x78, error_as_exception=True, timeout=True, i2c_int_port=0, i2c_int_pin=9):
  # check if the I2C has already data to be retrieved
  if iob.i2c_is_interrupt(i2c_int_port, i2c_int_pin):
    d = i2c_read_packet(iob, addr, mtu)
    if not data or len(data) == 0:
      return d
  # if no data, don't send :)
  if data and len(data) > 0:
    i2c_write_packet(data, iob, addr, mtu)
    #support timeouts when the target is under debug
  """
    while True:
      try:
        iob.i2c_wait_interrupt(i2c_int_port, i2c_int_pin)
        break
      except:
        if timeout:
          raise
  if iob.i2c_is_interrupt(i2c_int_port, i2c_int_pin):
    data = i2c_read_packet(iob, addr, mtu)
    if data and data[0] == 0x80:
      # check error
      ret = struct.unpack_from(">BI", data)
      if ret[1] != 0 and error_as_exception:
        raise BaseException("Error %08X"  % ret[1])
    return data
  """
  data = iob.i2c_read_xfer(addr, i2c_int_port, i2c_int_pin)
  if data and data[0] == 0x80:
      # check error
      ret = struct.unpack_from(">BI", data)
      if ret[1] != 0 and error_as_exception:
        raise BaseException("Error %08X"  % ret[1])
  return data

if __name__ == '__main__':

  def auto_int(x):
    return int(x, 0)

  # TODO support using the native I2C bus of the RPi to work with local HSM

  parser = argparse.ArgumentParser(description="Fragment STDIN packets as I2C CARGO frames, reassemble replies and print packets")
  parser.add_argument("--port", default="/dev/ttyACM0", help="""Serial interface to use""")
  parser.add_argument("--baudrate", default="921600", help="IOBridge USART speed", type=auto_int)
  parser.add_argument("--addr", default="0x78", help="", type=auto_int)
  parser.add_argument("--mtu", default="255", help="", type=auto_int)
  parser.add_argument("--off", action="store_true")
  parser.add_argument("--bootdelay", default="100", help="Milliseconds after power on and before I2C transaction", type=auto_int)
  parser.add_argument("--loopbacktest", action="store_true")

  args = parser.parse_args()

  s = serial.Serial(port=args.port, baudrate=args.baudrate)
  iob = IOBridge(s)

  if args.off:
    iob.off()
  time.sleep(args.bootdelay/1000)
  iob.on()
  time.sleep(args.bootdelay/1000)
  if args.off:
    # wait atr
    iob.i2c_wait_interrupt()

  # negociate MTU with the device
  mtu = i2c_negociate_mtu(args.mtu)

  for line in sys.stdin:
    line = line.rstrip("\n").rstrip("\r")
    # previous reply not consumed? do it now
    if iob.i2c_is_interrupt():
      i2c_exchange(iob, b'', mtu, args.addr)
    data = binascii.unhexlify(line)
    if data and len(data):
      datar = i2c_exchange(iob, data, mtu, args.addr)
      if datar:
        print(binascii.hexlify(datar).decode("utf8"))
      if args.loopbacktest and datar != data:
        print("Loopback not ok")
        sys.exit(-1)

  s.close()
  sys.stdout.flush()
