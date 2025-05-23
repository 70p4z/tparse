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

import socket
import traceback
import select
import time
import threading

import logging
import os

LOGLEVEL = os.environ.get('LOGLEVEL', 'INFO').upper()
LOGFORMAT = '%(asctime)s %(levelname)s %(threadName)s %(message)s'
logging.basicConfig(level=LOGLEVEL, format=LOGFORMAT)
default_logger = logging.getLogger("")
default_logger.setLevel(LOGLEVEL)

IOBRIDGE_HOST = os.environ.get('IOBRIDGE_HOST', 'localhost')
IOBRIDGE_PORT = int(os.environ.get('IOBRIDGE_PORT', '6955'), 0)

class IOBridgeException(BaseException):
  def __init__(self, reason=""):
    super().__init__(reason)

class IOBridge(UsartIface):
  def __init__(self, serial, logger=None):
    super().__init__(serial)
    self.logger = logger
    if not logger:
      self.logger = default_logger
    self.logger.debug("IOBridge init")
    self.client_socket = None

  def _exchange_serial_for_socket(self, cmd):
    if self.logger:
      self.logger.debug(">"+cmd)
    self.serial.write((cmd + "\n").encode("utf8"))
    while True:
      rline = self.serial.readline()
      rline = rline.rstrip(b'\n').decode("utf8")
      if rline == None or len(rline) == 0 or not rline.startswith('OK:'):
        if not rline:
          rline = ""
        if self.logger:
          self.logger.debug(rline)
        if rline.startswith('#'):
          continue
        if self.logger:
          self.logger.error("Serial error: " + cmd + ": " + rline)
      break
    if self.logger:
      self.logger.debug("<"+rline)
    return rline

  # pipe the requested socket request into the serial
  # error are forwarded, 

  def _run_server(self):
    self.clients = [self.server_socket]
    self.client_lines = {}
    while True:
      try:
        readable, writable, exceptional = select.select(self.clients, [], self.clients, 0)
        for s in readable:
          if s == self.server_socket:
            # accept a new connection
            try:
              (client_socket, address) = self.server_socket.accept()
            except:
              traceback.print_exc()
              sys.exit(-1)
            if self.logger:
              self.logger.debug("Connection from %s:%s" % address)
            self.client_lines[client_socket] = b''
            self.clients.append(client_socket)
          else:
            # read from client socket
            try:
              chunk = s.recv(4096)
              self.client_lines[client_socket] += chunk
              eol = self.client_lines[client_socket].find(b'\n')
              while eol != -1:
                # pop complete line
                l = self.client_lines[client_socket][:eol+1]
                self.client_lines[client_socket] = self.client_lines[client_socket][eol+1:]
                try:
                  l = l.decode().rstrip('\n')
                  reply = self._exchange_serial_for_socket(l)
                  s.sendall(reply.encode('utf8')+b'\n')
                except Exception as e:
                  traceback.print_exc()
                  s.sendall(b'error: '+str(e).encode('utf8')+b'\n')
                eol = self.client_lines[client_socket].find(b'\n')
            except:
              self.logger.debug("invalid data received")
              self.clients.remove(s)
              self.client_lines.remove(s)
              s.close()
        for s in exceptional:
          self.logger.debug("socket in error")
          self.clients.remove(s)
          self.client_lines.remove(s)
          s.close()
      except:
        traceback.print_exc()

  def _start_server_socket_or_connect(self):
    self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
      self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)
      self.server_socket.bind((IOBRIDGE_HOST, IOBRIDGE_PORT))
      self.server_socket.listen(5)
      self.server_thread = threading.Thread(name="iobridge_server", target=self._run_server)
      self.server_thread.start()
    except socket.error as msg:
      if self.logger:
        self.logger.debug("IOBridge server socket already binded")

    # now connect to the iobridge server socket to further communicate with it
    if self.client_socket is None:
      retry=5
      while retry > 0:
        try:
          s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
          s.connect((IOBRIDGE_HOST, IOBRIDGE_PORT))
          self.client_socket = s
          return
        except:
          pass
        time.sleep(0.1)
        retry-=1
      raise IOBridgeException("Can't connect to IOBridge server socket")


  # try bind the serversocket, if can't, then use the existing server
  # send the request to the server and wait for the response
  def exchange(self, cmd):
    self._start_server_socket_or_connect()
    if self.logger:
      self.logger.debug(">>"+cmd)

    # send command to the server socket
    self.client_socket.sendall((cmd + "\n").encode("utf8"))

    # wait for response
    rline = b''
    while True:
      # fetch a complete line
      eol = -1
      while eol == -1:
        rline += self.client_socket.recv(4096)
        eol = rline.find(b'\n')
      rline = rline.rstrip(b'\n').decode("utf8")
      if rline != None and len(rline) > 0:
        if rline.startswith('OK:'):
          break
        if self.logger:
          self.logger.debug(rline)
        if rline.startswith('#'):
          continue
        if self.logger:
          self.logger.error("Error: " + cmd)
          self.logger.error("       " + rline)
        raise IOBridgeException(rline)
    if self.logger:
      self.logger.debug("<<"+rline)
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
