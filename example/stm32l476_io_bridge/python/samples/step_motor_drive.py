import serial
from iobridge.iobridge import IOBridge
import logging
import os
import traceback
import time

LOGLEVEL = os.environ.get('LOGLEVEL', 'INFO').upper()
LOGFORMAT = '%(asctime)s %(levelname)s %(threadName)s %(message)s'
logging.basicConfig(level=LOGLEVEL, format=LOGFORMAT)
log = logging.getLogger("runner")

# configure the right tty to open the nucleo to target
iob = IOBridge(serial.Serial(port="/dev/ttyACM0", baudrate=921600))

#######################################################################################################################
#                                                                                                                     #
#     ▄▄▄▄▄▄     ▄▄▄▄▄▄   ▄▄▄   ▄▄            ▄▄▄  ▄▄▄     ▄▄     ▄▄▄▄▄▄    ▄▄▄▄▄▄     ▄▄▄▄▄▄   ▄▄▄   ▄▄     ▄▄▄▄     #
#     ██▀▀▀▀█▄   ▀▀██▀▀   ███   ██            ███  ███    ████    ██▀▀▀▀█▄  ██▀▀▀▀█▄   ▀▀██▀▀   ███   ██   ██▀▀▀▀█    #
#     ██    ██     ██     ██▀█  ██            ████████    ████    ██    ██  ██    ██     ██     ██▀█  ██  ██          #
#     ██████▀      ██     ██ ██ ██            ██ ██ ██   ██  ██   ██████▀   ██████▀      ██     ██ ██ ██  ██  ▄▄▄▄    #
#     ██           ██     ██  █▄██            ██ ▀▀ ██   ██████   ██        ██           ██     ██  █▄██  ██  ▀▀██    #
#     ██         ▄▄██▄▄   ██   ███            ██    ██  ▄██  ██▄  ██        ██         ▄▄██▄▄   ██   ███   ██▄▄▄██    #
#     ▀▀         ▀▀▀▀▀▀   ▀▀   ▀▀▀            ▀▀    ▀▀  ▀▀    ▀▀  ▀▀        ▀▀         ▀▀▀▀▀▀   ▀▀   ▀▀▀     ▀▀▀▀     #
#                                                                                                                     #
#                                                                                                                     #
#######################################################################################################################
#on the morpho top right connector
# PC8
port_EN = 2
pin_EN = 8

# PC5
port_DIR = 2
pin_DIR = 5

# PC6
port_STEP = 2
pin_STEP = 6

###############################################################################
#                                                                             #
#     ▄▄    ▄▄  ▄▄▄▄▄▄▄▄  ▄▄        ▄▄▄▄▄▄    ▄▄▄▄▄▄▄▄  ▄▄▄▄▄▄      ▄▄▄▄      #
#     ██    ██  ██▀▀▀▀▀▀  ██        ██▀▀▀▀█▄  ██▀▀▀▀▀▀  ██▀▀▀▀██  ▄█▀▀▀▀█     #
#     ██    ██  ██        ██        ██    ██  ██        ██    ██  ██▄         #
#     ████████  ███████   ██        ██████▀   ███████   ███████    ▀████▄     #
#     ██    ██  ██        ██        ██        ██        ██  ▀██▄       ▀██    #
#     ██    ██  ██▄▄▄▄▄▄  ██▄▄▄▄▄▄  ██        ██▄▄▄▄▄▄  ██    ██  █▄▄▄▄▄█▀    #
#     ▀▀    ▀▀  ▀▀▀▀▀▀▀▀  ▀▀▀▀▀▀▀▀  ▀▀        ▀▀▀▀▀▀▀▀  ▀▀    ▀▀▀  ▀▀▀▀▀      #
#                                                                             #
#                                                                             #
###############################################################################
class StepMotor():
  def __init__(self, iob, port_EN, pin_EN, port_DIR, pin_DIR, port_STEP, pin_STEP):
    self.iob = iob
    self.port_EN = port_EN
    self.pin_EN = pin_EN
    self.port_DIR = port_DIR
    self.pin_DIR = pin_DIR
    self.port_STEP = port_STEP
    self.pin_STEP = pin_STEP
  
  def enable(self, enable):
    self.iob.gpio_set(self.port_EN, self.pin_EN, enable);

  def dir(self, direction):
    self.iob.gpio_set(self.port_DIR, self.pin_DIR, direction); 

  def steps(self, stepcount=0, delay_us=0):
    while stepcount>0:
      self.iob.gpio_set(self.port_STEP, self.pin_STEP, 1);
      time.sleep(delay_us/1000000.0/2)
      self.iob.gpio_set(self.port_STEP, self.pin_STEP, 0);
      time.sleep(delay_us/1000000.0/2)
      stepcount-=1


#########################################################################################
#                                                                                       #
#       ▄▄▄▄       ▄▄▄▄   ▄▄▄▄▄▄▄▄  ▄▄▄   ▄▄     ▄▄     ▄▄▄▄▄▄     ▄▄▄▄▄▄     ▄▄▄▄      #
#     ▄█▀▀▀▀█    ██▀▀▀▀█  ██▀▀▀▀▀▀  ███   ██    ████    ██▀▀▀▀██   ▀▀██▀▀    ██▀▀██     #
#     ██▄       ██▀       ██        ██▀█  ██    ████    ██    ██     ██     ██    ██    #
#      ▀████▄   ██        ███████   ██ ██ ██   ██  ██   ███████      ██     ██    ██    #
#          ▀██  ██▄       ██        ██  █▄██   ██████   ██  ▀██▄     ██     ██    ██    #
#     █▄▄▄▄▄█▀   ██▄▄▄▄█  ██▄▄▄▄▄▄  ██   ███  ▄██  ██▄  ██    ██   ▄▄██▄▄    ██▄▄██     #
#      ▀▀▀▀▀       ▀▀▀▀   ▀▀▀▀▀▀▀▀  ▀▀   ▀▀▀  ▀▀    ▀▀  ▀▀    ▀▀▀  ▀▀▀▀▀▀     ▀▀▀▀      #
#                                                                                       #
#                                                                                       #
#########################################################################################
sm = StepMotor(iob, port_EN, pin_EN, port_DIR, pin_DIR, port_STEP, pin_STEP)
sm.enable(1)
sm.dir(0)
sm.steps(50, 1000)
sm.dir(1)
sm.steps(50, 1000)
# after 2 seconds, release
time.sleep(2)
sm.enable(0)
