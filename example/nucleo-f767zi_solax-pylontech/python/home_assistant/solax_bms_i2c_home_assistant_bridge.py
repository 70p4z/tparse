from smbus2 import SMBus
from smbus2 import i2c_msg
import binascii 
import struct
import traceback
import requests
import time
import json
import sys

#TODO grab parameter to reach home assistant from the command line parameters (and the sensor stem)
#TODO allow to override i2c address from command line

#40 pin connector
i2cbus = SMBus(1)
i2c_addr = 0x22 # validated on bus 7bit address
hacs_token = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJjN2VlZGQ3MDg0Njk0YTA4OTE1MzU2NDI2MmM3YjQ3OCIsImlhdCI6MTY3MzU1NDg0MCwiZXhwIjoxOTg4OTE0ODQwfQ.HGhqPP9NKxneoEEuhtwGtFBRXEjrF0mKNqHPQ5ekczE"
hacs_url = "http://192.168.0.4:8123"

def home_assistant_push(ident, name, value, unit=None):
  if not unit:
    requests.post(
        hacs_url + "/api/states/" + ident,
        headers={
            "Authorization": "Bearer " + hacs_token,
            "content-type": "application/json",
        },
        data=json.dumps({"state": ""+str(value), "attributes": {"state_class": "total", "friendly_name": name, "unique_id": ident, "entity_id": ident}}),
    )
  else:
    requests.post(
      hacs_url + "/api/states/" + ident,
      headers={
          "Authorization": "Bearer " + hacs_token,
          "content-type": "application/json",
      },
      data=json.dumps({"state": ""+str(value), "attributes": {"state_class": "total", "unit_of_measurement": unit, "friendly_name": name, "unique_id": ident, "entity_id": ident}}),
    )

# read values from the solax
while True:
  try:
    write = i2c_msg.write(i2c_addr,b'\x00')
    read = i2c_msg.read(i2c_addr, 25)
    i2cbus.i2c_rdwr(write, read)
    # convert as byte array
    data = b''
    for v in read:
      data+=bytes([v])

    #check data format
    if data[0] != 25 or data[1] != 1:
      print("Unsupported encoding")
      time.sleep(0.2)
      continue

    fields = struct.unpack_from(">BBBBhhhhhhhhhhB", data)
    IDX_STATE = 2
    IDX_FORCED_MODE = 3
    IDX_GRID_EXPORT = 4
    IDX_GRID = 5
    IDX_PV1 = 6
    IDX_PV2 = 7
    IDX_INV_BAT = 8
    IDX_BMS_BAT = 9
    IDX_SOC = 10
    IDX_CH = 11
    IDX_DISCH = 12
    IDX_FORCED_CH = 13
    IDX_OPT_RULE = 14
    print (repr(fields))

    home_assistant_push("sensor.solax_grid_export", "Grid export",        fields[IDX_GRID_EXPORT], "W")
    home_assistant_push("sensor.solax_grid",        "Grid",               fields[IDX_GRID], "W")
    home_assistant_push("sensor.solax_pv1",         "PV array 1",         fields[IDX_PV1], "W")
    home_assistant_push("sensor.solax_pv2",         "PV array 2",         fields[IDX_PV2], "W")
    home_assistant_push("sensor.solax_inv_bat",     "Battery (inverter)", fields[IDX_INV_BAT], "W")
    home_assistant_push("sensor.solax_bms_bat",     "Battery (BMS)",      fields[IDX_BMS_BAT], "W")
    home_assistant_push("sensor.solax_soc",         "Battery Charge",     fields[IDX_SOC], "%")
    home_assistant_push("sensor.solax_opt_rule",    "Solax Optimization", fields[IDX_OPT_RULE])
    
    state = fields[IDX_STATE]
    str_states = {}
    if fields[IDX_STATE] in str_states:
      mode = str_states[fields[IDX_STATE]]
    home_assistant_push("sensor.solax_state", "Running state", state)

    mode = fields[IDX_FORCED_MODE]
    str_modes = {}
    if fields[IDX_FORCED_MODE] in str_modes:
      mode = str_modes[fields[IDX_FORCED_MODE]]
    home_assistant_push("sensor.solax_mode", "Work mode", mode)
  except:
    print(traceback.print_exc())
    sys.exit(-1)
  time.sleep(1)