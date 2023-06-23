from smbus2 import SMBus
from smbus2 import i2c_msg
import binascii 
import struct
import traceback
import requests
import time
import json
import sys
import paho.mqtt.client as mqtt

#TODO grab parameter to reach home assistant from the command line parameters (and the sensor stem)
#TODO allow to override i2c address from command line

#40 pin connector
i2cbus = SMBus(1)
i2c_addr = 0x22 # validated on bus 7bit address

mqtt_client = None

def mqtt_start():
  global mqtt_client

  def mqtt_setup():
    global mqtt_client

    mqtt_client.publish('homeassistant/sensor/solax_grid_export/config', payload=json.dumps({"device_class": "power", "name": "Solax Grid Export", "state_topic": "homeassistant/sensor/solax_grid_export/state", "unit_of_measurement": "Wh"}), retain=True)
    mqtt_client.publish('homeassistant/sensor/solax_grid/config', payload=json.dumps({"device_class": "power", "name": "Solax Grid", "state_topic": "homeassistant/sensor/solax_grid/state", "unit_of_measurement": "Wh"}), retain=True)
    mqtt_client.publish('homeassistant/sensor/solax_pv1/config', payload=json.dumps({"device_class": "power", "name": "Solax PV1", "state_topic": "homeassistant/sensor/solax_pv1/state", "unit_of_measurement": "Wh"}), retain=True)
    mqtt_client.publish('homeassistant/sensor/solax_pv2/config', payload=json.dumps({"device_class": "power", "name": "Solax PV2", "state_topic": "homeassistant/sensor/solax_pv2/state", "unit_of_measurement": "Wh"}), retain=True)
    mqtt_client.publish('homeassistant/sensor/solax_battery/config', payload=json.dumps({"device_class": "power", "name": "Solax Battery", "state_topic": "homeassistant/sensor/solax_battery/state", "unit_of_measurement": "Wh"}), retain=True)
    mqtt_client.publish('homeassistant/sensor/solax_eps/config', payload=json.dumps({"device_class": "power", "name": "Solax EPS", "state_topic": "homeassistant/sensor/solax_eps/state", "unit_of_measurement": "Wh"}), retain=True)
    mqtt_client.publish('homeassistant/sensor/solax_output/config', payload=json.dumps({"device_class": "power", "name": "Solax Output", "state_topic": "homeassistant/sensor/solax_output/state", "unit_of_measurement": "Wh"}), retain=True)

    mqtt_client.publish('homeassistant/sensor/solax_pv1_voltage/config', payload=json.dumps({"device_class": "voltage", "name": "Solax PV1 Voltage", "state_topic": "homeassistant/sensor/solax_pv1_voltage/state", "unit_of_measurement": "V"}), retain=True)
    mqtt_client.publish('homeassistant/sensor/solax_pv2_voltage/config', payload=json.dumps({"device_class": "voltage", "name": "Solax PV2 Voltage", "state_topic": "homeassistant/sensor/solax_pv2_voltage/state", "unit_of_measurement": "V"}), retain=True)

    mqtt_client.publish('homeassistant/sensor/solax_battery_soc/config', payload=json.dumps({"device_class": "battery", "name": "Solax Battery SoC", "state_topic": "homeassistant/sensor/solax_battery_soc/state", "unit_of_measurement": "%"}), retain=True)

    mqtt_client.publish('homeassistant/sensor/solax_battery_max_charge_current/config', payload=json.dumps({"device_class": "current", "name": "Solax Battery Max Charge Current", "state_topic": "homeassistant/sensor/solax_battery_max_charge_current/state", "unit_of_measurement": "A"}), retain=True)

  def on_connect(client, userdata, flags, rc):
    #print(f"on_connect: rc={rc}")
    if rc==0:
      mqtt_setup()

  mqtt_client = mqtt.Client('solax_inverter')
  mqtt_client.on_connect=on_connect
  while True:
    try:
      mqtt_client.connect('192.168.0.4', 1883)
      mqtt_client.loop_start()
      break
    except:
      traceback.print_exc()
    time.sleep(1)

mqtt_start()

# read values from the solax
while True:
  try:
    write = i2c_msg.write(i2c_addr,b'\x00')
    read = i2c_msg.read(i2c_addr, 1)
    i2cbus.i2c_rdwr(write, read)
    # convert as byte array
    data = b''
    for v in read:
      data+=bytes([v])
    print(binascii.hexlify(data))
    length=data[0]
    write = i2c_msg.write(i2c_addr,b'\x00')
    read = i2c_msg.read(i2c_addr, length)
    i2cbus.i2c_rdwr(write, read)
    data = b''
    for v in read:
      data+=bytes([v])
    print(binascii.hexlify(data))
    #check data format
    if length < 2 or data[1] != 1:
      print("Unsupported encoding")
      time.sleep(0.2)
      continue

    fields = struct.unpack_from(">BBBBhhhhhhhhhhBBhhhHBBBBBBHH", data)
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
    IDX_MODE = 15
    IDX_PV1_VOLTAGE=16
    IDX_PV2_VOLTAGE=17
    IDX_EPS_CURRENT=18
    IDX_Y=19
    IDX_M=20
    IDX_D=21
    IDX_HR=22
    IDX_MN=23
    IDX_PV1_SWITCH_ON=24
    IDX_PV2_SWITCH_ON=25
    IDX_EPS_POWER=26
    IDX_EPS_VOLTAGE=27
    print (repr(fields))

    mqtt_client.publish('homeassistant/sensor/solax_grid_export/state', payload=str(fields[IDX_GRID_EXPORT]), retain=True)
    mqtt_client.publish('homeassistant/sensor/solax_grid/state', payload=str(fields[IDX_GRID]), retain=True)
    mqtt_client.publish('homeassistant/sensor/solax_pv1/state', payload=str(fields[IDX_PV1]), retain=True)
    mqtt_client.publish('homeassistant/sensor/solax_pv2/state', payload=str(fields[IDX_PV2]), retain=True)
    mqtt_client.publish('homeassistant/sensor/solax_battery/state', payload=str(fields[IDX_BMS_BAT]), retain=True)
    mqtt_client.publish('homeassistant/sensor/solax_eps/state', payload=str(fields[IDX_EPS_POWER]), retain=True)
    mqtt_client.publish('homeassistant/sensor/solax_output/state', payload=str(fields[IDX_EPS_POWER]+fields[IDX_GRID]-fields[IDX_GRID_EXPORT]), retain=True)
    mqtt_client.publish('homeassistant/sensor/solax_pv1_voltage/state', payload=str(fields[IDX_PV1_VOLTAGE]/10.0), retain=True)
    mqtt_client.publish('homeassistant/sensor/solax_pv2_voltage/state', payload=str(fields[IDX_PV2_VOLTAGE]/10.0), retain=True)
    mqtt_client.publish('homeassistant/sensor/solax_battery_soc/state', payload=str(fields[IDX_SOC]), retain=True)
    mqtt_client.publish('homeassistant/sensor/solax_battery_max_charge_current/state', payload=str(fields[IDX_CH]/10.0), retain=True)
  except:
    print(traceback.print_exc())
    sys.exit(-1)
  time.sleep(1)