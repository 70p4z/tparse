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
import threading

#TODO grab parameter to reach home assistant from the command line parameters (and the sensor stem)
#TODO allow to override i2c address from command line

#40 pin connector
i2cbus = SMBus(1)
i2c_addr = 0x22 # validated on bus 7bit address
i2clock = threading.RLock()

mqtt_client = None

def mqtt_boolstr(b):
  if b:
    return "ON"
  return "OFF"

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

    # PV1 GMPPT state
    def on_message_solax_pv1_gmppt(client, userdata, msg):
      try:
        if msg.payload.decode('utf-8') == "ON":
          msg=b'\x02'
        else:
          msg=b'\x01'
        with i2clock:
          write = i2c_msg.write(i2c_addr, msg)
          i2cbus.i2c_rdwr(write)
      except:
        traceback.print_exc()
    mqtt_client.message_callback_add('homeassistant/switch/solax_pv1_gmppt/set', on_message_solax_pv1_gmppt)
    mqtt_client.publish('homeassistant/switch/solax_pv1_gmppt/config', payload=json.dumps({"name": "PV1 GMPPT", "command_topic": "homeassistant/switch/solax_pv1_gmppt/set"}), retain=True)
    mqtt_client.subscribe('homeassistant/switch/solax_pv1_gmppt/set')

    # force grid tie
    def on_message_solax_force_grid_tie(client, userdata, msg):
      try:
        if msg.payload.decode('utf-8') == "ON":
          msg=b'\x0C'
          with i2clock:
            write = i2c_msg.write(i2c_addr, msg)
            i2cbus.i2c_rdwr(write)
      except:
        traceback.print_exc()
    mqtt_client.message_callback_add('homeassistant/switch/solax_force_grid_tie/set', on_message_solax_force_grid_tie)
    mqtt_client.publish('homeassistant/switch/solax_force_grid_tie/config', payload=json.dumps({"name": "Solax force gridtie", "command_topic": "homeassistant/switch/solax_force_grid_tie/set", "state_topic": "homeassistant/switch/solax_force_grid_tie/state" }), retain=True)
    mqtt_client.subscribe('homeassistant/switch/solax_force_grid_tie/set')

    # force offgrid
    def on_message_solax_force_offgrid(client, userdata, msg):
      try:
        if msg.payload.decode('utf-8') == "ON":
          msg=b'\x0A'
          with i2clock:
            write = i2c_msg.write(i2c_addr, msg)
            i2cbus.i2c_rdwr(write)
      except:
        traceback.print_exc()
    mqtt_client.message_callback_add('homeassistant/switch/solax_force_offgrid/set', on_message_solax_force_offgrid)
    mqtt_client.publish('homeassistant/switch/solax_force_offgrid/config', payload=json.dumps({"name": "Solax force offgrid", "command_topic": "homeassistant/switch/solax_force_offgrid/set", "state_topic": "homeassistant/switch/solax_force_offgrid/state" }), retain=True)
    mqtt_client.subscribe('homeassistant/switch/solax_force_offgrid/set')

    # auto grid connection
    def on_message_solax_auto_grid(client, userdata, msg):
      try:
        if msg.payload.decode('utf-8') == "ON":
          msg=b'\x0B'
          with i2clock:
            write = i2c_msg.write(i2c_addr, msg)
            i2cbus.i2c_rdwr(write)
      except:
        traceback.print_exc()
    mqtt_client.message_callback_add('homeassistant/switch/solax_auto_grid/set', on_message_solax_auto_grid)
    mqtt_client.publish('homeassistant/switch/solax_auto_grid/config', payload=json.dumps({"name": "Solax Auto Grid", "command_topic": "homeassistant/switch/solax_auto_grid/set", "state_topic": "homeassistant/switch/solax_auto_grid/state" }), retain=True)
    mqtt_client.subscribe('homeassistant/switch/solax_auto_grid/set')

    # force stop discharge
    def on_message_solax_force_stop_discharge(client, userdata, msg):
      try:
        if msg.payload.decode('utf-8') == "ON":
          msg=b'\x0D'
          with i2clock:
            write = i2c_msg.write(i2c_addr, msg)
            i2cbus.i2c_rdwr(write)
      except:
        traceback.print_exc()
    mqtt_client.message_callback_add('homeassistant/switch/solax_force_stop_discharge/set', on_message_solax_force_stop_discharge)
    mqtt_client.publish('homeassistant/switch/solax_force_stop_discharge/config', payload=json.dumps({"name": "Solax Force Stop", "command_topic": "homeassistant/switch/solax_force_stop_discharge/set", "state_topic": "homeassistant/switch/solax_force_stop_discharge/state" }), retain=True)
    mqtt_client.subscribe('homeassistant/switch/solax_force_stop_discharge/set')

    # force self use
    def on_message_solax_force_self_use(client, userdata, msg):
      try:
        if msg.payload.decode('utf-8') == "ON":
          msg=b'\x0E'
          with i2clock:
            write = i2c_msg.write(i2c_addr, msg)
            i2cbus.i2c_rdwr(write)
      except:
        traceback.print_exc()
    mqtt_client.message_callback_add('homeassistant/switch/solax_force_self_use/set', on_message_solax_force_self_use)
    mqtt_client.publish('homeassistant/switch/solax_force_self_use/config', payload=json.dumps({"name": "Solax Force Self Use", "command_topic": "homeassistant/switch/solax_force_self_use/set", "state_topic": "homeassistant/switch/solax_force_self_use/state" }), retain=True)
    mqtt_client.subscribe('homeassistant/switch/solax_force_self_use/set')

    # auto mode
    def on_message_solax_auto_mode(client, userdata, msg):
      try:
        if msg.payload.decode('utf-8') == "ON":
          msg=b'\x0F'
          with i2clock:
            write = i2c_msg.write(i2c_addr, msg)
            i2cbus.i2c_rdwr(write)
      except:
        traceback.print_exc()
    mqtt_client.message_callback_add('homeassistant/switch/solax_auto_mode/set', on_message_solax_auto_mode)
    mqtt_client.publish('homeassistant/switch/solax_auto_mode/config', payload=json.dumps({"name": "Solax Auto Self Use", "command_topic": "homeassistant/switch/solax_auto_mode/set", "state_topic": "homeassistant/switch/solax_auto_mode/state" }), retain=True)
    mqtt_client.subscribe('homeassistant/switch/solax_auto_mode/set')

  def on_connect(client, userdata, flags, rc):
    #print(f"on_connect: rc={rc}")
    if rc==0:
      mqtt_setup()

  mqtt_client = mqtt.Client('solax_inverter',clean_session=True)
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
    with i2clock:
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
#(54, 1, 2, 1, -1934, 183, 206, 0, 0, 0, 40, 250, 250, -1, 0, 0, 3208, 0, 0, 2023, 11, 12, 15, 13, 0, 0, 0, 0, 1240465, 0, 1, 1, 0, 2)
# 

    fields = struct.unpack_from(">BBBBhhhhhhhhhhBBhhhHBBBBBBHHIHBBB", data)
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
    IDX_TICKS=28
    IDX_OUTPUT_VA=29
    IDX_SELF_USE_AUTO=30
    IDX_EPS_SWITCH_AUTO=31
    IDX_EPS_FORCED=32
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
    mqtt_client.publish('homeassistant/switch/solax_force_offgrid/state', payload=mqtt_boolstr(fields[IDX_EPS_SWITCH_AUTO]==0 and fields[IDX_EPS_FORCED]!=0), retain=True)
    mqtt_client.publish('homeassistant/switch/solax_force_grid_tie/state', payload=mqtt_boolstr(fields[IDX_EPS_SWITCH_AUTO]==0 and fields[IDX_EPS_FORCED]==0), retain=True)
    mqtt_client.publish('homeassistant/switch/solax_auto_grid/state', payload=mqtt_boolstr(fields[IDX_EPS_SWITCH_AUTO]!=0), retain=True)
    mqtt_client.publish('homeassistant/switch/solax/status', payload=str(fields[IDX_STATE]), retain=True)
    mqtt_client.publish('homeassistant/switch/solax_force_stop_discharge/state', payload=mqtt_boolstr(fields[IDX_SELF_USE_AUTO]==0 and fields[IDX_FORCED_MODE]==3), retain=True)
    mqtt_client.publish('homeassistant/switch/solax_force_self_use/state', payload=mqtt_boolstr(fields[IDX_SELF_USE_AUTO]==0 and fields[IDX_FORCED_MODE]==1), retain=True)
    mqtt_client.publish('homeassistant/switch/solax_auto_mode/state', payload=mqtt_boolstr(fields[IDX_SELF_USE_AUTO]!=0), retain=True)
  except:
    print(traceback.print_exc())
    sys.exit(-1)
  time.sleep(1)