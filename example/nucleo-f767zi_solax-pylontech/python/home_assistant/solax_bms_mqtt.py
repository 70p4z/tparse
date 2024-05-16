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

    mqtt_client.publish('homeassistant/sensor/solax_state/config', payload=json.dumps({"name": "Solax State", "state_topic": "homeassistant/sensor/solax_state/state"}), retain=True)
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

    mqtt_client.publish('homeassistant/sensor/solax_battery_soc_mwh/config', payload=json.dumps({"device_class": "battery", "name": "Solax Battery SoC (mWh)", "state_topic": "homeassistant/sensor/solax_battery_soc_mwh/state", "unit_of_measurement": "%"}), retain=True)

    mqtt_client.publish('homeassistant/sensor/solax_battery_max_charge_current/config', payload=json.dumps({"device_class": "current", "name": "Solax Battery Max Charge Current", "state_topic": "homeassistant/sensor/solax_battery_max_charge_current/state", "unit_of_measurement": "A"}), retain=True)

    mqtt_client.publish('homeassistant/sensor/solax_battery_capacity_mah/config', payload=json.dumps({"device_class": "battery", "name": "Solax Battery Capacity (mAh)", "state_topic": "homeassistant/sensor/solax_battery_capacity_mah/state", "unit_of_measurement": "mAh"}), retain=True)

    mqtt_client.publish('homeassistant/sensor/solax_battery_capacity_mwh/config', payload=json.dumps({"device_class": "battery", "name": "Solax Battery Capacity (mWh)", "state_topic": "homeassistant/sensor/solax_battery_capacity_mwh/state", "unit_of_measurement": "mWh"}), retain=True)

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

    # PV2 GMPPT state
    def on_message_solax_pv2_gmppt(client, userdata, msg):
      try:
        if msg.payload.decode('utf-8') == "ON":
          msg=b'\x04'
        else:
          msg=b'\x03'
        with i2clock:
          write = i2c_msg.write(i2c_addr, msg)
          i2cbus.i2c_rdwr(write)
      except:
        traceback.print_exc()
    mqtt_client.message_callback_add('homeassistant/switch/solax_pv2_gmppt/set', on_message_solax_pv2_gmppt)
    mqtt_client.publish('homeassistant/switch/solax_pv2_gmppt/config', payload=json.dumps({"name": "PV2 GMPPT", "command_topic": "homeassistant/switch/solax_pv2_gmppt/set"}), retain=True)
    mqtt_client.subscribe('homeassistant/switch/solax_pv2_gmppt/set')

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


    def solax_forced_charge_stop():
      try:
        msg=b'\x13'
        with i2clock:
          write = i2c_msg.write(i2c_addr, msg)
          i2cbus.i2c_rdwr(write)
      except:
        traceback.print_exc()
        
    # stop forced charge
    def on_message_solax_forced_charge_stop(client, userdata, msg):
      try:
        if msg.payload.decode('utf-8') == "ON":
          solax_forced_charge_stop()
      except:
        traceback.print_exc()
    mqtt_client.message_callback_add('homeassistant/switch/solax_forced_charge_stop/set', on_message_solax_forced_charge_stop)
    mqtt_client.publish('homeassistant/switch/solax_forced_charge_stop/config', payload=json.dumps({"name": "Solax Stop Forced Charge", "command_topic": "homeassistant/switch/solax_forced_charge_stop/set", "state_topic": "homeassistant/switch/solax_forced_charge_stop/state" }), retain=True)
    mqtt_client.subscribe('homeassistant/switch/solax_forced_charge_stop/set')

    mqtt_client.forced_charge_stop_timer=None

    # start forced charge
    def on_message_solax_forced_charge_start(client, userdata, msg):
      try:
        if msg.payload.decode('utf-8') == "ON":

          # # avoid oups, by disabling forced charge after a while
          # if mqtt_client.forced_charge_stop_timer:
          #   mqtt_client.forced_charge_stop_timer.cancel()
          # mqtt_client.forced_charge_stop_timer = threading.Timer(60, solax_forced_charge_stop)
          # mqtt_client.forced_charge_stop_timer.start()

          msg=b'\x14\x32' # 5.0A
          with i2clock:
            write = i2c_msg.write(i2c_addr, msg)
            i2cbus.i2c_rdwr(write)
      except:
        traceback.print_exc()
    mqtt_client.message_callback_add('homeassistant/switch/solax_forced_charge_start/set', on_message_solax_forced_charge_start)
    mqtt_client.publish('homeassistant/switch/solax_forced_charge_start/config', payload=json.dumps({"name": "Solax Start Forced Charge", "command_topic": "homeassistant/switch/solax_forced_charge_start/set", "state_topic": "homeassistant/switch/solax_forced_charge_start/state"}), retain=True)
    mqtt_client.subscribe('homeassistant/switch/solax_forced_charge_start/set')

    # start forced slow charge
    def on_message_solax_forced_slow_charge_start(client, userdata, msg):
      try:
        if msg.payload.decode('utf-8') == "ON":

          # # avoid oups, by disabling forced charge after a while
          # if mqtt_client.forced_charge_stop_timer:
          #   mqtt_client.forced_charge_stop_timer.cancel()
          # mqtt_client.forced_charge_stop_timer = threading.Timer(60, solax_forced_charge_stop)
          # mqtt_client.forced_charge_stop_timer.start()

          msg=b'\x14\x03'
          with i2clock:
            write = i2c_msg.write(i2c_addr, msg)
            i2cbus.i2c_rdwr(write)
      except:
        traceback.print_exc()
    mqtt_client.message_callback_add('homeassistant/switch/solax_forced_slow_charge_start/set', on_message_solax_forced_slow_charge_start)
    mqtt_client.publish('homeassistant/switch/solax_forced_slow_charge_start/config', payload=json.dumps({"name": "Solax Start Forced Slow Charge", "command_topic": "homeassistant/switch/solax_forced_slow_charge_start/set", "state_topic": "homeassistant/switch/solax_forced_slow_charge_start/state"}), retain=True)
    mqtt_client.subscribe('homeassistant/switch/solax_forced_slow_charge_start/set')

  def on_connect(client, userdata, flags, rc):
    #print(f"on_connect: rc={rc}")
    if rc==0:
      mqtt_setup()

  mqtt_client = mqtt.Client('solax_inverter',clean_session=True)
  mqtt_client.on_connect=on_connect
  while True:
    try:
      mqtt_client.connect('192.168.0.4', 1883)
      mqtt.bms_packs_topic = {}
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
    if length < 2 or data[1] != 2:
      print("Unsupported encoding")
      time.sleep(0.2)
      continue

    """
    b'40'
    b'400207010000000000000000000000003e00fa00faffff070000000000000007e8011512320101000808fc000147860007010101191f61000079db00002e913e'
    (64, 2, 7, 1, 0, 0, 0, 0, 0, 0, 62, 250, 250, -1, 7, 0, 0, 0, 0, 2024, 1, 21, 18, 50, 1, 1, 8, 2300, 83846, 7, 1, 1, 1, 25, 31, 97, 31195, 11921, 62)
    """

    fields = struct.unpack_from(">BBBBhhhhhhBhhhBBhhhHBBBBBBHHIHBBBBBBIIB", data)
    IDX_LEN = 0
    IDX_SCHEMA_VERSION = 1
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
    IDX_CONNECT_SOC=33
    IDX_DISCONNECT_SOC=34
    IDX_MAXCHARGE_SOC=35
    IDX_CAPACITY_MAH=36
    IDX_CAPACITY_MWH=37
    IDX_SOC_MWH=38
    print (repr(fields))

    solax_state_names = ["Waiting", "Checking", "Normal", "Fault", "Permanent Fault", "Update", "EPS waiting", "EPS", "Testing", "Idle", "Standby"]

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
    mqtt_client.publish('homeassistant/sensor/solax_battery_soc_mwh/state', payload=str(fields[IDX_SOC_MWH]), retain=True)
    mqtt_client.publish('homeassistant/sensor/solax_battery_max_charge_current/state', payload=str(fields[IDX_CH]/10.0), retain=True)
    mqtt_client.publish('homeassistant/switch/solax_force_offgrid/state', payload=mqtt_boolstr(fields[IDX_EPS_SWITCH_AUTO]==0 and fields[IDX_EPS_FORCED]!=0), retain=True)
    mqtt_client.publish('homeassistant/switch/solax_force_grid_tie/state', payload=mqtt_boolstr(fields[IDX_EPS_SWITCH_AUTO]==0 and fields[IDX_EPS_FORCED]==0), retain=True)
    mqtt_client.publish('homeassistant/switch/solax_auto_grid/state', payload=mqtt_boolstr(fields[IDX_EPS_SWITCH_AUTO]!=0), retain=True)
    mqtt_client.publish('homeassistant/sensor/solax_state/state', payload=solax_state_names[fields[IDX_STATE]], retain=True)
    mqtt_client.publish('homeassistant/switch/solax_force_stop_discharge/state', payload=mqtt_boolstr(fields[IDX_SELF_USE_AUTO]==0 and fields[IDX_FORCED_MODE]==3), retain=True)
    mqtt_client.publish('homeassistant/switch/solax_force_self_use/state', payload=mqtt_boolstr(fields[IDX_SELF_USE_AUTO]==0 and fields[IDX_FORCED_MODE]==1), retain=True)
    mqtt_client.publish('homeassistant/switch/solax_auto_mode/state', payload=mqtt_boolstr(fields[IDX_SELF_USE_AUTO]!=0), retain=True)
    mqtt_client.publish('homeassistant/sensor/solax_battery_capacity_mah/state', payload=str(fields[IDX_CAPACITY_MAH]), retain=True)
    mqtt_client.publish('homeassistant/sensor/solax_battery_capacity_mwh/state', payload=str(fields[IDX_CAPACITY_MWH]), retain=True)
    mqtt_client.publish('homeassistant/switch/solax_forced_charge_stop/state', payload=mqtt_boolstr(fields[IDX_FORCED_CH] == -1), retain=True)
    mqtt_client.publish('homeassistant/switch/solax_forced_charge_start/state', payload=mqtt_boolstr(fields[IDX_FORCED_CH] != -1), retain=True)

    packs_info=data[64:]
    while (len(packs_info) >= 7):
      pack_fields = struct.unpack_from(">BBBhh", packs_info)
      packs_info = packs_info[7:]
      str_pack_id=hex(0x100+pack_fields[0])[-2:]
      if (pack_fields[1] != 255 and pack_fields[2] != 255 and pack_fields[3] != -1 and pack_fields[4] != -1):
        infos=""
        infos += str(pack_fields[1]) + '/'
        infos += str(pack_fields[2]) + '% '
        infos += str(pack_fields[3]) + '/' +str(pack_fields[4]) + 'mV'

        if not str_pack_id in mqtt.bms_packs_topic:
          mqtt_client.publish('homeassistant/sensor/solax_battery_infos_'+str_pack_id+'/config', payload=json.dumps({"name": "Battery Infos " + str_pack_id, "state_topic": "homeassistant/sensor/solax_battery_infos_"+str_pack_id+"/state"}), retain=True)
          mqtt.bms_packs_topic[str_pack_id] = str_pack_id

        mqtt_client.publish('homeassistant/sensor/solax_battery_infos_'+str_pack_id+'/state', payload=str(infos), retain=True)

  except:
    print(traceback.print_exc())
    sys.exit(-1)
  time.sleep(1)