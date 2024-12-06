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

i2clock = threading.RLock()
try:
  #40 pin connector
  i2cbus = SMBus(1)
  i2c_addr = 0x22 # validated on bus 7bit address
except:
  traceback.print_exc()
  sys.exit(-1)

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
    mqtt_client.publish('homeassistant/sensor/solax_grid_export/config', payload=json.dumps({"device_class": "power", "name": "Solax Grid Export", "state_topic": "homeassistant/sensor/solax_grid_export/state", "unit_of_measurement": "W"}), retain=True)
    mqtt_client.publish('homeassistant/sensor/solax_grid/config', payload=json.dumps({"device_class": "power", "name": "Solax Grid", "state_topic": "homeassistant/sensor/solax_grid/state", "unit_of_measurement": "W"}), retain=True)
    mqtt_client.publish('homeassistant/sensor/solax_pv1/config', payload=json.dumps({"device_class": "power", "name": "Solax PV1", "state_topic": "homeassistant/sensor/solax_pv1/state", "unit_of_measurement": "W"}), retain=True)
    mqtt_client.publish('homeassistant/sensor/solax_pv2/config', payload=json.dumps({"device_class": "power", "name": "Solax PV2", "state_topic": "homeassistant/sensor/solax_pv2/state", "unit_of_measurement": "W"}), retain=True)
    mqtt_client.publish('homeassistant/sensor/solax_battery/config', payload=json.dumps({"device_class": "power", "name": "Solax Battery", "state_topic": "homeassistant/sensor/solax_battery/state", "unit_of_measurement": "W"}), retain=True)
    mqtt_client.publish('homeassistant/sensor/solax_eps/config', payload=json.dumps({"device_class": "power", "name": "Solax EPS", "state_topic": "homeassistant/sensor/solax_eps/state", "unit_of_measurement": "W"}), retain=True)
    mqtt_client.publish('homeassistant/sensor/solax_output/config', payload=json.dumps({"device_class": "power", "name": "Solax Output", "state_topic": "homeassistant/sensor/solax_output/state", "unit_of_measurement": "W"}), retain=True)

    mqtt_client.publish('homeassistant/sensor/solax_pv1_voltage/config', payload=json.dumps({"device_class": "voltage", "name": "Solax PV1 Voltage", "state_topic": "homeassistant/sensor/solax_pv1_voltage/state", "unit_of_measurement": "V"}), retain=True)
    mqtt_client.publish('homeassistant/sensor/solax_pv2_voltage/config', payload=json.dumps({"device_class": "voltage", "name": "Solax PV2 Voltage", "state_topic": "homeassistant/sensor/solax_pv2_voltage/state", "unit_of_measurement": "V"}), retain=True)

    mqtt_client.publish('homeassistant/sensor/solax_battery_soc/config', payload=json.dumps({"device_class": "battery", "name": "Solax Battery SoC", "state_topic": "homeassistant/sensor/solax_battery_soc/state", "unit_of_measurement": "%"}), retain=True)

    mqtt_client.publish('homeassistant/sensor/solax_battery_soc_mwh/config', payload=json.dumps({"device_class": "battery", "name": "Solax Battery SoC (mWh)", "state_topic": "homeassistant/sensor/solax_battery_soc_mwh/state", "unit_of_measurement": "%"}), retain=True)

    mqtt_client.publish('homeassistant/sensor/solax_battery_bms_max_charge_current/config', payload=json.dumps({"device_class": "current", "name": "Solax Battery BMS Max Charge Current", "state_topic": "homeassistant/sensor/solax_battery_bms_max_charge_current/state", "unit_of_measurement": "A"}), retain=True)

    mqtt_client.publish('homeassistant/sensor/solax_battery_max_charge_current/config', payload=json.dumps({"device_class": "current", "name": "Solax Battery Max Charge Current", "state_topic": "homeassistant/sensor/solax_battery_max_charge_current/state", "unit_of_measurement": "A"}), retain=True)

    def on_message_solax_effective_max_charge_current(client, userdata, msg):
      try:
        print("set: " + msg.payload.decode('utf-8'))
        intval = int(float(msg.payload.decode('utf-8'))*10)
        msg=b'\x14' + struct.pack(">B", intval)
        with i2clock:
          write = i2c_msg.write(i2c_addr, msg)
          i2cbus.i2c_rdwr(write)
      except:
        traceback.print_exc()
    mqtt_client.message_callback_add('homeassistant/number/solax_battery_forced_max_charge_current/set', on_message_solax_effective_max_charge_current)
    mqtt_client.subscribe('homeassistant/number/solax_battery_forced_max_charge_current/set')
    mqtt_client.publish('homeassistant/number/solax_battery_forced_max_charge_current/config', payload=json.dumps({"device_class": "current", "name": "Solax Battery Forced Max Charge Current", "state_topic": "homeassistant/number/solax_battery_forced_max_charge_current/state", "unit_of_measurement": "A", "command_topic": "homeassistant/number/solax_battery_forced_max_charge_current/set", "min": "0", "max": "12", "step": "0.1"}), retain=True)

    mqtt_client.publish('homeassistant/sensor/solax_battery_capacity_mah/config', payload=json.dumps({"device_class": "battery", "name": "Solax Battery Capacity (mAh)", "state_topic": "homeassistant/sensor/solax_battery_capacity_mah/state", "unit_of_measurement": "mAh"}), retain=True)

    mqtt_client.publish('homeassistant/sensor/solax_battery_capacity_mwh/config', payload=json.dumps({"device_class": "battery", "name": "Solax Battery Capacity (Wh)", "state_topic": "homeassistant/sensor/solax_battery_capacity_mwh/state", "unit_of_measurement": "Wh"}), retain=True)

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

    # force charge (to top up batteries)
    def on_message_solax_force_charge(client, userdata, msg):
      try:
        if msg.payload.decode('utf-8') == "ON":
          msg=b'\x15'
          with i2clock:
            write = i2c_msg.write(i2c_addr, msg)
            i2cbus.i2c_rdwr(write)
      except:
        traceback.print_exc()
    mqtt_client.message_callback_add('homeassistant/switch/solax_force_charge/set', on_message_solax_force_charge)
    mqtt_client.publish('homeassistant/switch/solax_force_charge/config', payload=json.dumps({"name": "Solax Force Charge From Grid", "command_topic": "homeassistant/switch/solax_force_charge/set", "state_topic": "homeassistant/switch/solax_force_charge/state" }), retain=True)
    mqtt_client.subscribe('homeassistant/switch/solax_force_charge/set')

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

    def on_message_solax_auto_charge_current(client, userdata, msg):
      try:
        if msg.payload.decode('utf-8') == "ON":
          msg=b'\x13'
          with i2clock:
            write = i2c_msg.write(i2c_addr, msg)
            i2cbus.i2c_rdwr(write)
      except:
        traceback.print_exc()
    mqtt_client.message_callback_add('homeassistant/switch/solax_auto_charge_current/set', on_message_solax_auto_charge_current)
    mqtt_client.publish('homeassistant/switch/solax_auto_charge_current/config', payload=json.dumps({"name": "Solax Battery Auto Charge Current", "command_topic": "homeassistant/switch/solax_auto_charge_current/set", "state_topic": "homeassistant/switch/solax_auto_charge_current/state"}), retain=True)
    mqtt_client.subscribe('homeassistant/switch/solax_auto_charge_current/set')


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

    #check length and data schema version
    if length < 2 or data[1] != 3:
      print("Unsupported encoding")
      time.sleep(0.2)
      continue

    """
    v2
      b'40'
      b'400207010000000000000000000000003e00fa00faffff070000000000000007e8011512320101000808fc000147860007010101191f61000079db00002e913e'
      (64, 2, 7, 1, 0, 0, 0, 0, 0, 0, 62, 250, 250, -1, 7, 0, 0, 0, 0, 2024, 1, 21, 18, 50, 1, 1, 8, 2300, 83846, 7, 1, 1, 1, 25, 31, 97, 31195, 11921, 62)
    v3
      b'79'
      b'7903070400000000008e004d00a700b064000000fa000707030002c102c1000007e80b1c11100000000a08fb003a0895000c000101141e640000c28e00004a7f639864640dc40dcf9164640dd10dfc6664640df50e0c3464620d720e0f2464640db00de42264610d740d954764640dcb0e0e1364630d320dcd'
      (121, 3, 7, 4, 0, 0, 142, 77, 167, 176, 100, 0, 250, 7, 7, 3, 0, 705, 705, 0, 2024, 11, 28, 17, 16, 0, 0, 10, 2299, 3803285, 12, 0, 1, 1, 20, 30, 100, 49806, 19071, 99)

    """



    fields = struct.unpack_from(">BBBBhhhhhhBhhhbBBhhhHBBBBBBHHIHBBBBBBIIB", data)

    (IDX_LEN,
    IDX_SCHEMA_VERSION,
    IDX_STATE,
    IDX_FORCED_MODE,
    IDX_GRID_EXPORT,
    IDX_GRID,
    IDX_PV1,
    IDX_PV2,
    IDX_INV_BAT,
    IDX_BMS_BAT,
    IDX_SOC,
    IDX_CH,
    IDX_DISCH,
    IDX_COMPUTED_CH,
    IDX_FORCED_CH,
    IDX_OPT_RULE,
    IDX_MODE,
    IDX_PV1_VOLTAGE,
    IDX_PV2_VOLTAGE,
    IDX_EPS_CURRENT,
    IDX_Y,
    IDX_M,
    IDX_D,
    IDX_HR,
    IDX_MN,
    IDX_PV1_SWITCH_ON,
    IDX_PV2_SWITCH_ON,
    IDX_EPS_POWER,
    IDX_EPS_VOLTAGE,
    IDX_TICKS,
    IDX_OUTPUT_VA,
    IDX_SELF_USE_AUTO,
    IDX_EPS_SWITCH_AUTO,
    IDX_EPS_FORCED,
    IDX_CONNECT_SOC,
    IDX_DISCONNECT_SOC,
    IDX_MAXCHARGE_SOC,
    IDX_CAPACITY_MAH,
    IDX_CAPACITY_MWH,
    IDX_SOC_MWH) = range(0, 40)
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
    mqtt_client.publish('homeassistant/sensor/solax_battery_bms_max_charge_current/state', payload=str(fields[IDX_CH]/10.0), retain=True)
    if (fields[IDX_FORCED_CH] == -1):
      mqtt_client.publish('homeassistant/number/solax_battery_forced_max_charge_current/state', payload=str(0), retain=True)
    else:
      mqtt_client.publish('homeassistant/number/solax_battery_forced_max_charge_current/state', payload=str(fields[IDX_FORCED_CH]/10.0), retain=True)
    mqtt_client.publish('homeassistant/sensor/solax_battery_max_charge_current/state', payload=str(fields[IDX_COMPUTED_CH]/10.0), retain=True)
    mqtt_client.publish('homeassistant/switch/solax_force_offgrid/state', payload=mqtt_boolstr(fields[IDX_EPS_SWITCH_AUTO]==0 and fields[IDX_EPS_FORCED]!=0), retain=True)
    mqtt_client.publish('homeassistant/switch/solax_force_grid_tie/state', payload=mqtt_boolstr(fields[IDX_EPS_SWITCH_AUTO]==0 and fields[IDX_EPS_FORCED]==0), retain=True)
    mqtt_client.publish('homeassistant/switch/solax_auto_grid/state', payload=mqtt_boolstr(fields[IDX_EPS_SWITCH_AUTO]!=0), retain=True)
    mqtt_client.publish('homeassistant/sensor/solax_state/state', payload=solax_state_names[fields[IDX_STATE]], retain=True)
    mqtt_client.publish('homeassistant/switch/solax_force_stop_discharge/state', payload=mqtt_boolstr(fields[IDX_SELF_USE_AUTO]==0 and fields[IDX_FORCED_MODE]==3), retain=True)
    mqtt_client.publish('homeassistant/switch/solax_force_charge/state', payload=mqtt_boolstr(fields[IDX_SELF_USE_AUTO]==0 and fields[IDX_FORCED_MODE]==4), retain=True)
    mqtt_client.publish('homeassistant/switch/solax_force_self_use/state', payload=mqtt_boolstr(fields[IDX_SELF_USE_AUTO]==0 and fields[IDX_FORCED_MODE]==1), retain=True)
    mqtt_client.publish('homeassistant/switch/solax_auto_mode/state', payload=mqtt_boolstr(fields[IDX_SELF_USE_AUTO]!=0), retain=True)
    mqtt_client.publish('homeassistant/sensor/solax_battery_capacity_mah/state', payload=str(fields[IDX_CAPACITY_MAH]), retain=True)
    mqtt_client.publish('homeassistant/sensor/solax_battery_capacity_mwh/state', payload=str(fields[IDX_CAPACITY_MWH]), retain=True)
    mqtt_client.publish("homeassistant/switch/solax_auto_charge_current/state", payload=mqtt_boolstr(fields[IDX_FORCED_CH] == -1), retain=True)
    
    packs_info=data[65:]
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