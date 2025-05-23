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
  from smbus2 import SMBus
  from smbus2 import i2c_msg
  #40 pin connector
  try:
    # map bus 3 to gpio 0 and 1
    #/boot/config.txt: dtoverlay=i2c-gpio,bus=0,i2c_gpio_sda=0,i2c_gpio_scl=1,i2c_gpio_delay_us=30
    #   +-----+-----+---------+------+---+-Pi ZeroW-+---+------+---------+-----+-----+
    #   | BCM | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | BCM |
    #   +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+
    #   |     |     |    3.3v |      |   |  1 || 2  |   |      | 5v      |     |     |
    #   |   2 |   8 |   SDA.1 | ALT0 | 1 |  3 || 4  |   |      | 5v      |     |     |
    #   |   3 |   9 |   SCL.1 | ALT0 | 0 |  5 || 6  |   |      | 0v      |     |     |
    #   |   4 |   7 | GPIO. 7 |   IN | 0 |  7 || 8  | 1 | ALT0 | TxD     | 15  | 14  |
    #   |     |     |      0v |      |   |  9 || 10 | 1 | ALT0 | RxD     | 16  | 15  |
    #   |  17 |   0 | GPIO. 0 |  OUT | 0 | 11 || 12 | 0 | IN   | GPIO. 1 | 1   | 18  |
    #   |  27 |   2 | GPIO. 2 |   IN | 0 | 13 || 14 |   |      | 0v      |     |     |
    #   |  22 |   3 | GPIO. 3 |   IN | 0 | 15 || 16 | 0 | IN   | GPIO. 4 | 4   | 23  |
    #   |     |     |    3.3v |      |   | 17 || 18 | 0 | IN   | GPIO. 5 | 5   | 24  |
    #   |  10 |  12 |    MOSI |   IN | 0 | 19 || 20 |   |      | 0v      |     |     |
    #   |   9 |  13 |    MISO |   IN | 0 | 21 || 22 | 0 | IN   | GPIO. 6 | 6   | 25  |
    #   |  11 |  14 |    SCLK |   IN | 0 | 23 || 24 | 1 | IN   | CE0     | 10  | 8   |
    #   |     |     |      0v |      |   | 25 || 26 | 1 | IN   | CE1     | 11  | 7   |
    #>>>|   0 |  30 |   SDA.0 |   IN | 1 | 27 || 28 | 1 | IN   | SCL.0   | 31  | 1   |
    #   |   5 |  21 | GPIO.21 |   IN | 1 | 29 || 30 |   |      | 0v      |     |     |
    #   |   6 |  22 | GPIO.22 |   IN | 1 | 31 || 32 | 0 | IN   | GPIO.26 | 26  | 12  |
    #   |  13 |  23 | GPIO.23 |   IN | 0 | 33 || 34 |   |      | 0v      |     |     |
    #   |  19 |  24 | GPIO.24 |   IN | 0 | 35 || 36 | 0 | IN   | GPIO.27 | 27  | 16  |
    #   |  26 |  25 | GPIO.25 |   IN | 0 | 37 || 38 | 0 | IN   | GPIO.28 | 28  | 20  |
    #   |     |     |      0v |      |   | 39 || 40 | 0 | IN   | GPIO.29 | 29  | 21  |
    #   +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+
    #   | BCM | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | BCM |
    #   +-----+-----+---------+------+---+-Pi ZeroW-+---+------+---------+-----+-----+
    i2cbus = SMBus(3)
  except:
    i2cbus = SMBus(1)
  i2c_addr = 0x22 # validated on bus 7bit address

  def i2c_write(data):
    with i2clock:
      write = i2c_msg.write(i2c_addr, data)
      i2cbus.i2c_rdwr(write)  

  def i2c_write_read(datawrite, lenread):
    with i2clock:
      write = i2c_msg.write(i2c_addr, datawrite)
      read = i2c_msg.read(i2c_addr, lenread)
      i2cbus.i2c_rdwr(write, read)
      data = b''
      for v in read:
        data+=bytes([v])
      return data
except:
  import serial
  from iobridge.iobridge import IOBridge
  i2c_addr = 0x44 # validated on bus 8bit address (iobridge)
  port = sys.argv[1]
  iob = IOBridge(serial.Serial(port=port, baudrate=921600))

  def i2c_write(data):
    with i2clock:
      iob.i2c_write(i2c_addr, data)

  def i2c_write_read(datawrite, lenread):
    with i2clock:
      iob.i2c_write(i2c_addr, datawrite)
      return iob.i2c_read(i2c_addr, lenread)

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

    def on_message_solax_forced_max_charge_current(client, userdata, msg):
      try:
        print("set: " + msg.payload.decode('utf-8'))
        if float(msg.payload.decode('utf-8')) >= 0:
          intval = int(float(msg.payload.decode('utf-8'))*10)
          msg=b'\x14' + struct.pack(">B", intval)
        else:
          msg=b'\x13'
        i2c_write(msg)
      except:
        traceback.print_exc()
    mqtt_client.message_callback_add('homeassistant/number/solax_battery_forced_max_charge_current/set', on_message_solax_forced_max_charge_current)
    mqtt_client.subscribe('homeassistant/number/solax_battery_forced_max_charge_current/set')
    mqtt_client.publish('homeassistant/number/solax_battery_forced_max_charge_current/config', payload=json.dumps({"device_class": "current", "name": "Solax Battery Forced Max Charge Current", "state_topic": "homeassistant/number/solax_battery_forced_max_charge_current/state", "unit_of_measurement": "A", "command_topic": "homeassistant/number/solax_battery_forced_max_charge_current/set", "min": "-0.1", "max": "3", "step": "0.1"}), retain=True)

    def on_message_solax_maxcharge_voltage(client, userdata, msg):
      try:
        print("set: " + msg.payload.decode('utf-8'))
        intval = int(float(msg.payload.decode('utf-8'))*10)
        msg=b'\x17' + struct.pack(">B", intval)
        i2c_write(msg)
      except:
        traceback.print_exc()
    mqtt_client.message_callback_add('homeassistant/number/solax_battery_max_charge_voltage/set', on_message_solax_maxcharge_voltage)
    mqtt_client.subscribe('homeassistant/number/solax_battery_max_charge_voltage/set')
    mqtt_client.publish('homeassistant/number/solax_battery_max_charge_voltage/config', payload=json.dumps({"device_class": "voltage", "name": "Solax Battery Max Charge Voltage", "state_topic": "homeassistant/number/solax_battery_max_charge_voltage/state", "unit_of_measurement": "V", "command_topic": "homeassistant/number/solax_battery_max_charge_voltage/set", "min": "3.6", "max": "3.8", "step": "0.1"}), retain=True)

    mqtt_client.publish('homeassistant/sensor/solax_battery_capacity_mah/config', payload=json.dumps({"device_class": "battery", "name": "Solax Battery Capacity (mAh)", "state_topic": "homeassistant/sensor/solax_battery_capacity_mah/state", "unit_of_measurement": "mAh"}), retain=True)

    mqtt_client.publish('homeassistant/sensor/solax_battery_capacity_mwh/config', payload=json.dumps({"device_class": "battery", "name": "Solax Battery Capacity (Wh)", "state_topic": "homeassistant/sensor/solax_battery_capacity_mwh/state", "unit_of_measurement": "Wh"}), retain=True)

    # PV1 GMPPT state
    def on_message_solax_pv1_gmppt(client, userdata, msg):
      try:
        if msg.payload.decode('utf-8') == "ON":
          msg=b'\x02'
        else:
          msg=b'\x01'
        i2c_write(msg)
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
        i2c_write(msg)
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
          i2c_write(msg)
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
          i2c_write(msg)
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
          i2c_write(msg)
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
          i2c_write(msg)
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
          i2c_write(msg)
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
          i2c_write(msg)
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
          i2c_write(msg)
      except:
        traceback.print_exc()
    mqtt_client.message_callback_add('homeassistant/switch/solax_auto_mode/set', on_message_solax_auto_mode)
    mqtt_client.publish('homeassistant/switch/solax_auto_mode/config', payload=json.dumps({"name": "Solax Auto Self Use", "command_topic": "homeassistant/switch/solax_auto_mode/set", "state_topic": "homeassistant/switch/solax_auto_mode/state" }), retain=True)
    mqtt_client.subscribe('homeassistant/switch/solax_auto_mode/set')

    def on_message_solax_forced_soc(client, userdata, msg):
      try:
        print("set: " + msg.payload.decode('utf-8'))
        intval = int(msg.payload.decode('utf-8'))
        msg=b'\x18' + struct.pack(">b", intval)
        i2c_write(msg)
      except:
        traceback.print_exc()
    mqtt_client.message_callback_add('homeassistant/number/solax_battery_forced_soc/set', on_message_solax_forced_soc)
    mqtt_client.subscribe('homeassistant/number/solax_battery_forced_soc/set')
    mqtt_client.publish('homeassistant/number/solax_battery_forced_soc/config', payload=json.dumps({"device_class": "voltage", "name": "Solax Battery Forced SoC", "state_topic": "homeassistant/number/solax_battery_forced_soc/state", "unit_of_measurement": "%", "command_topic": "homeassistant/number/solax_battery_forced_soc/set", "min": "-1", "max": "104", "step": "5"}), retain=True)

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
    print("<")
    length = i2c_write_read(b'\x00', 1)[0]
    print(">")
    data = i2c_write_read(b'\x00', length)
    print(binascii.hexlify(data))

    #check length and data schema version
    if length < 2 or data[1] != 4:
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



    fields = struct.unpack_from(">BBBBhhhhhhBhhhbBBhhhHBBBBBBHHIHBBBBBBIIBBb", data)

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
    IDX_SOC_MWH,
    IDX_MAX_CHG_VOLTAGE,
    IDX_FORCED_SOC) = range(0, 42)
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
      mqtt_client.publish('homeassistant/number/solax_battery_forced_max_charge_current/state', payload=str(-0.1), retain=True)
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
    mqtt_client.publish("homeassistant/number/solax_battery_max_charge_voltage/state", payload=str(fields[IDX_MAX_CHG_VOLTAGE]/10), retain=True)
    mqtt_client.publish("homeassistant/number/solax_battery_forced_soc/state", payload=str(fields[IDX_FORCED_SOC]), retain=True)
    
    packs_info=data[67:]
    pack_idx=0
    while (len(packs_info) >= 7):
      pack_fields = struct.unpack_from(">BBBhh", packs_info)
      packs_info = packs_info[7:]
      str_pack_id=hex(0x100+pack_idx)[-2:]+"_"+hex(0x100+pack_fields[0])[-2:]
      if (pack_fields[1] != 255 and pack_fields[2] != 255 and pack_fields[3] != -1 and pack_fields[4] != -1):
        infos=""
        infos += str(pack_fields[1]) + '/'
        infos += str(pack_fields[2]) + '% '
        infos += str(pack_fields[3]) + '/' +str(pack_fields[4]) + 'mV'

        if not str_pack_id in mqtt.bms_packs_topic:
          mqtt_client.publish('homeassistant/sensor/solax_battery_infos_'+str_pack_id+'/config', payload=json.dumps({"name": "Battery Infos " + str_pack_id, "state_topic": "homeassistant/sensor/solax_battery_infos_"+str_pack_id+"/state"}), retain=True)
          mqtt.bms_packs_topic[str_pack_id] = str_pack_id

        mqtt_client.publish('homeassistant/sensor/solax_battery_infos_'+str_pack_id+'/state', payload=str(infos), retain=True)
      pack_idx += 1

  except:
    print(traceback.print_exc())
    sys.exit(-1)
  time.sleep(1)