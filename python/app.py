import paho.mqtt.client as mqtt
from gpiozero import LED
import time
import spidev
import config

spi = spidev.SpiDev()
spi.open(config.spi["bus"], config.spi["device"])
spi.max_speed_hz = config.spi["max_speed_hz"]
spi.mode = config.spi["mode"]

relay = [False, False, False, False, False]
relayHasUpdate = False

relayLastUpdate = time.time()
tAndHLastUpdate = time.time()
lightLastUpdate = time.time()
pmLastUpdate = time.time()

rebootStmPin = LED(config.spi["reboot_stm"], active_high=False)
rebootStmPin.off()

def main():
  global relayHasUpdate
  global relayLastUpdate
  global tAndHLastUpdate
  global lightLastUpdate
  global pmLastUpdate

  mqttClient = mqtt.Client()
  mqttClient.on_connect = mqttOnConnect
  mqttClient.on_message = mqttOnMessage
  mqttClient.username_pw_set(username=config.mqtt["mqttUser"], password=config.mqtt["mqttPass"])
  mqttClient.connect(host=config.mqtt["mqttHost"], port=config.mqtt["mqttPort"])
  mqttClient.loop_start()

  try:
    while True:
      now = time.time()

      if (relayHasUpdate and (now - relayLastUpdate) > 0.2):
        relayHasUpdate = False
        print("Relay update: " + str(relay))

        relayValue = boolean_array_to_int(relay)
        command = [1, 1, 0, 0, 0, relayValue]
        crc = crc16_modbus(command)
        commandCrc = command + crc

        print(f"Relay command {commandCrc}")

        spi.xfer2(commandCrc)

        time.sleep(config.spi["delay"])

        receivedData = spi.readbytes(8)

        if not validate_crc16_modbus(receivedData):
           rebootStm()

        print(receivedData)

      if (now - tAndHLastUpdate > config.sensor["tAndH"]):
        tAndHLastUpdate = now

        command = [2, 1, 0, 0, 0, 0]
        crc = crc16_modbus(command)
        commandCrc = command + crc

        spi.xfer2(commandCrc)

        time.sleep(config.spi["delay"])

        receivedData = spi.readbytes(8)

        if validate_crc16_modbus(receivedData):
          temperature = receivedData[4] / 10
          humidity = receivedData[5] / 10

          if (temperature <= 0 or temperature > 50):
            rebootStm()
            continue

          # if (humidity < 10 or humidity > 90):
          #    rebootStm()
          #    continue

          mqttClient.publish(config.ha["discoveryPrefix"] + "/sensor/temperature/state", temperature, retain=True)
          mqttClient.publish(config.ha["discoveryPrefix"] + "/sensor/humidity/state", humidity, retain=True)

          print(f"Temperature: {temperature}°C, Humidity: {humidity}%")

      if (now - pmLastUpdate > config.sensor["pm"]):
        pmLastUpdate = now

        pm = [0, 0, 0]

        for i in range(3):
          command = [3, i + 1, 0, 0, 0, 0]
          crc = crc16_modbus(command)
          commandCrc = command + crc

          spi.xfer2(commandCrc)

          time.sleep(config.spi["delay"])

          receivedData = spi.readbytes(8)

          if validate_crc16_modbus(receivedData):
            pm[i] = (receivedData[5] << 8) | receivedData[4]
          else:
            rebootStm()

        if (pm[0] == 0 or pm[1] == 0 or pm[2] == 0):
          rebootStm()
          continue

        mqttClient.publish(config.ha["discoveryPrefix"] + "/sensor/pm1/state", pm[0], retain=True)
        mqttClient.publish(config.ha["discoveryPrefix"] + "/sensor/pm25/state", pm[1], retain=True)
        mqttClient.publish(config.ha["discoveryPrefix"] + "/sensor/pm10/state", pm[2], retain=True)

        print(f"PM1: {pm[0]} µg/m³, PM2.5: {pm[1]} µg/m³, PM10: {pm[2]} µg/m³")


      if (now - lightLastUpdate > config.sensor["light"]):
        lightLastUpdate = now

        command = [4, 1, 0, 0, 0, 0]
        crc = crc16_modbus(command)
        commandCrc = command + crc

        spi.xfer2(commandCrc)

        time.sleep(config.spi["delay"])

        receivedData = spi.readbytes(8)

        if validate_crc16_modbus(receivedData):
          light = (receivedData[5] << 8) | receivedData[4]

          mqttClient.publish(config.ha["discoveryPrefix"] + "/sensor/light/state", light, retain=True)

          print(f"Light: {light} lux")

        else:
          rebootStm()

  except KeyboardInterrupt:
    mqttClient.disconnect()
    mqttClient.loop_stop()

def mqttOnConnect(self, client, userdata, rc):
  print("MQTT Connected with result code "+str(rc))

  sensorDiscovery(self)

  # Subscribe to relay switches
  for i in range(len(relay)):
    self.subscribe(config.ha["discoveryPrefix"] + "/switch/relay" + str(i) + "/cmd")

def mqttOnMessage(self, userdata, message):
  global relayLastUpdate
  global relayHasUpdate

  # Relay switch
  for i in range(len(relay)):
    if message.topic == config.ha["discoveryPrefix"] + "/switch/relay" + str(i) + "/cmd":
      relayLastUpdate = time.time()
      relayHasUpdate = True
      if message.payload.decode("utf-8") == "ON":
        relay[i] = True
      elif message.payload.decode("utf-8") == "OFF":
        relay[i] = False

  for i in range(len(relay)):
    topic = config.ha["discoveryPrefix"] + "/switch/relay" + str(i) + "/state"
    self.publish(topic, "ON" if relay[i] else "OFF", retain=True)

def sensorDiscovery(client):
  # Relay switch discovery
  for i in range(len(relay)):
    topic = config.ha["discoveryPrefix"] + "/switch/relay" + str(i) + "/config"
    payload = {
      "name": "Relay " + str(i),
      "unique_id": "relay_" + str(i),
      "command_topic": config.ha["discoveryPrefix"] + "/switch/relay" + str(i) + "/cmd",
      "state_topic": config.ha["discoveryPrefix"] + "/switch/relay" + str(i) + "/state",
      "payload_on": "ON",
      "payload_off": "OFF",
      "retain": "true"
    }

    payload = str(payload).replace("'", "\"")
    client.publish(topic, payload)

    topic = config.ha["discoveryPrefix"] + "/switch/relay" + str(i) + "/state"
    client.publish(topic, "ON" if relay[i] else "OFF", retain=True)

    topic = config.ha["discoveryPrefix"] + "/switch/relay" + str(i) + "/cmd"
    client.subscribe(topic)

  # Temperature discovery
  topic = config.ha["discoveryPrefix"] + "/sensor/temperature/config"
  payload = {
    "name": "Temperature",
    "unique_id": "temperature",
    "state_topic": config.ha["discoveryPrefix"] + "/sensor/temperature/state",
    "unit_of_measurement": "°C",
    "device_class": "temperature",
    "retain": "true"
  }

  payload = str(payload).replace("'", "\"")
  client.publish(topic, payload)

  # Humidity discovery
  topic = config.ha["discoveryPrefix"] + "/sensor/humidity/config"
  payload = {
    "name": "Humidity",
    "unique_id": "humidity",
    "state_topic": config.ha["discoveryPrefix"] + "/sensor/humidity/state",
    "unit_of_measurement": "%",
    "device_class": "humidity",
    "retain": "true"
  }

  payload = str(payload).replace("'", "\"")
  client.publish(topic, payload)

  # Light discovery
  topic = config.ha["discoveryPrefix"] + "/sensor/light/config"
  payload = {
    "name": "Light",
    "unique_id": "light",
    "state_topic": config.ha["discoveryPrefix"] + "/sensor/light/state",
    "unit_of_measurement": "lux",
    "device_class": "illuminance",
    "retain": "true"
  }

  payload = str(payload).replace("'", "\"")
  client.publish(topic, payload)

  # particulate matter (1) discovery
  topic = config.ha["discoveryPrefix"] + "/sensor/pm1/config"
  payload = {
    "name": "Particulate Matter 1",
    "unique_id": "pm1",
    "state_topic": config.ha["discoveryPrefix"] + "/sensor/pm1/state",
    "unit_of_measurement": "µg/m³",
    "device_class": "pm1",
    "retain": "true"
  }

  payload = str(payload).replace("'", "\"")
  client.publish(topic, payload)

  # particulate matter (2.5) discovery
  topic = config.ha["discoveryPrefix"] + "/sensor/pm25/config"
  payload = {
    "name": "Particulate Matter 2.5",
    "unique_id": "pm25",
    "state_topic": config.ha["discoveryPrefix"] + "/sensor/pm25/state",
    "unit_of_measurement": "µg/m³",
    "device_class": "pm25",
    "retain": "true"
  }

  payload = str(payload).replace("'", "\"")
  client.publish(topic, payload)

  # particulate matter (10) discovery
  topic = config.ha["discoveryPrefix"] + "/sensor/pm10/config"
  payload = {
    "name": "Particulate Matter 10",
    "unique_id": "pm10",
    "state_topic": config.ha["discoveryPrefix"] + "/sensor/pm10/state",
    "unit_of_measurement": "µg/m³",
    "device_class": "pm10",
    "retain": "true"
  }

  payload = str(payload).replace("'", "\"")
  client.publish(topic, payload)


def crc16_modbus(data: bytes) -> list:
    """Calculate the CRC16 Modbus checksum and return as a list of 2 bytes (MSB, LSB)."""
    crc = 0xFFFF
    for pos in data:
        crc ^= pos
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    # Return as two bytes: MSB first, then LSB
    msb = (crc >> 8) & 0xFF  # Most significant byte
    lsb = crc & 0xFF         # Least significant byte
    return [msb, lsb]

def validate_crc16_modbus(data: bytes) -> bool:
    """Validate the CRC16 Modbus checksum for an 8-byte data packet."""
    if len(data) != 8:
        raise ValueError("Data packet must be 8 bytes long (6 data + 2 CRC).")

    # Split the data into payload (first 6 bytes) and CRC (last 2 bytes)
    payload = data[:-2]
    crc_received = list(data[-2:])  # Last two bytes as a list (MSB first)

    # Calculate the CRC16 on the first 6 bytes of the payload
    crc_calculated = crc16_modbus(payload)

    # Compare calculated CRC with the received CRC
    return crc_received == crc_calculated

def boolean_array_to_int(bool_array):
    # Pad with False (0) at the beginning if the array has fewer than 8 elements
    bool_array = [False] * (8 - len(bool_array)) + bool_array

    # Convert boolean array to an integer, treating it like bits in an 8-bit integer
    return sum(val << (7 - idx) for idx, val in enumerate(bool_array))

def rebootStm():
  print("Reboot STM")

  rebootStmPin.on()
  time.sleep(0.1)
  rebootStmPin.off()

  time.sleep(1)

  relayValue = boolean_array_to_int(relay)
  command = [1, 1, 0, 0, 0, relayValue]
  crc = crc16_modbus(command)
  commandCrc = command + crc

  print(f"Relay command {commandCrc}")

  spi.xfer2(commandCrc)

  time.sleep(config.spi["delay"])

  receivedData = spi.readbytes(8)
  print(receivedData)

rebootStm()

if __name__ == '__main__':
  main()