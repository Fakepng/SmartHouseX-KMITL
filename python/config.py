mqtt = dict(
  mqttHost = "localhost",
  mqttPort = 1883,
  mqttUser = "test",
  mqttPass = "test"
)

ha = dict(
  discoveryPrefix = "homeassistant",
)

spi = dict(
  bus = 0,
  device = 0,
  max_speed_hz = 50_000,
  mode = 0,
  delay = 0.2,
  reboot_stm = 5
)

button = dict(
  numberOfBtn = 8,
  stmFlagPin = 25
)

sensor = dict(
  tAndH = 10,
  light = 5,
  pm = 5
)