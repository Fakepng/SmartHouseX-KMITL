#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoHA.h>
#include <JC_Button.h>
#include "esp_adc_cal.h"

const char* SSID = "Fakepng-IoT";
const char* PASSWORD = "iotengineering";
const char* BROKER_ADDRESS = "project.local";
const char* BROKER_USERNAME = "remote";
const char* BROKER_PASSWORD = "remotepassword";
const char* BROKER_CLIENT_ID = "esp32remote";

const int btn1_pin = 25; const char* btn1_name = "btn1"; bool btn1_holding = false;
const int btn2_pin = 26; const char* btn2_name = "btn2"; bool btn2_holding = false;
const int btn3_pin = 32; const char* btn3_name = "btn3"; bool btn3_holding = false;
const int btn4_pin = 33; const char* btn4_name = "btn4"; bool btn4_holding = false;

const int LED_BUILD_IN = 2;
const int BATTERY_SENSING_PIN = 36;

const int battery_buffer_size = 15;
int battery_buffer[battery_buffer_size] = {0};
int battery_i = 0;
int battery_raw = 0;
int battery_Filtered = 0;
uint32_t battery_voltage = 0;
int battery_sample_rate = 30 * 1000;
unsigned long battery_last_sample = 0;

WiFiClient wifiClient;

HADevice device;
HAMqtt mqtt(wifiClient, device);

HADeviceTrigger btn1ShortPressTrigger(HADeviceTrigger::ButtonShortPressType, btn1_name);
HADeviceTrigger btn1LongPressTrigger(HADeviceTrigger::ButtonLongPressType, btn1_name);
Button btn1(btn1_pin);

HADeviceTrigger btn2ShortPressTrigger(HADeviceTrigger::ButtonShortPressType, btn2_name);
HADeviceTrigger btn2LongPressTrigger(HADeviceTrigger::ButtonLongPressType, btn2_name);
Button btn2(btn2_pin);

HADeviceTrigger btn3ShortPressTrigger(HADeviceTrigger::ButtonShortPressType, btn3_name);
HADeviceTrigger btn3LongPressTrigger(HADeviceTrigger::ButtonLongPressType, btn3_name);
Button btn3(btn3_pin);

HADeviceTrigger btn4ShortPressTrigger(HADeviceTrigger::ButtonShortPressType, btn4_name);
HADeviceTrigger btn4LongPressTrigger(HADeviceTrigger::ButtonLongPressType, btn4_name);
Button btn4(btn4_pin);

HASensorNumber voltage("battery_voltage", HASensorNumber::PrecisionP2);

HASensorNumber percentage("battery_percentage", HASensorNumber::PrecisionP2);

void setup() {
  Serial.begin(115200);

  pinMode(LED_BUILD_IN, OUTPUT);
  pinMode(BATTERY_SENSING_PIN, INPUT);

  byte mac[6];
  WiFi.macAddress(mac);
  device.setName("ESP32 Remote");
  device.setUniqueId(mac, sizeof(mac));

  btn1.begin();
  btn2.begin();
  btn3.begin();
  btn4.begin();

  voltage.setIcon("mdi:battery");
  voltage.setName("battery_voltage");
  voltage.setUnitOfMeasurement("V");
  voltage.setDeviceClass("voltage");

  percentage.setIcon("mdi:battery");
  percentage.setName("battery_percentage");
  percentage.setUnitOfMeasurement("%");
  percentage.setDeviceClass("battery");

  Serial.print("Connecting to ");
  Serial.print(SSID);
  WiFi.begin(SSID, PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected");

  mqtt.begin(BROKER_ADDRESS, BROKER_USERNAME, BROKER_PASSWORD);

  Serial.println("Start");
}

uint32_t ADC_Calculate(int raw_adc) {
  esp_adc_cal_characteristics_t adc_chars;

  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);

  return esp_adc_cal_raw_to_voltage(raw_adc, &adc_chars);
}

int ADC_Average(int raw_adc) {
  int sum = 0;

  battery_buffer[battery_i] = raw_adc;
  battery_i = (battery_i + 1) % battery_buffer_size;

  for (int i = 0; i < battery_buffer_size; i++) {
    sum += battery_buffer[i];
  }

  return sum / battery_buffer_size;
}

double mapf(double val, double in_min, double in_max, double out_min, double out_max) {
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_BUILD_IN, LOW);

    WiFi.begin(SSID, PASSWORD);

    Serial.println("Reconnecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
  }

  unsigned long now = millis();

  digitalWrite(LED_BUILD_IN, HIGH);

  mqtt.loop();

  btn1.read();
  btn2.read();
  btn3.read();
  btn4.read();

  if (btn1.pressedFor(500) && !btn1_holding) {
    btn1LongPressTrigger.trigger();
    Serial.println("Button 1 Long Press");
    btn1_holding = true;
  } else if (btn1.wasReleased()) {
    if (btn1_holding) {
      btn1_holding = false;
    } else {
      btn1ShortPressTrigger.trigger();
      Serial.println("Button 1 Short Press");
    }
  }

  if (btn2.pressedFor(500) && !btn2_holding) {
    btn2LongPressTrigger.trigger();
    Serial.println("Button 2 Long Press");
    btn2_holding = true;
  } else if (btn2.wasReleased()) {
    if (btn2_holding) {
      btn2_holding = false;
    } else {
      btn2ShortPressTrigger.trigger();
      Serial.println("Button 2 Short Press");
    }
  }

  if (btn3.pressedFor(500) && !btn3_holding) {
    btn3LongPressTrigger.trigger();
    Serial.println("Button 3 Long Press");
    btn3_holding = true;
  } else if (btn3.wasReleased()) {
    if (btn3_holding) {
      btn3_holding = false;
    } else {
      btn3ShortPressTrigger.trigger();
      Serial.println("Button 3 Short Press");
    }
  }

  if (btn4.pressedFor(500) && !btn4_holding) {
    btn4LongPressTrigger.trigger();
    Serial.println("Button 4 Long Press");
    btn4_holding = true;
  } else if (btn4.wasReleased()) {
    if (btn4_holding) {
      btn4_holding = false;
    } else {
      btn4ShortPressTrigger.trigger();
      Serial.println("Button 4 Short Press");
    }
  }

  if (now - battery_last_sample > battery_sample_rate) {
    battery_last_sample = now;

    for (int i = 0; i < battery_buffer_size; i++) {
      battery_raw = analogRead(BATTERY_SENSING_PIN);
      battery_Filtered = ADC_Average(battery_raw);
    }

    battery_voltage = ADC_Calculate(battery_Filtered);

    double true_voltage = (battery_voltage / 1000.0) * 3.0;
    double battery_percentage = mapf(true_voltage, 6.0, 9.0, 0.0, 100.0);


    voltage.setValue((float)true_voltage);
    percentage.setValue((float)battery_percentage);

    Serial.print("Battery Voltage: "); Serial.print(true_voltage); Serial.println(" V");
    Serial.print("Battery Percentage: "); Serial.print(battery_percentage); Serial.println(" %");
  }
}