#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#define ARDUINOJSON_USE_DOUBLE 1
#include <ArduinoJson.h>
#include <Arduino.h>
#include <math.h>
#include <SensorToolkitWifi.h>
#include <SensorToolkitMqtt.h>
#include "config.h"
#include <SensorToolkitMathUtils.h>
#include <Adafruit_seesaw.h>
#include "SoilSensorClient.h"
#include "BME280Client.h"

WiFiClient espClient;
SensorToolkitWifi *wifiClient;
SensorToolkitMqtt mqttClient = SensorToolkitMqtt(espClient, CONFIG_MQTT_BROKER_ADDRESS, CONFIG_MQTT_BROKER_PORT, CONFIG_MQTT_CLIENT_ID);

BME280Client bme280 = BME280Client(ADDRESS_BME280, mqttClient, TOPIC_AIR);
SoilSensorClient soilChannel1 = SoilSensorClient(ADDRESS_SOIL_SENSOR_1, mqttClient, TOPIC_SOIL_CHANNEL_1);
// SoilSensorClient soilChannel2 = SoilSensorClient(ADDRESS_SOIL_SENSOR_2, mqttClient, TOPIC_SOIL_CHANNEL_2);

// Forward function declarations
 void setupWifi();
 void connectToMqtt();
 void logBuildInfo();
 void initialiseSensors();
 void readSensorsAndPublish();
 void callback(char* topic, byte* payload, unsigned int length);

void setup() {
    unsigned long startTimestampMs = millis();
    
    Serial.begin(115200);
    Serial.setTimeout(2000);
    
    // initialize GPIO 2 as an output.
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);   // turn the LED on

    // Wakeup pin for deep sleep
    pinMode(PIN_WAKEUP, WAKEUP_PULLUP);

  // Setup Wi-Fi.
    wifiClient = new SensorToolkitWifi();
    // wifiClient->setConnectionTickCallback(wifiConnectionTickCallback);
    if (wifiClient->connectToWifi(WIFI_SSID, WIFI_PASSWORD, true)) {
        // mqttClient.setCallback(mqttSubscriptionCallback);
        mqttClient.connect(MQTT_USERNAME, MQTT_PASSWORD, CONFIG_MQTT_KEEP_ALIVE);
        logBuildInfo();
        initialiseSensors();
        readSensorsAndPublish();
    }

  // Delay a small amount of time to allow the MQTT client time to publish the message
  delay(100);
  digitalWrite(PIN_LED, LOW);   // turn the LED off
  
  unsigned long stopTimestampMs = millis();
  unsigned long sampleDurationMs = stopTimestampMs - startTimestampMs;
  unsigned long sampleDurationMicroseconds = sampleDurationMs * 1000;
  
  unsigned long timeToNextSleepMicroseconds;
  if (sampleDurationMicroseconds < samplingIntervalMicroseconds) {
    timeToNextSleepMicroseconds = samplingIntervalMicroseconds - sampleDurationMicroseconds;
  }
  else {
    timeToNextSleepMicroseconds = minSamplingIntervalMicroseconds;
  }
  ESP.deepSleep(timeToNextSleepMicroseconds);
}

void loop() {
    // Nothing to see here - the code goes into deep sleep at the end of setup().
}

void logBuildInfo() {
  StaticJsonDocument<200> json;

  json["project_name"] = PROJECT_NAME;
  json["project_url"] = PROJECT_URL;

  char jsonOutput[200];
  serializeJson(json, jsonOutput);        
  mqttClient.publish(TOPIC_LOG, jsonOutput);
}

void initialiseSensors() {
    if (!bme280.connect()) {
        Serial.println("Could not connect to BME280 sensor, check wiring, address, sensor ID!");
    }
    if (!soilChannel1.connect()) {
        Serial.println("Could not connect to soil sensor channel 1, check wiring, address, sensor ID!");
    }
    // if (!soilChannel2.connect()) {
    //     Serial.println("Could not connect to soil sensor channel 1, check wiring, address, sensor ID!");
    // }
}

void readSensorsAndPublish() {
    bme280.sampleAndPublish();

    soilChannel1.sampleAndPublish();

    // soilChannel2.sampleAndPublish();
}