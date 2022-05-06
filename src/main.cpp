#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_BME280.h>
#define ARDUINOJSON_USE_DOUBLE 1
#include <ArduinoJson.h>
#include <Arduino.h>
#include <math.h>
#include <SensorToolkitWifi.h>
#include <SensorToolkitMqtt.h>
#include "config.h"
#include <SensorToolkitMathUtils.h>

WiFiClient espClient;
SensorToolkitWifi *wifiClient;
SensorToolkitMqtt mqttClient = SensorToolkitMqtt(espClient, CONFIG_MQTT_BROKER_ADDRESS, CONFIG_MQTT_BROKER_PORT, CONFIG_MQTT_CLIENT_ID);

Adafruit_BME280 bme;
char jsonOutput[200];

// Forward function declarations
 void setupWifi();
 void connectToMqtt();
 void initialiseSensor();
 void readSensorAndPublish();
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
    // wifiClient->setIpConfig(CONFIG_WIFI_IP_ADDRESS, CONFIG_WIFI_GATEWAY, CONFIG_WIFI_SUBNET);
    if (wifiClient->connectToWifi(WIFI_SSID, WIFI_PASSWORD, true)) {
        // mqttClient.setCallback(mqttSubscriptionCallback);
        mqttClient.connect(MQTT_USERNAME, MQTT_PASSWORD, CONFIG_MQTT_KEEP_ALIVE);
        initialiseSensor();
        readSensorAndPublish();
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

//   Serial.println();
//   Serial.print("Base sampling interval: ");
//   Serial.println(samplingIntervalMicroseconds);
  
//   Serial.println();
//   Serial.print("Sample duration micros: ");
//   Serial.println(sampleDurationMs * 1000);
  
//   Serial.println();
//   Serial.print("Time to next sleep micros: ");
//   Serial.println(timeToNextSleepMicroseconds);
  
  ESP.deepSleep(timeToNextSleepMicroseconds);
}

// the loop function runs over and over again forever
void loop() {
    // Nothing to see here - the code goes into deep sleep at the end of setup().
}

void initialiseSensor() {
  // default settings
    unsigned status = bme.begin(0x76);
  if (!status) {
      Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
      Serial.print("SensorID is: 0x"); Serial.println(bme.sensorID(),16);
      while (1) delay(10);
  }
}

void readSensorAndPublish() {
    StaticJsonDocument<200> json;
    double temperature = roundDouble(bme.readTemperature() - 1, 1);
    double humidity = roundDouble(bme.readHumidity(), 1);
    double pressure = roundDouble(bme.readPressure() / 1000.0, 3);
    Serial.println("-----");
    Serial.print("Pressure: ");
    Serial.println(pressure);
    Serial.println("-----");
    json["temperature"] = temperature;
    json["humidity"] = humidity;
    json["pressure"] = pressure;

    serializeJsonPretty(json, Serial);
    serializeJson(json, jsonOutput);
    
    mqttClient.publish(TOPIC_DATA, jsonOutput);
}