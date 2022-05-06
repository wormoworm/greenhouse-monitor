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
#include <Adafruit_seesaw.h>

WiFiClient espClient;
SensorToolkitWifi *wifiClient;
SensorToolkitMqtt mqttClient = SensorToolkitMqtt(espClient, CONFIG_MQTT_BROKER_ADDRESS, CONFIG_MQTT_BROKER_PORT, CONFIG_MQTT_CLIENT_ID);

Adafruit_BME280 bme280;
Adafruit_seesaw seesawCh1, seesawCh2;
char jsonOutput[200];

bool bme280Connected = false;
bool seesawChannel1Connected = false;
bool seesawChannel2Connected = false;

// Forward function declarations
 void setupWifi();
 void connectToMqtt();
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
    // wifiClient->setIpConfig(CONFIG_WIFI_IP_ADDRESS, CONFIG_WIFI_GATEWAY, CONFIG_WIFI_SUBNET);
    if (wifiClient->connectToWifi(WIFI_SSID, WIFI_PASSWORD, true)) {
        // mqttClient.setCallback(mqttSubscriptionCallback);
        mqttClient.connect(MQTT_USERNAME, MQTT_PASSWORD, CONFIG_MQTT_KEEP_ALIVE);
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

void initialiseSensors() {
    if (bme280.begin(0x76)) {
        Serial.print("Connected to BME280 sensor with ID 0x");
        Serial.println(bme280.sensorID(), 16);
        bme280Connected = true;
    }
    else {
        Serial.println("Could not connect to BME280 sensor, check wiring, address, sensor ID!");
        // while (1) delay(10);
    }
    if (seesawCh1.begin(0x36)) {
        Serial.print("Connected to seesaw soil sensor with version ");
        Serial.print(seesawCh1.getVersion());
        seesawChannel1Connected = true;
    }
    else {
        Serial.println("Could not connect to seesaw channel 1, check wiring, address, sensor ID!");
        // while (1) delay(10);
    }
}

void readSoilSensorAndPublish(Adafruit_seesaw soilSensor, const char *topic) {
    double temperature = roundDouble(soilSensor.getTemp(), 1);
    uint16_t capacitance = soilSensor.touchRead(0);
    
    StaticJsonDocument<200> json;
    json["temperature"] = temperature;
    json["capacitance"] = capacitance;

    serializeJson(json, jsonOutput);        
    mqttClient.publish(topic, jsonOutput);
    
    serializeJsonPretty(json, Serial);
}

void readSensorsAndPublish() {
    if (bme280Connected){
        double temperature = roundDouble(bme280.readTemperature() - 1, 1);
        double humidity = roundDouble(bme280.readHumidity(), 1);
        double pressure = roundDouble(bme280.readPressure() / 1000.0, 3);

        StaticJsonDocument<200> json;
        json["temperature"] = temperature;
        json["humidity"] = humidity;
        json["pressure"] = pressure;

        serializeJson(json, jsonOutput);        
        mqttClient.publish(TOPIC_AIR, jsonOutput);
        
        serializeJsonPretty(json, Serial);
    }

    if (seesawChannel1Connected){
        readSoilSensorAndPublish(seesawCh1, TOPIC_SOIL_CHANNEL_1);
    }

    if (seesawChannel2Connected){
        readSoilSensorAndPublish(seesawCh2, TOPIC_SOIL_CHANNEL_2);
    }
}