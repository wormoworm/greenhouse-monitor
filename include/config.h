#ifndef config_H
#define config_H

// Pins.
#define PIN_WAKEUP 0
#define PIN_LED 2

// I2C address of the bme280 sensor.
#define ADDRESS_BME280 0x76

// For static IP configuration on local LAN.
// #define CONFIG_WIFI_IP_ADDRESS_STRING "10.0.1.80"
// #define CONFIG_WIFI_GATEWAY_STRING "10.0.0.1"
// #define CONFIG_WIFI_SUBNET_STRING "255.255.252.0"

IPAddress CONFIG_WIFI_IP_ADDRESS, CONFIG_WIFI_GATEWAY, CONFIG_WIFI_SUBNET;
boolean _1 = CONFIG_WIFI_IP_ADDRESS.fromString("10.0.1.80");
boolean _2 = CONFIG_WIFI_GATEWAY.fromString("10.0.0.1");
boolean _3 = CONFIG_WIFI_SUBNET.fromString("255.255.252.0");

// For MQTT broker connection.
#define CONFIG_MQTT_BROKER_ADDRESS "10.0.1.2"
#define CONFIG_MQTT_BROKER_PORT 8884
#define CONFIG_MQTT_CLIENT_ID "greenhouse"
#define CONFIG_MQTT_KEEP_ALIVE 10

// Normal between data publications to MQTT.
#define SAMPLING_INTERVAL_S 120
// Minimum between data publications to MQTT.
#define MIN_SAMPLING_INTERVAL_S 1
const unsigned long samplingIntervalMicroseconds = SAMPLING_INTERVAL_S * 1000000;
const unsigned long minSamplingIntervalMicroseconds = MIN_SAMPLING_INTERVAL_S * 1000000;

// MQTT topics.
#define TOPIC_BASE "sensors/environment/" CONFIG_MQTT_CLIENT_ID
#define TOPIC_AIR TOPIC_BASE "/air"
#define TOPIC_SOIL_CHANNEL_1 TOPIC_BASE "/soil_channel_1"
#define TOPIC_SOIL_CHANNEL_2 TOPIC_BASE "/soil_channel_2"

#endif