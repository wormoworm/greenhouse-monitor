#ifndef config_H
#define config_H

// Pins.
#define PIN_WAKEUP 0
#define PIN_LED 2

// I2C sensor addresses.
#define ADDRESS_BME280 0x76
#define ADDRESS_SOIL_SENSOR_1 0x36
// #define ADDRESS_SOIL_SENSOR_1 0x37

// Smoothing config for soil sensors.
#define SOIL_SENSOR_SAMPLES 20
#define SOIL_SENSOR_SAMPLING_INTERVAL 50

// For MQTT broker connection.
#define CONFIG_MQTT_BROKER_ADDRESS "10.0.1.2"
#define CONFIG_MQTT_BROKER_PORT 8884
#define CONFIG_MQTT_CLIENT_ID "garage"
#define CONFIG_MQTT_KEEP_ALIVE 10

// Normal between data publications to MQTT.
#define SAMPLING_INTERVAL_S 60
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