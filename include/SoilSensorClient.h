#ifndef SoilSensorClient_H
#define SoilSensorClient_H

#include <SmoothingBuffer.h>
#include <SensorToolkitMqtt.h>
#include <Adafruit_seesaw.h>

class SoilSensorClient {

    public:
    
        SoilSensorClient(uint8_t i2cAddress, SensorToolkitMqtt& mqttClient, char *data_topic);

        boolean connect();

        boolean sampleAndPublish();

    private:

        Adafruit_seesaw _soilSensor;
        uint8_t _i2cAddress;
        SmoothingBuffer _buffer;
        SensorToolkitMqtt *_mqttClient;
        char *_data_topic;
        boolean _isConnected;
        char _jsonOutput[200];
};

#endif