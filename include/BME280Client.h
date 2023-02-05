#ifndef BME280Client_H
#define BME280Client_H

#include <SmoothingBuffer.h>
#include <SensorToolkitMqtt.h>
#include <Adafruit_BME280.h>

class BME280Client {

    public:
    
        BME280Client(uint8_t i2cAddress, SensorToolkitMqtt& mqttClient, char *data_topic);

        boolean connect();

        boolean sampleAndPublish();

    private:

        Adafruit_BME280 _bme280;
        uint8_t _i2cAddress;
        SmoothingBuffer _buffer;
        SensorToolkitMqtt *_mqttClient;
        char *_data_topic;
        boolean _isConnected;
        char _jsonOutput[200];
};

#endif