#include "SoilSensorClient.h"
#include "config.h"
#include <SensorToolkitMathUtils.h>
#include <ArduinoJson.h>

SoilSensorClient::SoilSensorClient(uint8_t i2cAddress, SensorToolkitMqtt& mqttClient, char *data_topic) : _buffer{SOIL_SENSOR_SAMPLES} {
    _i2cAddress = i2cAddress;
    _mqttClient = &mqttClient;
    _data_topic = data_topic;
}

boolean SoilSensorClient::connect() {
    if (_soilSensor.begin(_i2cAddress)) {
        Serial.print("Connected to seesaw soil sensor with version ");
        Serial.print(_soilSensor.getVersion());
        _isConnected = true;
    }
    else {
        _isConnected = false;
    }
    return _isConnected;
}

boolean SoilSensorClient::sampleAndPublish() {
    if (_isConnected) {
        for (int i=0; i<SOIL_SENSOR_SAMPLES; i++) {
            _buffer.storeValue(_soilSensor.touchRead(0));
            delay(SOIL_SENSOR_SAMPLING_INTERVAL);
        }
        int capacitance = round(_buffer.getAverage());

        // Also get the temperature from the soil probe. This is the remperature of the chip, which is close to (but not in!) the soil.
        double temperature = roundDouble(_soilSensor.getTemp(), 1);
        
        StaticJsonDocument<200> json;
        json["temperature"] = temperature;
        json["capacitance"] = capacitance;

        serializeJson(json, _jsonOutput);        
        _mqttClient->publish(_data_topic, _jsonOutput);
        return true;
    }
    else {
        return false;
    }
}