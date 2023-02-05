#include "BME280Client.h"
#include "config.h"
#include <SensorToolkitMathUtils.h>
#include <ArduinoJson.h>

#define TEMPERATURE_VALIDATION_MINIMUM -50
#define TEMPERATURE_VALIDATION_MAXIMUM 150
#define HUMIDITY_VALIDATION_MINIMUM 0
#define HUMIDITY_VALIDATION_MAXIMUM 100
#define PRESSURE_VALIDATION_MINIMUM 50
#define PRESSURE_VALIDATION_MAXIMUM 150

BME280Client::BME280Client(uint8_t i2cAddress, SensorToolkitMqtt& mqttClient, char *data_topic) : _buffer{SOIL_SENSOR_SAMPLES} {
    _i2cAddress = i2cAddress;
    _mqttClient = &mqttClient;
    _data_topic = data_topic;
}

boolean BME280Client::connect() {
    if (_bme280.begin(_i2cAddress)) {
        Serial.print("Connected to BME280 sensor with ID 0x");
        Serial.println(_bme280.sensorID(), 16);
        _isConnected = true;
    }
    else {
        _isConnected = false;
    }
    return _isConnected;
}

boolean validate_double(double value, double min, double max) {
    return value >= min && value <= max;
}

boolean is_temperature_valid(double temperature) {
    return validate_double(temperature, TEMPERATURE_VALIDATION_MINIMUM, TEMPERATURE_VALIDATION_MAXIMUM);
}

boolean is_humidity_valid(double humidity) {
    return validate_double(humidity, HUMIDITY_VALIDATION_MINIMUM, HUMIDITY_VALIDATION_MAXIMUM);
}

boolean is_pressure_valid(double pressure) {
    return validate_double(pressure, PRESSURE_VALIDATION_MINIMUM, PRESSURE_VALIDATION_MAXIMUM);
}

boolean BME280Client::sampleAndPublish() {
    if (_isConnected) {
        double temperature = roundDouble(_bme280.readTemperature() - 1, 1);
        double humidity = roundDouble(_bme280.readHumidity(), 1);
        double pressure = roundDouble(_bme280.readPressure() / 1000.0, 3);

        StaticJsonDocument<200> json;
        if (is_temperature_valid(temperature)) {
            json["temperature"] = temperature;
        }
        if (is_humidity_valid(humidity)) {
            json["humidity"] = humidity;
        }
        if (is_pressure_valid(pressure)) {
            json["pressure"] = pressure;
        }

        serializeJson(json, _jsonOutput);        
        _mqttClient->publish(_data_topic, _jsonOutput);
        return true;
    }
    else {
        return false;
    }
}