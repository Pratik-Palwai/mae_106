#ifndef IMU_HPP
#define IMU_HPP

#include <Arduino.h>
#include <LSM6.h>

#include "robot_vars.hpp"

const int GYRO_CAL_SAMPLES = 3000;
const float GYRO_SCALING = 0.00891089108;

class InertialMeasurementUnit106 {
    LSM6 sensor;

    float rate_bias_x, rate_bias_y, rate_bias_z;

public:
    void initialize() {
        if (!sensor.init()) { 
            Serial.println("Failed to detect/initialize LSM6");
            while(1);
        }

        else { Serial.println("Successfully initialized and dected LSM6"); }
        sensor.enableDefault();
    }

    void calibrate() {
        double rates_sum_x = 0.0, rates_sum_y = 0.0, rates_sum_z = 0.0;

        Serial.print("Starting gyro calibration loop ... ");

        for (int i = 0; i < GYRO_CAL_SAMPLES; ++i) {
            delay(1); sensor.read();

            rates_sum_x += sensor.g.x * GYRO_SCALING;
            rates_sum_y += sensor.g.y * GYRO_SCALING;
            rates_sum_z += sensor.g.z * GYRO_SCALING;
        }

        Serial.println("finished");

        rate_bias_x = rates_sum_x / GYRO_CAL_SAMPLES;
        rate_bias_y = rates_sum_y / GYRO_CAL_SAMPLES;
        rate_bias_z = rates_sum_z / GYRO_CAL_SAMPLES;

        Serial.println("    gyro_rate_bias_x: " + String(rate_bias_x));
        Serial.println("    gyro_rate_bias_y: " + String(rate_bias_y));
        Serial.println("    gyro_rate_bias_z: " + String(rate_bias_z));
    }

    void read(SensorPacket& sensor_packet) {
        sensor.read();

        sensor_packet.gyro_x = (sensor.g.x * GYRO_SCALING) - rate_bias_x;
        sensor_packet.gyro_x = (sensor.g.y * GYRO_SCALING) - rate_bias_y;
        sensor_packet.gyro_x = (sensor.g.z * GYRO_SCALING) - rate_bias_z;

        sensor_packet.accel_x = sensor.a.x;
        sensor_packet.accel_y = sensor.a.y;
        sensor_packet.accel_z = sensor.a.z;
    }
};

#endif