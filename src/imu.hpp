#ifndef IMU_HPP
#define IMU_HPP

#include <Arduino.h>
#include <LSM6.h>

#include "packets_vars_functions.hpp"

const int GYRO_CAL_SAMPLES = 3000; // number of samples to be averaged out, more samples increases accuracy but takes longer
const float GYRO_SCALING = 0.00891089108; // convert betweeen whatever units the gyro is in to degrees per second

// only one instance of this class ever needs to be created because there is only one IMU on the robot
class InertialMeasurementUnit106 { // placing all the methods into a class makes it easier to split code between files
    LSM6 sensor; // this is the core sensor, the STMicroelectronics LSM6DSOX

    // gyroscopes drift because the rates even when resting aren't zero
    float rate_bias_x, rate_bias_y, rate_bias_z; // by subtracting the rate biases we can try to minimize the drift (resting rate becomes closer to zero)

public:
    void initialize() { // initializes the I2C communcation between the ESP32 and the sensor
        if (!sensor.init()) { 
            Serial.println("Failed to detect/initialize LSM6");
            while(1);
        }

        else { Serial.println("Successfully initialized and detected LSM6"); }
        sensor.enableDefault();
    }

    // take samples from the gyro and average them out over a few seconds to calculate the rate bias offsets
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

    // read the sensor values and place them into the given sensor packet
    void read(SensorPacket& sensor_packet) { // because sensor_packet is passed by reference(&) it avoids having to make excessive copies
        sensor.read();

        sensor_packet.gyro_x = (sensor.g.x * GYRO_SCALING) - rate_bias_x;
        sensor_packet.gyro_y = (sensor.g.y * GYRO_SCALING) - rate_bias_y;
        sensor_packet.gyro_z = (sensor.g.z * GYRO_SCALING) - rate_bias_z;

        sensor_packet.accel_x = sensor.a.x; // will add accelerometer calibration later
        sensor_packet.accel_y = sensor.a.y;
        sensor_packet.accel_z = sensor.a.z;
    }
};

#endif