#ifndef MAGNETOMETER_HPP
#define MAGNETOMETER_HPP

#include <Arduino.h>
#include <EEPROM.h>
#include <LIS3MDL.h>

#include "robot_vars.hpp"

#define EEPROM_MAG_X_OFFSET_ADDRESS 0
#define EEPROM_MAG_X_SCALING_ADDRESS 4
#define EEPROM_MAG_Y_OFFSET_ADDRESS 8
#define EEPROM_MAG_Y_SCALING_ADDRESS 12
#define EEPROM_MAG_Z_OFFSET_ADDRESS 16
#define EEPROM_MAG_Z_SCALING_ADDRESS 20

const int MAG_CAL_SAMPLES = 3000;

class Magnetometer106 {
    LIS3MDL sensor;

    float x_offset = 0.0, x_scaling = 1.0;
    float y_offset = 0.0, y_scaling = 1.0;
    float z_offset = 0.0, z_scaling = 1.0;

    bool manual_calibration = false;

    void calibrateToEEPROM() {
        float x_min = 2147483646, x_max = -2147483646;
        float y_min = 2147483646, y_max = - 2147483646;
        float z_min = 2147483646, z_max = - 2147483646;

        Serial.print("Starting magnetometer calibration loop ... ");

        unsigned long start_time = millis();

        for (int i = 0; i < MAG_CAL_SAMPLES; ++i) {
            sensor.read();
        
            if (sensor.m.x < x_min) { x_min = sensor.m.x; }
            if (sensor.m.x > x_max) { x_max = sensor.m.x; }
            if (sensor.m.y < y_min) { y_min = sensor.m.y; }
            if (sensor.m.y > y_max) { y_max = sensor.m.y; }
            if (sensor.m.z > z_min) { z_min = sensor.m.z; }
            if (sensor.m.z > z_max) { z_max = sensor.m.z; }

            delay(1);
        }

        x_offset = (x_max + x_min) / 2.0;
        y_offset = (y_max + y_min) / 2.0;
        z_offset = (z_max + z_min) / 2.0;

        float radius_x = (x_max - x_min) / 2.0;
        float radius_y = (y_max - y_min) / 2.0;
        float radius_z = (z_max - z_min) / 2.0;
        float radius_avg = (radius_x + radius_y + radius_z) / 3.0;

        x_scaling = radius_avg / radius_x;
        y_scaling = radius_avg / radius_y;
        z_scaling = radius_avg / radius_z;

        Serial.println("finished");
        
        Serial.println("    mag_x_offset:" + String(x_offset) + " | mag_x_scaling:" + String(x_scaling));
        Serial.println("    mag_y_offset:" + String(y_offset) + " | mag_y_scaling:" + String(y_scaling));
        Serial.println("    mag_z_offset:" + String(z_offset) + " | mag_z_scaling:" + String(z_scaling));

        EEPROM.put(EEPROM_MAG_X_OFFSET_ADDRESS, x_offset);
        EEPROM.put(EEPROM_MAG_X_SCALING_ADDRESS, x_scaling);
        EEPROM.put(EEPROM_MAG_Y_OFFSET_ADDRESS, y_offset);
        EEPROM.put(EEPROM_MAG_Y_SCALING_ADDRESS, y_scaling);
        EEPROM.put(EEPROM_MAG_Z_OFFSET_ADDRESS, z_offset);
        EEPROM.put(EEPROM_MAG_Z_SCALING_ADDRESS, z_scaling);

        EEPROM.commit();
        Serial.println("    saved calibrations to EEPROM");
    }

    void calibrateFromEEPROM() {
        Serial.println("Pulling compass calibrations from EEPROM ...");

        EEPROM.get(EEPROM_MAG_X_OFFSET_ADDRESS, x_offset);
        EEPROM.get(EEPROM_MAG_X_SCALING_ADDRESS, x_scaling);
        EEPROM.get(EEPROM_MAG_Y_OFFSET_ADDRESS, y_offset);
        EEPROM.get(EEPROM_MAG_Y_SCALING_ADDRESS, y_scaling);
        EEPROM.get(EEPROM_MAG_Z_OFFSET_ADDRESS, z_offset);
        EEPROM.get(EEPROM_MAG_Z_SCALING_ADDRESS, z_scaling);

        Serial.println("    finished");

        Serial.println("    mag_x_offset:" + String(x_offset) + " | mag_x_scaling:" + String(x_scaling));
        Serial.println("    mag_y_offset:" + String(y_offset) + " | mag_y_scaling:" + String(y_scaling));
        Serial.println("    mag_z_offset:" + String(z_offset) + " | mag_z_scaling:" + String(z_scaling));
    }

public:
    void initialize() {
        if (!sensor.init()) {
            Serial.println("Failed to detect/initialize LIS3MDL");
            while(1);
        }

        else { Serial.println("Successfully detected and initialized LIS3MDL"); }
        sensor.enableDefault();
    }

    void calibrate() {
        if (manual_calibration) { calibrateToEEPROM(); }
        else { calibrateFromEEPROM(); }
    }

    void read(SensorPacket& sensor_packet) {
        sensor.read();

        sensor_packet.mag_x = (sensor.m.x - x_offset) * x_scaling;
        sensor_packet.mag_y = (sensor.m.y - y_offset) * y_scaling;
        sensor_packet.mag_z = (sensor.m.z - z_offset) * z_scaling;
    }
};

#endif