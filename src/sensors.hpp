#include <Arduino.h>
#include <MadgwickAHRS.h>

#include "imu.hpp"
#include "magnetometer.hpp"
#include "robot_vars.hpp"

Madgwick filter_main;

InertialMeasurementUnit106 imu_main;
Magnetometer106 compass_main;

void readAllSensors(void *param) {
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(2);

    while (1) {
        imu_main.read(sensor_packet_main);
        compass_main.read(sensor_packet_main);

        xTaskDelayUntil(&last_wake, period);
    }
}

void updateAHRS(void *param) {
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(2); // AHRS is updated at 0.5 kHz, preventing sensor packets (updated at 1 kHz) from being used more than once

    while(1) {
        filter_main.update(sensor_packet_main.gyro_x, sensor_packet_main.gyro_y, sensor_packet_main.gyro_z,
                           sensor_packet_main.accel_x, sensor_packet_main.accel_y, sensor_packet_main.accel_z,
                           sensor_packet_main.mag_x, sensor_packet_main.mag_y, sensor_packet_main.mag_z);
        
        ahrs_packet_main.roll = filter_main.getRoll();
        ahrs_packet_main.pitch = filter_main.getPitch();
        ahrs_packet_main.yaw = filter_main.getYaw();

        xTaskDelayUntil(&last_wake, period);
    }
}