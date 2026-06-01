#ifndef SENSORS_HPP
#define SENSORS_HPP

#include <Arduino.h>
#include <MadgwickAHRS.h>

#include "imu.hpp"
#include "magnetometer.hpp"
#include "packets_vars_functions.hpp"

Madgwick filter_main; // creates an instance of the Madgwick filter, which is a common sensor fusion algorithm for microcontrollers
                      // it uses the accelerometer, gyroscope, and magnetometer to create a full 3-axis AHRS, and for 106 we only need to use the yaw

InertialMeasurementUnit106 imu_main; // creates an instance of the IMU, defined in "imu.hpp"
Magnetometer106 compass_main; // creates an instance of the compass, defined in "magnetometer.hpp"

// rtos task to read the sensors, executed at 1kHz which is the fastest FreeRTOS allows
void readAllSensors(void *param) {
    TickType_t last_wake = xTaskGetTickCount(); // FreeRTOS ticks are how everything is timed, on the ESP32C3 the tick rate is 1000 ticks/sec
    const TickType_t period = pdMS_TO_TICKS(1); // the period is 1 ms (1 tick)

    while (1) {
        imu_main.read(sensor_packet_main); // sensor_packet_main is passed by reference (indicated by the & in imu.hpp), meaning the function can modify the passed parameter in place
        compass_main.read(sensor_packet_main); // this avoids having to make excessive copies of each sensor packet

        sensor_packet_main.timestamp = static_cast<long>(millis());
        xTaskDelayUntil(&last_wake, period); // xTaskDelayUntil() is used instead of vTaskDelay() for a fixed task frequency
    }
}

// rtos task to update Madgwick AHRS, executed at 0.5 kHz to prevent sensor packets from being used more than once
void updateAHRS(void *param) {
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(2);

    while(1) {
        // this simple filter_main.update() method is what makes the library and filter so useful
        // instead of having to mess around with angle wrapping, the magnetometer low-pass filter, or gyro drift the filter takes care of all this
        filter_main.update(sensor_packet_main.gyro_x, sensor_packet_main.gyro_y, sensor_packet_main.gyro_z,
                           sensor_packet_main.accel_x, sensor_packet_main.accel_y, sensor_packet_main.accel_z,
                           sensor_packet_main.mag_x, sensor_packet_main.mag_y, sensor_packet_main.mag_z);
        
        
        // get AHRS variables (roll, pitch, and yaw) and push them to ahrs_packet_main
        // for 106 we only really need yaw because the robot is flat, but it is nice and not much work to get all three axes
        ahrs_packet_main.roll = filter_main.getPitch() - roll_trim;
        ahrs_packet_main.pitch = filter_main.getRoll() - pitch_trim;
        ahrs_packet_main.yaw = filter_main.getYaw() - yaw_trim;

        ahrs_packet_main.timestamp = static_cast<long>(millis());
        xTaskDelayUntil(&last_wake, period); // again, xTaskDelayUntil() gives better accuracy than a simple vTaskDelay()
    }
}

#endif