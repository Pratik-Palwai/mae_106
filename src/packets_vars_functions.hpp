#ifndef ROBOT_VARS_HPP
#define ROBOT_VARS_HPP

#include <Arduino.h>

struct SensorPacket { // includes the 9 measurements needed by the Madgwick filter
    float accel_x = 0.0, accel_y = 0.0, accel_z = 0.0;
    float gyro_x = 0.0, gyro_y = 0.0, gyro_z = 0.0;
    float mag_x = 0.0, mag_y = 0.0, mag_z = 0.0;
};

struct AHRSPacket { // again, we only need yaw but is nice and not too expensive to look at all the values
    long timestamp = 0;
    float roll = 0.0; // [deg]
    float pitch = 0.0;
    float yaw = 0.0;
};

const int CLICKS_BEFORE_TURN = 20; // tune this
const int CLICKS_AFTER_TURN = 20; // tune this

int clicks = 0; // will be incremented by the handleSwitch task and the interrupt
int clicks_on_straight = 0; // for knowing when to stop at the end of the trench

int heading_state = 0; // 0:initial heading, 1:initial turning, 2:final heading, 3: stop

double pid_input = 0.0, target_heading = 0.0, pid_output = 0.0; // pid input and output variables, used in actuation.hpp

AHRSPacket ahrs_packet_main; // because we are passing packets by reference we only ever need one instance of the packet
SensorPacket sensor_packet_main; // instead of creating copies the functions just modify the packets where they are

// returns difference between angles, accounting for angle wrapping
float angleDiff(float a, float b) {
    float c = a - b;
    while (c > 180.0) { c -= 360.0; }
    while (c < -180.0) { c += 360.0; }

    return c;
}

void serialOutput(void *param) {
    while(1)
    {
        Serial.print(">timestamp:" + String(ahrs_packet_main.timestamp));
        Serial.print(",accel_x:" + String(sensor_packet_main.accel_x));
        Serial.print(",accel_y:" + String(sensor_packet_main.accel_y));
        Serial.print(",accel_z:" + String(sensor_packet_main.accel_z));
        Serial.print(",gyro_x:" + String(sensor_packet_main.gyro_x));
        Serial.print(",gyro_y:" + String(sensor_packet_main.gyro_y));
        Serial.print(",gyro_z:" + String(sensor_packet_main.gyro_z));
        Serial.print(",mag_x:" + String(sensor_packet_main.mag_x));
        Serial.print(",mag_y:" + String(sensor_packet_main.mag_y));
        Serial.print(",mag_z:" + String(sensor_packet_main.mag_z));
        Serial.print(",roll:" + String(ahrs_packet_main.roll));
        Serial.print(",pitch:" + String(ahrs_packet_main.pitch));
        Serial.print(",yaw:" + String(ahrs_packet_main.yaw));
        Serial.print(",clicks:" + String(clicks));
        Serial.print(",heading_state:" + String(heading_state));
        Serial.print("\r\n");

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

#endif