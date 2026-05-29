#ifndef ROBOT_VARS_HPP
#define ROBOT_VARS_HPP

struct SensorPacket {
    float accel_x = 0.0, accel_y = 0.0, accel_z = 0.0;
    float gyro_x = 0.0, gyro_y = 0.0, gyro_z = 0.0;
    float mag_x = 0.0, mag_y = 0.0, mag_z = 0.0;
};

struct AHRSPacket {
    float roll = 0.0;
    float pitch = 0.0;
    float yaw = 0.0;
};

int clicks = 0;
int heading_state = 0;

AHRSPacket ahrs_packet_main;
SensorPacket sensor_packet_main;

#endif