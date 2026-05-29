#ifndef ROBOT_VARS_HPP
#define ROBOT_VARS_HPP

struct SensorPacket { // includes the 9 measurements needed by the Madgwick filter
    float accel_x = 0.0, accel_y = 0.0, accel_z = 0.0;
    float gyro_x = 0.0, gyro_y = 0.0, gyro_z = 0.0;
    float mag_x = 0.0, mag_y = 0.0, mag_z = 0.0;
};

struct AHRSPacket { // again, we only need yaw but is nice and not too expensive to look at all the values
    float roll = 0.0;
    float pitch = 0.0;
    float yaw = 0.0;
};

const int CLICKS_BEFORE_TURN = 20; // tune this
const int CLICKS_AFTER_TURN = 20; // tune this

int clicks = 0; // will be incremented by the handleSwitch task and the interrupt
int clicks_on_straight = 0; // for knowing when to stop at the end of the trench

int heading_state = 0; // 0:initial heading, 1:initial turning, 2:final heading, 3: stop

AHRSPacket ahrs_packet_main; // because we are passing packets by reference we only ever need one instance of the packet
SensorPacket sensor_packet_main; // instead of creating copies the functions just modify the packets where they are

// returns difference between angles, accounting for angle wrapping
float angleDiff(float a, float b) {
    float c = a - b;
    while (c > 180.0) { c += 360.0; }
    while (c < 180.0) { c -= 360.0; }

    return c;
}

#endif