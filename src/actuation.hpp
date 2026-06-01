#ifndef ACTUATION_HPP
#define ACTUATION_HPP

#include <Arduino.h>
#include <PID_v1.h>
#include <Servo.h>

#include "packets_vars_functions.hpp"

const int SOLENOID_PIN = D7;
const int OPEN_TIME = 750, CLOSE_TIME = 250;

volatile bool solenoid_state = false;
volatile bool actuation_allowed = true;

Servo steering_servo;
const int SERVO_PIN = D9;

double pid_input = 0.0, target_heading = 0.0, pid_output = 0.0;
const float K_P = 0.50, K_I = 0.0, K_D = 0.0;
PID steering_correction(&pid_input, &pid_output, &target_heading, K_P, K_I, K_D, REVERSE);  // pid mode can be DIRECT or REVERSE depending on how the servo and magnetometer are mounted

void steerRobot(void *param) {
    float initial_heading = 180.0;
    float final_heading = 270.0;

    while(1) {
        pid_input = ahrs_packet_main.yaw;

        if (heading_state == 0) { target_heading = initial_heading; }
        else { target_heading = final_heading; }
        if ((heading_state == 0) && (clicks_on_straight > CLICKS_BEFORE_TURN)) { heading_state = 1; }
        else if (heading_state == 1) {
            float angle_error = angleDiff(pid_input, target_heading);
            if (abs(angle_error) < 5.0) { heading_state = 2; } // transition from turning state to corridor state
                                                               // even though the heading state changes the target remains the same
            
            clicks_on_straight = 0; // don't increment this value while turning
        }

        // transition between corridor state to stopped state
        else if ((heading_state == 2) && (clicks_on_straight > CLICKS_AFTER_TURN)) { heading_state = 3; }

        steering_correction.Compute();
        steering_servo.write(90.0 + pid_output); // convert the +/- heading from the PID to a 0-180 command for the servo

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void firePiston(void *param) {
    while (1) {
        if (millis() >= 72000) { actuation_allowed = false; } // after 75 sec (60 sec competition + 15 sec setup) the piston will shut off
        if (heading_state == 3) { actuation_allowed = false; } // if the robot has gone down the corridor the piston will shut off

        if (actuation_allowed) {
            digitalWrite(SOLENOID_PIN, solenoid_state);

            if (solenoid_state) { vTaskDelay(pdMS_TO_TICKS(OPEN_TIME)); }
            else { vTaskDelay(pdMS_TO_TICKS(CLOSE_TIME)); }

            solenoid_state = !solenoid_state;
        }

        else {
            digitalWrite(SOLENOID_PIN, LOW); // turn the solenoid off if the pin is left HIGH from the actuation_allowed cutoff
            vTaskDelay(50);
        }
    }
}

#endif