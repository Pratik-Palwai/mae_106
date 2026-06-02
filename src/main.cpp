#include <Arduino.h>
#include <MadgwickAHRS.h>

#include "sensors.hpp"
#include "actuation.hpp"
#include "imu.hpp"
#include "magnetometer.hpp"
#include "packets_vars_functions.hpp"
#include "switch.hpp"
#include "telemetry.hpp"

void setup() {
    Serial.begin(115200);
    delay(2000);

    Wire.begin(); // default I2C pins on ESP32C3: SDA GPIO8 and SCL GPIO9
    EEPROM.begin(24); // save 24 bytes: 3 axes * 2 values (scaling + offset) * 4 bytes per float

    telemetry_server_main.initialize();

    imu_main.initialize();
    compass_main.initialize();

    imu_main.calibrate();
    compass_main.calibrate();

    filter_main.begin(500); // must match updateAHRS frequency defined in sensors.hpp

    pinMode(SOLENOID_PIN, OUTPUT);
    pinMode(LIMIT_SWITCH_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_PIN), limitSwitchISR, RISING);

    steering_servo.attach(SERVO_PIN);
    steering_correction.SetOutputLimits(-40, 40);
    steering_correction.SetMode(AUTOMATIC);

    xTaskCreate(readAllSensors, "SENSE", 4096, NULL, 7, NULL);
    xTaskCreate(updateAHRS, "AHRS", 4096, NULL, 6, NULL);
    xTaskCreate(handleSwitch, "SWITCH", 4096, NULL, 5, NULL);
    xTaskCreate(steerRobot, "STEER", 4096, NULL, 4, NULL);
    xTaskCreate(firePiston, "FIRE", 4096, NULL, 3, NULL);
    xTaskCreate(wifiTelemetry, "WIFI", 4096, NULL, 2, NULL);
    xTaskCreate(serialOutput, "SERIAL", 4096, NULL, 1, NULL);
}

void loop() { } // nothing needs to be in loop() because FreeRTOS handles all