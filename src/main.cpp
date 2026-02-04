#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <LSM6.h>
#include <LIS3MDL.h>
#include <Servo.h>
#include <PID_v1.h>

LSM6 lsm6;
const float GYRO_CALIBRATION_SAMPLES = 500;
const float LSM6_GYRO_NORMALIZATION = 0.00875;
const float LIS3_MAG_NORMALIZATION = RAD_TO_DEG;

LIS3MDL lis3;
#define EEPROM_X_OFFSET 0
#define EEPROM_Y_OFFSET 4

float mag_x_offset = 0;
float mag_y_offset = 0;
int calibration_start_time = 0;

QueueHandle_t imu_queue;
QueueHandle_t ahrs_queue;
int ahrs_dt = 0;

const int LIMIT_SWITCH_PIN = D1;
volatile int clicks = 0;
volatile bool trigger = false;
const int DEBOUNCE_TIME = 75;

Servo steering_servo;
const int SERVO_PIN = D0;

enum motionState {STRAIGHT_1, TURNING, STRAIGHT_2};
motionState state = STRAIGHT_1;

double steering_angle = 0.0;
double setpoint = 0.0, orientation_z = 0.0, steering_angle_delta = 0.0;
const float K_P = 1.0, K_I = 0.0, K_D = 0.0;
PID steering_correction(&orientation_z, &steering_angle_delta, &setpoint, K_P, K_I, K_D, DIRECT);

bool solenoid_state = false;
const int OPEN_TIME = 250, CLOSE_TIME = 750;
const int SOLENOID_PIN = D7;

struct sensorPacket
{
    long timestamp = 0;
    float lsm6_gyro[3] = {0.0, 0.0, 0.0};
    float lis3_mag[3] = {0.0, 0.0, 0.0};
};

struct ahrsPacket
{
    long timestamp = 0;
    float gyro_angle = 0.0;;
    float gyro_rate = 0.0;
    float mag_fields[3] = {0.0, 0.0, 0.0};
    float mag_heading = 0.0;
    float fused_heading = 0.0;
};

void IRAM_ATTR limitSwitchISR() { trigger = true; }

void initializeLSM6(void)
{
    if (!lsm6.init())
    {
        Serial.println("Failed to detect/initialize LSM6");
        while(1);
    }

    else { Serial.println("Sucessfully initialized LSM6"); }

    lsm6.enableDefault();
}

void calibrateMagnetometer() {
    Serial.println("Calibrating... rotate sensor slowly in all directions for 10 sec");

    float x_min = 32767, x_max = -32768;
    float y_min = 32767, y_max = -32768;

    unsigned long t0 = millis();
    while (millis() - t0 < 10000) 
    {
        lis3.read();

        if (lis3.m.x < x_min) x_min = lis3.m.x;
        if (lis3.m.x > x_max) x_max = lis3.m.x;
        if (lis3.m.y < y_min) y_min = lis3.m.y;
        if (lis3.m.y > y_max) y_max = lis3.m.y;

        delay(10);
    }

    mag_x_offset = (x_max + x_min) / 2.0;
    mag_y_offset = (y_max + y_min) / 2.0;

    EEPROM.put(EEPROM_X_OFFSET, mag_x_offset);
    EEPROM.put(EEPROM_Y_OFFSET, mag_y_offset);
}

void initalizeLIS3(void)
{
    if (!lis3.init())
    {
        Serial.println("Failed to detect/initialize LIS3");
        while(1);
    }

    else
    {
        Serial.println("Successfully initialized LIS3");
    }

    Serial.println("Press 'R' within 5 seconds to recalibrate.");
    unsigned long t0 = millis();
    bool recalibrate = false;

    while (millis() - t0 < 5000)
    {
        if (Serial.available())
        {
            char c = Serial.read();
            if (c == 'R' || c == 'r') { recalibrate = true; break; }
        }
    }

    if (recalibrate)
    {
        calibrateMagnetometer();
        Serial.println("Calibration complete and saved to EEPROM.");
    }

    else
    {
        EEPROM.get(EEPROM_X_OFFSET, mag_x_offset);
        EEPROM.get(EEPROM_Y_OFFSET, mag_y_offset);
        Serial.println("Loaded calibration from EEPROM.");
    }

    Serial.println("X,Y,Z,Heading(deg)");

    calibration_start_time = millis();

    while(1)
    {
        unsigned long elapsed = millis() - calibration_start_time;
        if (elapsed >= 10000)
        {
            Serial.println("Measurement complete.");
            break;
        }

        lis3.read();

        float x_corr = lis3.m.x - mag_x_offset;
        float y_corr = lis3.m.y - mag_y_offset;

        float heading = atan2(y_corr, x_corr) * 180.0;
        if (heading < 0) heading += 360.0;

        Serial.print(lis3.m.x); Serial.print(" ");
        Serial.print(lis3.m.y); Serial.print(" ");
        Serial.print(lis3.m.z); Serial.print(" ");
        Serial.println(heading, 2);

        delay(10);
    }
}

void readSensors(void *param)
{
    sensorPacket sensor_packet;

    float gyro_bias = 0.0;

    Serial.println("Calibrating gyro");

    for (int i = 0; i < GYRO_CALIBRATION_SAMPLES; i++)
    {
        lsm6.read();

        gyro_bias += lsm6.g.z;

        vTaskDelay(pdMS_TO_TICKS(2));
    }

    gyro_bias /= GYRO_CALIBRATION_SAMPLES;

    Serial.println("Gyro calibration complete");

    const TickType_t period = pdMS_TO_TICKS(2);
    TickType_t last_wake = xTaskGetTickCount();

    while (1)
    {
        lsm6.read();

        sensor_packet.timestamp = xTaskGetTickCount();

        sensor_packet.lsm6_gyro[2] = (lsm6.g.z - gyro_bias) * LSM6_GYRO_NORMALIZATION;

        lis3.read();

        sensor_packet.lis3_mag[0] = (lis3.m.x - mag_x_offset);
        sensor_packet.lis3_mag[1] = (lis3.m.y - mag_y_offset);
        sensor_packet.lis3_mag[2] = lis3.m.z;

        for (int i = 0; i < 3; i++)
        {
            sensor_packet.lsm6_gyro[i] = fmod(sensor_packet.lsm6_gyro[i], 360.0);
            sensor_packet.lis3_mag[i] = fmod(sensor_packet.lis3_mag[i], 360.0);
        }

        xQueueOverwrite(imu_queue, &sensor_packet);
        xTaskDelayUntil(&last_wake, period);
    }
}

void updateAHRS(void *param)
{
    sensorPacket sensor_packet;
    ahrsPacket ahrs_packet;
    ahrsPacket ahrs_packet_new = {};

    const TickType_t period = pdMS_TO_TICKS(1);
    TickType_t last_wake = xTaskGetTickCount();
    const float dt = 0.001;

    while (1)
    {
        vTaskDelayUntil(&last_wake, period);

        xQueuePeek(imu_queue, &sensor_packet, 0);
        xQueuePeek(ahrs_queue, &ahrs_packet, 0);
    
        ahrs_packet_new.gyro_rate = sensor_packet.lsm6_gyro[2];
        ahrs_packet_new.gyro_angle += ahrs_packet.gyro_rate * ahrs_dt;

        ahrs_packet.mag_heading = atan2f(sensor_packet.lis3_mag[1], sensor_packet.lis3_mag[0]);

        orientation_z = ahrs_packet.mag_heading;
        ahrs_packet.timestamp = xTaskGetTickCount();

        xQueueOverwrite(ahrs_queue, &ahrs_packet_new);
    }
}

void moveServo(void *param)
{
    while (1)
    {
        switch(state)
        {
            case STRAIGHT_1:
                setpoint = 0.0;
                if (clicks >= 10) { state = TURNING; setpoint = 90.0; }
                break;
            case TURNING:
                setpoint = 90.0;
                if (fabs(orientation_z - setpoint) < 2.0) { state = STRAIGHT_2; }
                break;
            case STRAIGHT_2:
                setpoint = 90.0;
                break;
        }

        steering_correction.Compute();

        steering_angle = 90 + steering_angle_delta;
        steering_servo.write((int)steering_angle);

        vTaskDelay(pdMS_TO_TICKS(90));
    }
}

void handleSwitch(void *param)
{
    static uint32_t last_time = 0;

    while (1)
    {
        if (trigger)
        {
            trigger = false;
            uint32_t now = millis();

            if (now - last_time > DEBOUNCE_TIME)
            {
                clicks++;
                last_time = now;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void fireSolenoid(void *param)
{
    while(1)
    {
        solenoid_state = !solenoid_state;

        if (solenoid_state)
        {
            digitalWrite(SOLENOID_PIN, HIGH);
            vTaskDelay(pdMS_TO_TICKS(OPEN_TIME));
        }
        
        else
        {
            digitalWrite(SOLENOID_PIN, LOW);
            vTaskDelay(pdMS_TO_TICKS(CLOSE_TIME));
        }
    }
}

void updateSerial(void *param)
{
    ahrsPacket ahrs_packet;

    while(1)
    {
        xQueueReceive(ahrs_queue, &ahrs_packet, portMAX_DELAY);

        Serial.print(">ahrs_t:" + String(ahrs_packet.timestamp));
        Serial.print(",gyro_rate:" + String(ahrs_packet.gyro_rate));
        Serial.print(",gyro_angle:" + String(ahrs_packet.gyro_angle));
        Serial.print(",clicks:" + String(clicks));
        Serial.print(",steering_angle_delta:" + String(steering_angle_delta));
        Serial.println();

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void setup()
{
    Serial.begin(115200);
    delay(2000);
    Wire.begin();

    initializeLSM6();
    initalizeLIS3();

    imu_queue = xQueueCreate(1, sizeof(sensorPacket));
    ahrs_queue = xQueueCreate(1, sizeof(ahrsPacket));

    ahrsPacket initial_ahrs;
    initial_ahrs.timestamp = xTaskGetTickCount();
    xQueueOverwrite(ahrs_queue, &initial_ahrs);

    pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_PIN), limitSwitchISR, FALLING);

    steering_servo.attach(SERVO_PIN);
    steering_correction.SetMode(AUTOMATIC);
    steering_correction.SetOutputLimits(-45.0, 45.0);
    
    pinMode(SOLENOID_PIN, OUTPUT);

    xTaskCreate(readSensors, "IMU", 4096, NULL, 6, NULL);
    xTaskCreate(updateAHRS, "AHRS", 4096, NULL, 5, NULL);
    xTaskCreate(moveServo, "SERVO", 4096, NULL, 4, NULL);
    xTaskCreate(handleSwitch, "SWITCH", 4096, NULL, 3, NULL);
    xTaskCreate(fireSolenoid, "FIRE", 4096, NULL, 2, NULL);
    xTaskCreate(updateSerial, "DATA", 4096, NULL, 1, NULL);
}

void loop() { }