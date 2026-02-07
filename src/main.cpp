#include <Arduino.h>
#include <numeric>
#include <Wire.h>
#include <LSM6.h>
#include <LIS3MDL.h>
#include <Servo.h>
#include <PID_v1.h>
#include <SimpleKalmanFilter.h>

LSM6 gyroscope;
const int GYRO_CALIBRATION_SAMPLES = 2000;
const float GYRO_NORMALIZATION = 0.00891089108;
float gyro_rate_bias = 0.0;
float gyro_mag_offset = 0.0;

LIS3MDL magnetometer;
const int MAGNETOMETER_CALIBRATION_TIME = 10000;
int mag_x_offset = 0, mag_y_offset = 0;
float mag_x_scaling = 1.0, mag_y_scaling = 1.0;

SimpleKalmanFilter heading_correction(6, 6, 0.01);

QueueHandle_t sensor_queue;
QueueHandle_t ahrs_queue;

const int BUZZER_PIN = D9;

const int LIMIT_SWITCH_PIN = D0, DEBOUNCE_TIME = 50;
volatile bool trigger = false;

Servo steering_servo;
const int SERVO_PIN = D1;

const int SOLENOID_PIN = D7;
const int OPEN_TIME = 250, CLOSE_TIME = 750;
volatile bool solenoid_state = false;

double pid_input = 0.0, desired_heading = 0.0, pid_output = 0.0;
const float K_P = 3.0, K_I = 0.0, K_D = 0.0;
PID steering_correction(&pid_input, &pid_output, &desired_heading, K_P, K_I, K_D, REVERSE);

struct sensorPacket
{
    double timestamp = 0;
    double gyro_data[3] = {0.0, 0.0, 0.0};
    double mag_data[3] = {0.0, 0.0, 0.0};
};

struct ahrsPacket
{
    long timestamp = 0;
    float gyro_rate = 0.0;
    float gyro_heading = 0.0;
    float mag_heading = 0.0;
    float filtered_heading = 0.0;
    int clicks = 0;
    int robot_heading = 0; // 0:initial heading, 1:turning, 2:final heading
};

inline void delayCalibration()
{
    delay(1000);
    digitalWrite(BUZZER_PIN, HIGH);
    delay(500);
    digitalWrite(BUZZER_PIN, LOW);
    delay(1000);
}

float deltaTheta(float a, float b)
{
    float c = a - b;
    while (c > 180.0) { c -= 360.0; }
    while (c < -180.0) {c += 360.0; }

    return c;
}

float wrapAngle(float theta_i)
{
    while (theta_i < 0.0) { theta_i += 360.0; }
    while (theta_i >= 360.0) { theta_i -= 360; }
    return theta_i;
}

void IRAM_ATTR limitSwitchISR() { trigger = true; }

void initializeLSM6(void)
{
    if (!gyroscope.init())
    {
        Serial.println("Failed to detect/initialize LSM6");
        while(1);
    }

    else { Serial.println("Sucessfully initialized LSM6"); }

    gyroscope.enableDefault();
}

void initalizeLIS3MDL(void)
{
    if (!magnetometer.init())
    {
        Serial.println("Failed to detect/initialize LIS3MDL");
        while(1);
    }

    else { Serial.println("Successfully initialized LIS3MDL"); }

    magnetometer.enableDefault();
}

void calibrateLSM6(void)
{
    double rates_sum = 0.0;

    Serial.println("Starting gyro calibration loop ... ");

    for (int i = 0; i < GYRO_CALIBRATION_SAMPLES; i++)
    {
        gyroscope.read();
        rates_sum += gyroscope.g.z * GYRO_NORMALIZATION;
        delay(2);
    }

    Serial.println("    Finished");

    gyro_rate_bias = rates_sum / GYRO_CALIBRATION_SAMPLES;
    Serial.println("    gyro_rate_bias:" + String(gyro_rate_bias));
}

void calibrateLIS3MDL(void)
{
    float x_min = 32767, x_max = -32768;
    float y_min = 32767, y_max = -32768;
    int16_t running_min[3] = {32767, 32767, 32767}, running_max[3] = {-32768, -32768, -32768};

    Serial.println("Starting magnetometer calibration loop ... ");

    unsigned long start_time = millis();
    while (millis() - start_time < MAGNETOMETER_CALIBRATION_TIME)
    {
        magnetometer.read();

        if (magnetometer.m.x < x_min) x_min = magnetometer.m.x;
        if (magnetometer.m.x > x_max) x_max = magnetometer.m.x;
        if (magnetometer.m.y < y_min) y_min = magnetometer.m.y;
        if (magnetometer.m.y > y_max) y_max = magnetometer.m.y;

        delay(10);
    }

    mag_x_offset = (x_max + x_min) / 2.0;
    mag_y_offset = (y_max + y_min) / 2.0;

    float radius_x = (x_max - x_min) / 2.0;
    float radius_y = (y_max - y_min) / 2.0;
    float radius_avg = (radius_x + radius_y) / 2.0;

    mag_x_scaling = radius_avg / radius_x;
    mag_y_scaling = radius_avg / radius_y;

    Serial.println("    Finished");
    Serial.println("    mag_x_offset:" + String(mag_x_offset) + " | mag_y_offset:" + String(mag_y_offset));
    Serial.println("    mag_x_scaling:" + String(mag_x_scaling) + " | mag_y_scaling:" + String(mag_y_scaling));
}

sensorPacket getSensorData()
{
    sensorPacket sensor_packet;

    gyroscope.read();
    magnetometer.read();

    sensor_packet.timestamp = pdTICKS_TO_MS(xTaskGetTickCount());
    sensor_packet.gyro_data[0] = gyroscope.g.x * GYRO_NORMALIZATION;
    sensor_packet.gyro_data[1] = gyroscope.g.y * GYRO_NORMALIZATION;
    sensor_packet.gyro_data[2] = - ((gyroscope.g.z * GYRO_NORMALIZATION) - gyro_rate_bias);

    sensor_packet.mag_data[0] = (magnetometer.m.x - mag_x_offset) * mag_x_scaling;
    sensor_packet.mag_data[1] = (magnetometer.m.y - mag_y_offset) * mag_y_scaling;
    sensor_packet.mag_data[2] = magnetometer.m.z;

    return sensor_packet;
}

float computeMagHeading(const sensorPacket &sensor_packet)
{
    float x_component = sensor_packet.mag_data[0];
    float y_component = sensor_packet.mag_data[1];

    float heading = atan2f(y_component, x_component) * RAD_TO_DEG;
    if (heading < 0.0) { heading += 360.0; }

    return heading;
}

void gyroOffset()
{
    Serial.println("Computing gyro-mag alignment ...");

    float mag_sum = 0.0;
    int samples = 0;

    unsigned long start = millis();
    while (millis() - start < 2000)
    {
        sensorPacket s = getSensorData();
        mag_sum += computeMagHeading(s);
        samples++;
        delay(5);
    }

    float avg_mag = mag_sum / samples;
    gyro_mag_offset = wrapAngle(avg_mag);

    Serial.println("    Finished");
    Serial.println("    gyro_mag_offset: " + String(gyro_mag_offset));
}


void readSensorsRTOS(void *param)
{
    sensorPacket sensor_packet;

    while (1)
    {
        sensorPacket sensor_packet = getSensorData();
        xQueueOverwrite(sensor_queue, &sensor_packet);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void computeAHRS(void *param)
{
    sensorPacket sensor_packet;
    ahrsPacket ahrs_packet;
    ahrsPacket ahrs_packet_new;

    const float ALPHA = 0.02;
    TickType_t last_wake = xTaskGetTickCount();

    while (1)
    {
        xQueuePeek(sensor_queue, &sensor_packet, portMAX_DELAY);
        xQueuePeek(ahrs_queue, &ahrs_packet, portMAX_DELAY);

        const float AHRS_DT = 0.001;

        float gyro_rate = sensor_packet.gyro_data[2];
        float gyro_heading = ahrs_packet.gyro_heading + (gyro_rate * AHRS_DT);
        float corrected_gyro_heading = wrapAngle(gyro_heading);

        float mag_heading = computeMagHeading(sensor_packet);

        float filtered_heading = heading_correction.updateEstimate(mag_heading);

        ahrs_packet_new.timestamp = pdTICKS_TO_MS(xTaskGetTickCount());
        ahrs_packet_new.gyro_rate = gyro_rate;
        ahrs_packet_new.gyro_heading = fmod(corrected_gyro_heading, 360.0);
        ahrs_packet_new.mag_heading = fmod(mag_heading, 360.0);
        ahrs_packet_new.filtered_heading = fmod(filtered_heading, 360.0);
        ahrs_packet_new.robot_heading = ahrs_packet.robot_heading;

        xQueueOverwrite(ahrs_queue, &ahrs_packet_new);
        xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(AHRS_DT * 1000));
    }
}


void handleSwitch(void *param)
{
    while (1)
    {
        if (trigger)
        {
            vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_TIME));

            if (digitalRead(LIMIT_SWITCH_PIN) == LOW)
            {
                ahrsPacket ahrs_packet;
                xQueueReceive(ahrs_queue, &ahrs_packet, portMAX_DELAY);
                
                ahrs_packet.clicks++;
                xQueueOverwrite(ahrs_queue, &ahrs_packet);
            }

            trigger = false;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}


void steerRobot(void *param)
{
    static bool initialized = false;
    static float initial_heading = 0.0;
    static float turn_target = 90.0;

    while(1)
    {
        ahrsPacket ahrs_packet;
        xQueuePeek(ahrs_queue, &ahrs_packet, portMAX_DELAY);

        float current_heading = ahrs_packet.filtered_heading;
        pid_input = current_heading;

        if (!initialized)
        {
            initial_heading = current_heading;
            desired_heading = current_heading;
            initialized = true;
        }

        if ((ahrs_packet.robot_heading == 0) && (ahrs_packet.clicks > 10))
        {
            ahrs_packet.robot_heading = 1;
            turn_target = wrapAngle(current_heading + 90.0);
        }

        if (ahrs_packet.robot_heading == 1)
        {
            float angle_error = deltaTheta(current_heading, turn_target);

            if (abs(angle_error) < 15.0)
            {
                ahrs_packet.robot_heading = 2;
                desired_heading = turn_target;
            }
        }

        if (ahrs_packet.robot_heading == 0) { desired_heading = initial_heading; }
        else if (ahrs_packet.robot_heading == 1) { desired_heading = turn_target; }
        else if (ahrs_packet.robot_heading == 2) { desired_heading = turn_target; }

        steering_correction.Compute();
        int servo_command = int(90.0 + pid_output);
        servo_command = constrain(servo_command, 45, 135);
        steering_servo.write(servo_command);

        xQueueOverwrite(ahrs_queue, &ahrs_packet);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void fireSolenoid(void *param)
{
    while(1)
    {
        digitalWrite(SOLENOID_PIN, solenoid_state);
        solenoid_state = !solenoid_state;

        if (solenoid_state) { vTaskDelay(pdMS_TO_TICKS(CLOSE_TIME)); }
        else { vTaskDelay(pdMS_TO_TICKS(OPEN_TIME)); }
    }
}

void serialOutput(void *param)
{
    ahrsPacket ahrs_packet;
    sensorPacket sensor_packet;

    while(1)
    {
        xQueuePeek(ahrs_queue, &ahrs_packet, 0);

        Serial.print(">ahrs_t:" + String(ahrs_packet.timestamp));
        Serial.print(",gyro_rate:" + String(ahrs_packet.gyro_rate));
        Serial.print(",gyro_heading:" + String(ahrs_packet.gyro_heading));
        Serial.print(",mag_heading:" + String(ahrs_packet.mag_heading));
        Serial.print(",filtered_heading:" + String(ahrs_packet.filtered_heading));
        Serial.print(",clicks:" + String(ahrs_packet.clicks));
        Serial.println();

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void setup()
{
    Serial.begin(115200);
    delay(2000);
    Wire.begin();

    delayCalibration();
    initializeLSM6();
    initalizeLIS3MDL();
    delayCalibration();

    calibrateLIS3MDL();
    delayCalibration();
    calibrateLSM6();
    delayCalibration();

    gyroOffset();
    delayCalibration();

    sensor_queue = xQueueCreate(1, sizeof(sensorPacket));
    ahrs_queue = xQueueCreate(1, sizeof(ahrsPacket));

    ahrsPacket initial_ahrs;
    initial_ahrs.timestamp = xTaskGetTickCount();
    initial_ahrs.gyro_heading = gyro_mag_offset;
    
    xQueueOverwrite(ahrs_queue, &initial_ahrs);

    sensorPacket initial_sensor;
    initial_sensor.timestamp = xTaskGetTickCount();
    xQueueOverwrite(sensor_queue, &initial_sensor);

    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(SOLENOID_PIN, OUTPUT);
    pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_PIN), limitSwitchISR, FALLING);

    steering_servo.attach(SERVO_PIN);
    steering_correction.SetOutputLimits(-45.0, 45.0);

    xTaskCreate(readSensorsRTOS, "SENSE", 4096, NULL, 6, NULL);
    xTaskCreate(computeAHRS, "AHRS", 4096, NULL, 5, NULL);
    xTaskCreate(steerRobot, "SERVO", 4096, NULL, 4, NULL);
    xTaskCreate(fireSolenoid, "FIRE", 4096, NULL, 3, NULL);
    xTaskCreate(handleSwitch, "SWITCH", 4096, NULL, 2, NULL);
    xTaskCreate(serialOutput, "DATA", 4096, NULL, 1, NULL);
}

void loop() { }