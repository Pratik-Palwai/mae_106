#include <Arduino.h>
#include <Wire.h>
#include <LSM6.h>
#include <LIS3MDL.h>
#include <Servo.h>
#include <PID_v1.h>

LSM6 gyroscope;
const double GYRO_CALIBRATION_SAMPLES = 5000;
const float GYRO_NORMALIZATION = 0.00891089108;
double gyro_rate_bias = 0.0;
double gyro_mag_offset = 0.0;

LIS3MDL magnetometer;
const double MAGNETOMETER_CALIBRATION_TIME = 10000;
double mag_x_offset = 0, mag_y_offset = 0;
float mag_x_scaling = 1.0, mag_y_scaling = 1.0;

QueueHandle_t sensor_queue;
QueueHandle_t robot_state_queue;

const double BUZZER_PIN = D0;

const double LIMIT_SWITCH_PIN = D10, DEBOUNCE_TIME = 50;
volatile bool trigger = false;

Servo steering_servo;
const int SERVO_PIN = D1;

const double SOLENOID_PIN = D7;
const double OPEN_TIME = 250, CLOSE_TIME = 750;
volatile bool solenoid_state = false;

double pid_input = 0.0, desired_heading = 0.0, pid_output = 0.0;
const double K_P = 3.0, K_I = 0.0, K_D = 0.0;
PID steering_correction(&pid_input, &pid_output, &desired_heading, K_P, K_I, K_D, REVERSE);

struct sensorPacket
{
    double timestamp = 0;
    double gyro_data[3] = {0.0, 0.0, 0.0};
    double mag_data[3] = {0.0, 0.0, 0.0};
};

struct robotStatePacket
{
    long timestamp = 0;
    double gyro_rate = 0.0;
    double gyro_heading = 0.0;
    double mag_heading = 0.0;
    double filtered_heading = 0.0;
    double clicks = 0;
    double heading_state = 0; // 0:initial heading, 1:turning, 2:final heading
};

inline void delayCalibration()
{
    digitalWrite(BUZZER_PIN, HIGH);
    delay(500);
    digitalWrite(BUZZER_PIN, LOW);
    delay(2000);
}

double deltaTheta(double a, double b)
{
    double c = a - b;
    while (c > 180.0) { c -= 360.0; }
    while (c < -180.0) {c += 360.0; }

    return c;
}

double wrapAngle(double theta_i)
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

    Serial.println("    finished");

    gyro_rate_bias = rates_sum / GYRO_CALIBRATION_SAMPLES;
    Serial.println("    gyro_rate_bias:" + String(gyro_rate_bias));
}

void calibrateLIS3MDL(void)
{
    double x_min = 32767, x_max = -32768;
    double y_min = 32767, y_max = -32768;

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

    double radius_x = (x_max - x_min) / 2.0;
    double radius_y = (y_max - y_min) / 2.0;
    double radius_avg = (radius_x + radius_y) / 2.0;

    mag_x_scaling = radius_avg / radius_x;
    mag_y_scaling = radius_avg / radius_y;

    Serial.println("    finished");
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

double computeMagHeading(const sensorPacket &sensor_packet)
{
    double x_component = sensor_packet.mag_data[0];
    double y_component = sensor_packet.mag_data[1];

    double heading = atan2f(y_component, x_component) * RAD_TO_DEG;
    if (heading < 0.0) { heading += 360.0; }

    return heading;
}

void gyroOffset()
{
    Serial.println("Computing gyro-mag alignment ...");

    double mag_sum = 0.0;
    int samples = 0;

    unsigned long start = millis();
    while (millis() - start < 2000)
    {
        sensorPacket s = getSensorData();
        mag_sum += computeMagHeading(s);
        samples++;
        delay(5);
    }

    double avg_mag = mag_sum / samples;
    gyro_mag_offset = wrapAngle(avg_mag);

    Serial.println("    finished");
    Serial.println("    gyro_mag_offset: " + String(gyro_mag_offset));
}


void readSensorsRTOS(void *param)
{
    sensorPacket sensor_packet;

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1);

    while (1)
    {
        sensorPacket sensor_packet = getSensorData();
        xQueueOverwrite(sensor_queue, &sensor_packet);
        xTaskDelayUntil(&last_wake, period);
    }
}

void updateRobotState(void *param)
{
    sensorPacket sensor_packet;
    robotStatePacket robot_state_packet;
    robotStatePacket robot_state_packet_new;

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1);

    while (1)
    {
        xQueuePeek(sensor_queue, &sensor_packet, portMAX_DELAY);
        xQueuePeek(robot_state_queue, &robot_state_packet, portMAX_DELAY);

        const double DELTA_T = 0.001;

        double gyro_rate = sensor_packet.gyro_data[2];
        double gyro_heading = robot_state_packet.gyro_heading + (gyro_rate * DELTA_T);
        double corrected_gyro_heading = wrapAngle(gyro_heading);

        double mag_heading = computeMagHeading(sensor_packet);

        static bool initialized = false;
        static double fused_heading;

        if (!initialized)
        {
            fused_heading = robot_state_packet.gyro_heading;
            initialized = true;
        }

        fused_heading = wrapAngle(fused_heading + gyro_rate * DELTA_T);
        double mag_error = deltaTheta(mag_heading, fused_heading);

        const double K = 0.002; // smaller values trust gyro more
        fused_heading = wrapAngle(fused_heading + K * mag_error);

        robot_state_packet_new.filtered_heading = fused_heading;
        robot_state_packet_new.gyro_heading = corrected_gyro_heading;
        robot_state_packet_new.gyro_rate = gyro_rate;
        robot_state_packet_new.mag_heading = mag_heading;
        robot_state_packet_new.timestamp = pdTICKS_TO_MS(xTaskGetTickCount());
        robot_state_packet_new.clicks = robot_state_packet.clicks;
        robot_state_packet_new.heading_state = 

        xQueueOverwrite(robot_state_queue, &robot_state_packet_new);
        xTaskDelayUntil(&last_wake, period);
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
                robotStatePacket robot_state_packet;
                xQueuePeek(robot_state_queue, &robot_state_packet, portMAX_DELAY);

                robot_state_packet.clicks += 1;
                xQueueOverwrite(robot_state_queue, &robot_state_packet);
                last_time = now;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void steerRobot(void *param)
{
    static bool initialized = false;
    static double initial_heading = 0.0;
    static double turn_target = 90.0;

    while(1)
    {
        robotStatePacket robot_state_packet;
        xQueuePeek(robot_state_queue, &robot_state_packet, portMAX_DELAY);

        double current_heading = robot_state_packet.filtered_heading;
        pid_input = current_heading;

        if (!initialized)
        {
            initial_heading = current_heading;
            desired_heading = current_heading;
            initialized = true;
        }

        if ((robot_state_packet.heading_state == 0) && (robot_state_packet.clicks > 10))
        {
            robot_state_packet.heading_state = 1;
            turn_target = wrapAngle(current_heading + 90.0);
        }

        if (robot_state_packet.heading_state == 1)
        {
            double angle_error = deltaTheta(current_heading, turn_target);

            if (abs(angle_error) < 15.0)
            {
                robot_state_packet.heading_state = 2;
                desired_heading = turn_target;
            }
        }

        if (robot_state_packet.heading_state == 0) { desired_heading = initial_heading; }
        else if (robot_state_packet.heading_state == 1) { desired_heading = turn_target; }
        else if (robot_state_packet.heading_state == 2) { desired_heading = turn_target; }

        steering_correction.Compute();
        int servo_command = int(90.0 + pid_output);
        servo_command = constrain(servo_command, 45, 135);
        steering_servo.write(servo_command);

        xQueueOverwrite(robot_state_queue, &robot_state_packet);
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
    robotStatePacket robot_state_packet;
    sensorPacket sensor_packet;

    while(1)
    {
        xQueuePeek(robot_state_queue, &robot_state_packet, 0);

        Serial.print(">timestamp:" + String(robot_state_packet.timestamp));
        Serial.print(",gyro_rate:" + String(robot_state_packet.gyro_rate));
        Serial.print(",gyro_heading:" + String(robot_state_packet.gyro_heading));
        Serial.print(",mag_heading:" + String(robot_state_packet.mag_heading));
        Serial.print(",filtered_heading:" + String(robot_state_packet.filtered_heading));
        Serial.print(",clicks:" + String(robot_state_packet.clicks));
        Serial.print(",heading_state:" + String(robot_state_packet.heading_state));
        Serial.print(",desired_heading:" + String(desired_heading));
        Serial.print(",servo_correction:" + String(pid_output));
        Serial.print(",solenoid_state:" + String(solenoid_state));
        Serial.println();

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void setup()
{
    Serial.begin(115200);
    delay(2000);
    Wire.begin();

    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(SOLENOID_PIN, OUTPUT);
    pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_PIN), limitSwitchISR, FALLING);

    steering_servo.attach(SERVO_PIN);
    steering_correction.SetOutputLimits(-45.0, 45.0);
    steering_correction.SetMode(AUTOMATIC);

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
    robot_state_queue = xQueueCreate(1, sizeof(robotStatePacket));

    robotStatePacket initial_robot_state;
    initial_robot_state.timestamp = xTaskGetTickCount();
    initial_robot_state.gyro_heading = gyro_mag_offset;
    
    xQueueOverwrite(robot_state_queue, &initial_robot_state);

    sensorPacket initial_sensor;
    initial_sensor.timestamp = xTaskGetTickCount();
    xQueueOverwrite(sensor_queue, &initial_sensor);

    xTaskCreate(readSensorsRTOS, "SENSE", 4096, NULL, 6, NULL);
    xTaskCreate(updateRobotState, "STATE", 4096, NULL, 5, NULL);
    xTaskCreate(steerRobot, "SERVO", 4096, NULL, 4, NULL);
    xTaskCreate(fireSolenoid, "FIRE", 4096, NULL, 3, NULL);
    xTaskCreate(handleSwitch, "SWITCH", 4096, NULL, 2, NULL);
    xTaskCreate(serialOutput, "DATA", 4096, NULL, 1, NULL);
}

void loop() { }