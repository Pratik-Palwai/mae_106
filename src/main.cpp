#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <LSM6.h>
#include <Adafruit_LIS3MDL.h>
#include <Servo.h>
#include <PID_v1.h>

// imu + ahrs variables
LSM6 lsm6;
const float GYRO_CALIBRATION_SAMPLES = 500;
const float LSM6_IMU_NORMALIZATION = 0.00061;
const float LSM6_GYRO_NORMALIZATION = 0.00875;

Adafruit_LIS3MDL lis3;

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

struct imuPacket
{
    long timestamp = 0;
    float lsm6_imu[3] = {0.0, 0.0, 0.0};
    float lsm6_gyro[3] = {0.0, 0.0, 0.0};
    float lis3_mag[3] = {0.0, 0.0, 0.0};
};

struct ahrsPacket
{
    long timestamp = 0;
    float angles[3] = {0.0, 0.0, 0.0};
    float rates[3] = {0.0, 0.0, 0.0};
    float position[3] = {0.0, 0.0, 0.0};
    float velocity[3] = {0.0, 0.0, 0.0};
    float acceleration[3] = {0.0, 0.0, 0.0};
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

void initalizeLIS3(void)
{
    if (!lis3.begin_I2C(0x1E))
    {
        Serial.println("Failed to detect/initialize LIS3");
        while(1);
    }

    else
    {
        lis3.setOperationMode(LIS3MDL_CONTINUOUSMODE);
        lis3.setRange(LIS3MDL_RANGE_8_GAUSS);
        lis3.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
        lis3.setDataRate(LIS3MDL_DATARATE_1000_HZ);

        Serial.println("Successfully initialized LIS3");
    }
}

void readIMU(void *param)
{
    imuPacket imu_packet;

    float gyro_bias[3] = {0.0, 0.0, 0.0};

    Serial.println("Calibrating gyro");

    for (int i = 0; i < GYRO_CALIBRATION_SAMPLES; i++)
    {
        lsm6.read();

        gyro_bias[0] += lsm6.g.x;
        gyro_bias[1] += lsm6.g.y;
        gyro_bias[2] += lsm6.g.z;

        vTaskDelay(pdMS_TO_TICKS(2));
    }

    gyro_bias[0] /= GYRO_CALIBRATION_SAMPLES;
    gyro_bias[1] /= GYRO_CALIBRATION_SAMPLES;
    gyro_bias[2] /= GYRO_CALIBRATION_SAMPLES;

    Serial.println("Gyro calibration complete");

    const TickType_t period = pdMS_TO_TICKS(2);
    TickType_t last_wake = xTaskGetTickCount();

    while (1)
    {
        lsm6.read();

        imu_packet.timestamp = xTaskGetTickCount();

        imu_packet.lsm6_imu[0] = lsm6.a.x * LSM6_IMU_NORMALIZATION;
        imu_packet.lsm6_imu[1] = lsm6.a.y * LSM6_IMU_NORMALIZATION;
        imu_packet.lsm6_imu[2] = lsm6.a.z * LSM6_IMU_NORMALIZATION;

        imu_packet.lsm6_gyro[0] = (lsm6.g.x - gyro_bias[0]) * LSM6_GYRO_NORMALIZATION;
        imu_packet.lsm6_gyro[1] = (lsm6.g.y - gyro_bias[1]) * LSM6_GYRO_NORMALIZATION;
        imu_packet.lsm6_gyro[2] = (lsm6.g.z - gyro_bias[2]) * LSM6_GYRO_NORMALIZATION;

        lis3.read();

        imu_packet.lis3_mag[0] = lis3.x;
        imu_packet.lis3_mag[1] = lis3.y;
        imu_packet.lis3_mag[2] = lis3.z;

        xQueueOverwrite(imu_queue, &imu_packet);
        xTaskDelayUntil(&last_wake, period);
    }
}

void updateAHRS(void *param)
{
    imuPacket imu_packet;
    ahrsPacket ahrs_packet = {};

    const TickType_t period = pdMS_TO_TICKS(1);
    TickType_t last_wake = xTaskGetTickCount();
    const float dt = 0.001;

    while (1)
    {
        vTaskDelayUntil(&last_wake, period);

        if (xQueuePeek(imu_queue, &imu_packet, 0) == pdTRUE)
        {
            for (int i = 0; i < 3; i++)
            {
                ahrs_packet.rates[i] = imu_packet.lsm6_gyro[i];
                ahrs_packet.angles[i] += ahrs_packet.rates[i] * dt;

                ahrs_packet.acceleration[i] = imu_packet.lsm6_imu[i];
                ahrs_packet.velocity[i] += ahrs_packet.acceleration[i] * dt;
                ahrs_packet.position[i] += ahrs_packet.velocity[i] * dt;
            }

            orientation_z = ahrs_packet.angles[2];
            ahrs_packet.timestamp = xTaskGetTickCount();

            xQueueOverwrite(ahrs_queue, &ahrs_packet);
        }
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
        Serial.print(",rate_x:" + String(ahrs_packet.rates[0]));
        Serial.print(",rate_y:" + String(ahrs_packet.rates[1]));
        Serial.print(",rate_z:" + String(ahrs_packet.rates[2]));
        Serial.print(",angle_x:" + String(ahrs_packet.angles[0]));
        Serial.print(",angle_y:" + String(ahrs_packet.angles[1]));
        Serial.print(",angle_z:" + String(ahrs_packet.angles[2]));
        Serial.print(",clicks:" + String(clicks));
        Serial.print(",steering_angle_delta:" + String(steering_angle_delta));

        // Serial.print(",accel_x:" + String(ahrs_packet.acceleration[0]));
        // Serial.print(",accel_y:" + String(ahrs_packet.acceleration[1]));
        // Serial.print(",accel_z:" + String(ahrs_packet.acceleration[2]));
        // Serial.print(",velocity_x:" + String(ahrs_packet.velocity[0]));
        // Serial.print(",velocity_y:" + String(ahrs_packet.velocity[1]));
        // Serial.print(",velocity_z:" + String(ahrs_packet.velocity[2]));
        // Serial.print(",position_x:" + String(ahrs_packet.position[0]));
        // Serial.print(",position_y:" + String(ahrs_packet.position[1]));
        // Serial.print(",position_z:" + String(ahrs_packet.position[2]));
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

    imu_queue = xQueueCreate(1, sizeof(imuPacket));
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

    xTaskCreate(readIMU, "IMU", 4096, NULL, 6, NULL);
    xTaskCreate(updateAHRS, "AHRS", 4096, NULL, 5, NULL);
    xTaskCreate(moveServo, "SERVO", 4096, NULL, 4, NULL);
    xTaskCreate(handleSwitch, "SWITCH", 4096, NULL, 3, NULL);
    xTaskCreate(fireSolenoid, "FIRE", 4096, NULL, 2, NULL);
    xTaskCreate(updateSerial, "DATA", 4096, NULL, 1, NULL);
}

void loop() { }