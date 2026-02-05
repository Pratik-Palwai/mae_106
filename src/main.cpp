#include <Arduino.h>
#include <numeric>
#include <Wire.h>
#include <EEPROM.h>
#include <LSM6.h>
#include <LIS3MDL.h>
#include <Servo.h>
#include <PID_v1.h>

LSM6 gyroscope;
const float GYRO_NORMALIZATION = 0.01;
float gyro_rate_bias = 0.0;
bool initial_gyro_offset = false;

LIS3MDL magnetometer;
LIS3MDL::vector<int16_t> mag_min = {-32767, -32767, -32767};
LIS3MDL::vector<int16_t> mag_max = {+32767, +32767, +32767};
int16_t running_min[3] = {32767, 32767, 32767}, running_max[3] = {0, 0, 0};
int x_offset = 0, y_offset = 0;
#define EEPROM_MAG_X_OFFSET 0
#define EEPROM_MAG_Y_OFFSET 4

QueueHandle_t sensor_queue;
QueueHandle_t ahrs_queue;
float ahrs_dt = 0.001;

const int LIMIT_SWITCH_PIN = D0, DEBOUNCE_TIME = 50;
bool trigger = false;
volatile int clicks = 0;

Servo steering_servo;
const int SERVO_PIN = D1;

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
    float fused_heading = 0.0;
};

void IRAM_ATTR limitSwitchISR() { trigger = false; }

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
    const int GYRO_CALIBRATION_SAMPLES = 5000;
    double rates_sum = 0.0;

    Serial.println("Starting gyro calibration loop . . . ");

    for (int i = 0; i < GYRO_CALIBRATION_SAMPLES; i++)
    {
        gyroscope.read();
        rates_sum += gyroscope.g.z * GYRO_NORMALIZATION;
    }

    Serial.println("Finished gyro calibration loop . . . ");

    gyro_rate_bias = rates_sum / GYRO_CALIBRATION_SAMPLES;
    Serial.println("gyro_rate_bias:" + String(gyro_rate_bias));
}

void calibrateLIS3MDL(void)
{
    Serial.println("Calibrating... rotate sensor slowly in all directions for 10 sec");

    float x_min = 32767, x_max = -32768;
    float y_min = 32767, y_max = -32768;

    unsigned long t0 = millis();
    while (millis() - t0 < 10000)
    {
        magnetometer.read();

        if (magnetometer.m.x < x_min) x_min = magnetometer.m.x;
        if (magnetometer.m.x > x_max) x_max = magnetometer.m.x;
        if (magnetometer.m.y < y_min) y_min = magnetometer.m.y;
        if (magnetometer.m.y > y_max) y_max = magnetometer.m.y;

        delay(10);
    }

    x_offset = (x_max + x_min) / 2.0;
    y_offset = (y_max + y_min) / 2.0;

    EEPROM.put(EEPROM_MAG_X_OFFSET, x_offset);
    EEPROM.put(EEPROM_MAG_Y_OFFSET, y_offset);
}

void readSensors(void *param)
{
    sensorPacket pkt;

    while (1)
    {
        gyroscope.read();
        magnetometer.read();

        pkt.timestamp = xTaskGetTickCount();
        pkt.gyro_data[0] = (gyroscope.g.x * GYRO_NORMALIZATION) - gyro_rate_bias;
        pkt.gyro_data[1] = (gyroscope.g.y * GYRO_NORMALIZATION) - gyro_rate_bias;
        pkt.gyro_data[2] = (gyroscope.g.z * GYRO_NORMALIZATION) - gyro_rate_bias;

        pkt.mag_data[0] = magnetometer.m.x - x_offset;
        pkt.mag_data[1] = magnetometer.m.y - x_offset;
        pkt.mag_data[2] = magnetometer.m.z;

        xQueueOverwrite(sensor_queue, &pkt);
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

float computeMagHeading(const sensorPacket &pkt)
{
    LIS3MDL::vector<int32_t> m = { (int32_t)pkt.mag_data[0], (int32_t)pkt.mag_data[1], (int32_t)pkt.mag_data[2] };

    LIS3MDL::vector<int16_t> a = { (int16_t)gyroscope.a.x, (int16_t)gyroscope.a.y, (int16_t)gyroscope.a.z };

    m.x -= ((int32_t)mag_min.x + mag_max.x) / 2;
    m.y -= ((int32_t)mag_min.y + mag_max.y) / 2;
    m.z -= ((int32_t)mag_min.z + mag_max.z) / 2;

    LIS3MDL::vector<float> E, N;
    LIS3MDL::vector_cross(&m, &a, &E);
    LIS3MDL::vector_normalize(&E);

    LIS3MDL::vector_cross(&a, &E, &N);
    LIS3MDL::vector_normalize(&N);

    LIS3MDL::vector<float> from = {1.0f, 0.0f, 0.0f};

    float heading = atan2(LIS3MDL::vector_dot(&E, &from), LIS3MDL::vector_dot(&N, &from));
    heading *= RAD_TO_DEG;

    if (heading < 0) { heading += 360.0f; }
    return heading;
}

void computeAHRS(void *param)
{
    sensorPacket sensor_packet;
    ahrsPacket ahrs_packet;
    ahrsPacket ahrs_packet_new;

    const float alpha = 0.98;

    while (1)
    {
        xQueuePeek(sensor_queue, &sensor_packet, portMAX_DELAY);
        xQueuePeek(ahrs_queue, &ahrs_packet, 0);

        float gyro_rate = sensor_packet.gyro_data[2];
        float gyro_heading = gyro_heading + (gyro_rate * ahrs_dt);

        if (gyro_heading < 0)   gyro_heading += 360.0f;
        if (gyro_heading >= 360.0f) gyro_heading -= 360.0f;

        float mag_heading = computeMagHeading(sensor_packet);

        float fused_heading = alpha * gyro_heading + (1.0f - alpha) * mag_heading;

        if (fused_heading < 0)   fused_heading += 360.0f;
        if (fused_heading >= 360.0f) fused_heading -= 360.0f;

        ahrs_packet_new.timestamp = xTaskGetTickCount();
        ahrs_packet_new.gyro_rate = gyro_rate;
        ahrs_packet_new.gyro_heading = gyro_heading;
        ahrs_packet_new.mag_heading = mag_heading;
        ahrs_packet_new.fused_heading = fused_heading;

        xQueueOverwrite(ahrs_queue, &ahrs_packet_new);

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void handleSwitch(void *param)
{
    while (1)
    {
        trigger = (digitalRead(LIMIT_SWITCH_PIN));

        if (trigger)
        {
            clicks++;
            trigger = false;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void serialOutput(void *param)
{
    ahrsPacket ahrs_packet;
    sensorPacket sensor_packet;

    while(1)
    {
        xQueuePeek(ahrs_queue, &ahrs_packet, 0);
        xQueuePeek(sensor_queue, &sensor_packet, 0);

        Serial.print(">ahrs_t:" + String(ahrs_packet.timestamp));
        Serial.print(",gyro_rate:" + String(ahrs_packet.gyro_rate));
        Serial.print(",gyro_heading:" + String(ahrs_packet.gyro_heading));
        Serial.print(",mag_heading:" + String(ahrs_packet.mag_heading));
        Serial.print(",fused_heading:" + String(ahrs_packet.fused_heading));
        // Serial.print(",mag_field_x:" + String(sensor_packet.mag_data[0]));
        // Serial.print(",mag_field_y:" + String(sensor_packet.mag_data[1]));
        // Serial.print(",mag_field_z:" + String(sensor_packet.mag_data[2]));
        // Serial.print(",clicks:" + String(clicks));
        Serial.println();

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void setup()
{
    Serial.begin(115200);
    delay(2000);
    Wire.begin();

    initializeLSM6();
    initalizeLIS3MDL();

    calibrateLSM6();
    calibrateLIS3MDL();

    sensor_queue = xQueueCreate(1, sizeof(sensorPacket));
    ahrs_queue = xQueueCreate(1, sizeof(ahrsPacket));

    ahrsPacket initial_ahrs;
    initial_ahrs.timestamp = xTaskGetTickCount();
    xQueueOverwrite(ahrs_queue, &initial_ahrs);

    sensorPacket initial_sensor;
    initial_sensor.timestamp = xTaskGetTickCount();
    xQueueOverwrite(sensor_queue, &initial_sensor);

    pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_PIN), limitSwitchISR, RISING);

    steering_servo.attach(SERVO_PIN);

    xTaskCreate(readSensors, "SENSE", 4096, NULL, 4, NULL);
    xTaskCreate(computeAHRS, "AHRS", 4096, NULL, 3, NULL);
    xTaskCreate(handleSwitch, "SWITCH", 4096, NULL, 2, NULL);
    xTaskCreate(serialOutput, "DATA", 4096, NULL, 1, NULL);
}

void loop() { }