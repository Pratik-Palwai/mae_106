#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <LSM6.h>
#include <LIS3MDL.h>
#include <Servo.h>
#include <PID_v1.h>

LSM6 gyroscope;
const float GYRO_NORMALIZATION = 0.0875;

LIS3MDL magnetometer;
const float MAG_NORMALIZATION = RAD_TO_DEG;

#define EEPROM_X_OFFSET 0
#define EEPROM_Y_OFFSET 4

QueueHandle_t sensor_queue;
QueueHandle_t ahrs_queue;
float ahrs_dt = 0.001;

struct sensorPacket
{
    long timestamp = 0;
    float gyro_data[3] = {0.0, 0.0, 0.0};
    float mag_data[3] = {0.0, 0.0, 0.0};
};

struct ahrsPacket
{
    long timestamp = 0;
    float gyro_rate = 0.0;
    float gyro_heading = 0.0;
    float mag_heading = 0.0;
    float fused_heading = 0.0;
};

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
}

void calibrateLSM6(void) { }

void calibrateLIS3MDL(void) { }

void readSensors(void *param)
{
    sensorPacket sensor_packet;

    const TickType_t period = pdMS_TO_TICKS(2);
    TickType_t last_wake = xTaskGetTickCount();

    while (1)
    {
        gyroscope.read();
        magnetometer.read();
        
        sensor_packet.gyro_data[0] = gyroscope.g.x * GYRO_NORMALIZATION;
        sensor_packet.gyro_data[1] = gyroscope.g.y * GYRO_NORMALIZATION;
        sensor_packet.gyro_data[2] = gyroscope.g.z * GYRO_NORMALIZATION;

        sensor_packet.mag_data[0] = magnetometer.m.x;
        sensor_packet.mag_data[1] = magnetometer.m.y;
        sensor_packet.mag_data[2] = magnetometer.m.z;

        sensor_packet.timestamp = xTaskGetTickCount();

        xQueueOverwrite(sensor_queue, &sensor_packet);
        xTaskDelayUntil(&last_wake, period);
    }
}

void updateAHRS(void *param)
{
    sensorPacket sensor_packet;
    ahrsPacket ahrs_packet, ahrs_packet_new = ahrs_packet;

    const TickType_t period = pdMS_TO_TICKS(1);
    TickType_t last_wake = xTaskGetTickCount();

    while(1)
    {
        xQueueReceive(sensor_queue, &sensor_packet, portMAX_DELAY);
        xQueueReceive(ahrs_queue, &ahrs_packet, portMAX_DELAY);

        ahrs_packet_new.gyro_rate = sensor_packet.gyro_data[2];
        ahrs_packet_new.gyro_heading += ahrs_packet.gyro_rate * ahrs_dt;

        ahrs_packet_new.mag_heading = atan2f(sensor_packet.mag_data[1], sensor_packet.mag_data[0]);
        ahrs_packet_new.mag_heading *= RAD_TO_DEG;

        ahrs_packet_new.timestamp = xTaskGetTickCount();

        xQueueOverwrite(ahrs_queue, &ahrs_packet_new);
        xTaskDelayUntil(&last_wake, period);
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
        Serial.print(",mag_field_x:" + String(sensor_packet.mag_data[0]));
        Serial.print(",mag_field_y:" + String(sensor_packet.mag_data[1]));
        Serial.print(",mag_field_z:" + String(sensor_packet.mag_data[2]));
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
    initalizeLIS3MDL();

    sensor_queue = xQueueCreate(1, sizeof(sensorPacket));
    ahrs_queue = xQueueCreate(1, sizeof(ahrsPacket));

    ahrsPacket initial_ahrs;
    initial_ahrs.timestamp = xTaskGetTickCount();
    xQueueOverwrite(ahrs_queue, &initial_ahrs);

    sensorPacket initial_sensor;
    initial_sensor.timestamp = xTaskGetTickCount();
    xQueueOverwrite(sensor_queue, &initial_sensor);

    xTaskCreate(readSensors, "READ", 4096, NULL, 3, NULL);
    xTaskCreate(updateAHRS, "AHRS", 4096, NULL, 2, NULL);
    xTaskCreate(serialOutput, "DATA", 4096, NULL, 1, NULL);
}

void loop() { }