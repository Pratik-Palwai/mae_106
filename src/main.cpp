#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <LSM6.h>
#include <LIS3MDL.h>
#include <Servo.h>
#include <PID_v1.h>

#define EEPROM_MAG_X_SCALING_ADDRESS 0 // a float uses 4 bytes of memory so the addresses are spaced 4 bytes (32 bits) apart
#define EEPROM_MAG_Y_SCALING_ADDRESS 4
#define EEPROM_MAG_X_OFFSET_ADDRESS 8
#define EEPROM_MAG_Y_OFFSET_ADDRESS 12

// gyroscope variables. adafruit lsm6 library also exists but is more complicated
LSM6 gyroscope;
const long GYRO_CALIBRATION_SAMPLES = 15000; // 15 sec is the amount of time between the "Clear" signal and the race start, it is a good amount of time to calibrate the gyro
const float GYRO_NORMALIZATION = 0.00891089108;
float gyro_rate_bias = 0.0;
float gyro_heading_offset = 180.0;

// magnetometer (compass) variables. again the adafruit library offers more customization but reading the sensor requires more code
LIS3MDL magnetometer;
const long MAGNETOMETER_CALIBRATION_TIME = 10000;
float mag_x_offset = 0, mag_y_offset = 0;
float mag_x_scaling = 1.0, mag_y_scaling = 1.0;
float mag_heading_offset = 0.0; // for setting the initial compass heading to 180 before each run
const bool CALIBRATE_MAG = false; // true:calibrate magnetometer and store offsets/scaling to EEPROM, false:use saved EEPROM values

// RTOS queues. these are better for getting real-time data vs volatiles but it is a little more complicated to read/write from
QueueHandle_t sensor_queue;
QueueHandle_t ahrs_queue;

// buzzer pin. this beeps at the start and end of each calibration
const int BUZZER_PIN = D1;

// limit switch variables. On my robot the switch actuates once per revolution, and ten clicks is where the robot should start turning
const int LIMIT_SWITCH_PIN = D10, DEBOUNCE_TIME = 5;
const int CLICKS_BEFORE_TRENCH = 10, CLICKS_IN_TRENCH = 20, CLICKS_AFTER_TRENCH = 10;
volatile int clicks = 0; // variable is volatile because several tasks read/write to it
volatile int clicks_on_straight = 0;
bool trigger = false; // "latch" variable for software debouncing

// servo variables. any PWM-enabled pin can be used for this
Servo steering_servo;
const int SERVO_PIN = D0;

// solenoid variables. adjust open_time and close_time (milliseconds) to maximise distance travelled
const int SOLENOID_PIN = D7;
const int OPEN_TIME = 250, CLOSE_TIME = 750;
volatile bool solenoid_state = false, actuation = true;

// pid variables. see steerRobot() task to change turn behavior between left/right (+/- 90.0 deg)
double pid_input = 0.0, target_heading = 0.0, pid_output = 0.0;
const float K_P = 0.50, K_I = 0.0, K_D = 0.0; // tune PID constants for verification 2
PID steering_correction(&pid_input, &pid_output, &target_heading, K_P, K_I, K_D, REVERSE); // pid mode can be DIRECT or REVERSE depending on how the servo and magnetometer are mounted

// sensor packet overview. There is a timestamp for integration, and gyro + compass data on all three axes
struct sensorPacket
{
    float timestamp = 0;
    float gyro_data[3] = {0.0, 0.0, 0.0};
    float mag_data[3] = {0.0, 0.0, 0.0};
};

// ahrs packet overview. Not really a proper AHRS though
// similar to the sensor packet with a timestamp, but includes gyro/compass/filtered headings
struct AHRSPacket
{
    long timestamp = 0;
    float gyro_rate = 0.0;
    float gyro_heading = 0.0;
    float mag_heading = 0.0;
    float filtered_heading = 0.0;
    int heading_state = 0; // 0:initial heading, 1:initial turning, 2:trench heading, 3:final turning, 4:final heading, 5: stop
};

// LED blink before calibration. can be adjusted depending on personal preference and ease of moving the robot
inline void delayCalibration()
{
    digitalWrite(BUZZER_PIN, HIGH);
    delay(500);
    digitalWrite(BUZZER_PIN, LOW);
    delay(2000);
}

// difference between two given angles, accounding for 0/360 deg wrapping. used in updateAHRS()
float deltaTheta(float a, float b)
{
    float c = a - b;
    while (c > 180.0) { c -= 360.0; }
    while (c < -180.0) {c += 360.0; }

    return c;
}

// wraps given angle between 0 and 360 deg. used in computeMagHeading(), gyroOffset(), updateAHRS() and steerRobot()
float wrapAngle(float a)
{
    while (a < 0.0) { a += 360.0; }
    while (a >= 360.0) { a -= 360; }
    return a;
}

// hardware interrupt service routine (ISR) for listening to the limit switch
void IRAM_ATTR limitSwitchISR() { trigger = true; }

// initialize gyro I2C communication
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

// initialize compass I2C communcation
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

// calibrate gyro rate bias. Robot must be perfectly still during this routine and the gyro is sensitive to temperature changes
void calibrateLSM6(void)
{
    // the gyro drifts because even when it is still the z-axis rate is nonzero, leading to a linear drift that can be corrected
    double rates_sum = 0.0;

    Serial.println("Starting gyro calibration loop ... ");

    // loop to repeatedly increment the rates for averaging
    for (int i = 0; i < GYRO_CALIBRATION_SAMPLES; i++)
    {
        gyroscope.read();
        rates_sum += gyroscope.g.z * GYRO_NORMALIZATION;
        delay(1);
    }

    Serial.println("    finished");

    gyro_rate_bias = rates_sum / GYRO_CALIBRATION_SAMPLES;
    Serial.println("    gyro_rate_bias:" + String(gyro_rate_bias));
}

// calibrate local magnetic fields. Spin robot in a circle around magnetometer Z-axis
void calibrateLIS3MDL(void)
{
    if (CALIBRATE_MAG)
    {
        float x_min = 32767, x_max = -32768;
        float y_min = 32767, y_max = -32768;

        Serial.println("Starting magnetometer calibration loop ... ");

        // get x_min, x_max, y_min, y_max of the local b-fields
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

        // hard-iron correction
        mag_x_offset = (x_max + x_min) / 2.0;
        mag_y_offset = (y_max + y_min) / 2.0;

        // soft-iron correction
        float radius_x = (x_max - x_min) / 2.0;
        float radius_y = (y_max - y_min) / 2.0;
        float radius_avg = (radius_x + radius_y) / 2.0;

        mag_x_scaling = radius_avg / radius_x;
        mag_y_scaling = radius_avg / radius_y;

        Serial.println("    finished");
        Serial.println("    mag_x_offset:" + String(mag_x_offset) + " | mag_y_offset:" + String(mag_y_offset));
        Serial.println("    mag_x_scaling:" + String(mag_x_scaling) + " | mag_y_scaling:" + String(mag_y_scaling));
        
        EEPROM.put(EEPROM_MAG_X_OFFSET_ADDRESS, mag_x_offset);
        EEPROM.put(EEPROM_MAG_Y_OFFSET_ADDRESS, mag_y_offset);
        EEPROM.put(EEPROM_MAG_X_SCALING_ADDRESS, mag_x_scaling);
        EEPROM.put(EEPROM_MAG_Y_SCALING_ADDRESS, mag_y_scaling);

        EEPROM.commit();
        Serial.println("    saved calibrations to EEPROM");
    }

    else
    {
        Serial.println("Pulling compass calibrations from EEPROM ...");

        EEPROM.get(EEPROM_MAG_X_OFFSET_ADDRESS, mag_x_offset);
        EEPROM.get(EEPROM_MAG_Y_OFFSET_ADDRESS, mag_y_offset);
        EEPROM.get(EEPROM_MAG_X_SCALING_ADDRESS, mag_x_scaling);
        EEPROM.get(EEPROM_MAG_Y_SCALING_ADDRESS, mag_y_scaling);

        Serial.println("    finished");
        Serial.println("    mag_x_offset:" + String(mag_x_offset) + " | mag_y_offset:" + String(mag_y_offset));
        Serial.println("    mag_x_scaling:" + String(mag_x_scaling) + " | mag_y_scaling:" + String(mag_y_scaling));
    }
}

// function for reading both sensors. This is used in both the gyro and magnetometer offset calibrations so it is not inside the main RTOS task
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

// compute compass heading given magnetic fields in three axes. much simpler than the Pololu computeHeading() method 
float computeMagHeading(sensorPacket &sensor_packet)
{
    float x_component = sensor_packet.mag_data[0];
    float y_component = sensor_packet.mag_data[1];

    float heading = atan2f(y_component, x_component) * RAD_TO_DEG;
    heading = wrapAngle(heading);

    return heading;
}

// sets initial compass heading to 180 to match the gyro
void zeroMagHeading()
{
    Serial.println("Computing initial compass heading ...");

    float mag_sum = 0.0;
    int samples = 0;
    sensorPacket sensor_packet;
    unsigned long start = millis();

    // comput average compass heading and use that to set the initial point of the magnetometer to 180
    while(millis() - start < 2000)
    {
        sensor_packet = getSensorData();
        mag_sum += computeMagHeading(sensor_packet);
        samples++;
        delay(1);
    }

    float avg_mag_heading = mag_sum / samples;
    mag_heading_offset = avg_mag_heading - 180;

    Serial.println("    finished");
    Serial.println("    mag_heading_offset: " + String(mag_heading_offset));
}

// simple wrapper for the getSensorData() method. This task occurs at 1 kHz which is about the limit of the I2C communication between the MCU and sensor
void readSensorsRTOS(void *param)
{
    sensorPacket sensor_packet;

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1);

    while (1)
    {
        sensorPacket sensor_packet = getSensorData();
        xQueueOverwrite(sensor_queue, &sensor_packet);
        xTaskDelayUntil(&last_wake, period); // xTaskDelayUntil() is used here instead of vTaskDelay() to ensure a consistent task frequency
    }
}

// main task that updates the robot's position. also occurs at 1 kHz which prevents double-integrating gyro rates
void updateAHRS(void *param)
{
    sensorPacket sensor_packet;
    AHRSPacket ahrs_packet;
    AHRSPacket ahrs_packet_new;

    int32_t last_time = micros();

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1);

    while (1)
    {
        xQueuePeek(sensor_queue, &sensor_packet, portMAX_DELAY); // xQueuePeek() gets data from a queue without removing it from the queue
        xQueuePeek(ahrs_queue, &ahrs_packet, portMAX_DELAY); // xQueueReceive() removes data from the queue, which can cause errors if a queue is empty and another task tries to read the queue

        // ahrs timestamps are in microseconds instead of milliseconds to avoid rounding errors
        int32_t now = micros();
        float DELTA_T = (now - last_time) * 0.000001;
        last_time = now;

        // avoid incremental errors with closely-spaced task calls
        if (DELTA_T <= 0.0 || DELTA_T > 0.1) { DELTA_T = 0.00001; } 

        // gyro rate heading integration
        float gyro_rate = sensor_packet.gyro_data[2];
        float gyro_heading = ahrs_packet.gyro_heading + (gyro_rate * DELTA_T);
        float corrected_gyro_heading = wrapAngle(gyro_heading);

        float mag_heading = computeMagHeading(sensor_packet);
        
        float filtered_heading = ahrs_packet.filtered_heading;

        // Ignore magnetometer heading when solenoid is on and only use gyro heading
        if (!solenoid_state)
        {
            filtered_heading = wrapAngle(filtered_heading + gyro_rate * DELTA_T);
            float mag_error = deltaTheta(mag_heading, filtered_heading);

            const float K = 0.02;
            filtered_heading = wrapAngle(filtered_heading + K * mag_error);
        }
        else { filtered_heading = corrected_gyro_heading; }

        // write new ahrs_packet to queue
        ahrs_packet_new.filtered_heading = filtered_heading;
        ahrs_packet_new.gyro_heading = corrected_gyro_heading;
        ahrs_packet_new.gyro_rate = gyro_rate;
        ahrs_packet_new.mag_heading = mag_heading;
        ahrs_packet_new.timestamp = pdTICKS_TO_MS(xTaskGetTickCount()); 
        // heading_state is not modified here, only steerRobot() should write to this variable

        xQueueOverwrite(ahrs_queue, &ahrs_packet_new);
        xTaskDelayUntil(&last_wake, period); //  again, xTaskDelayUntil() is used here instead of vTaskDelay() to ensure a consistent task frequency
    }
}

// task to handle limit switch clicks. Runs at 50 Hz to prevent CPU hogging but can be set to run faster if high click rates are needed
void handleSwitch(void *param)
{
    static long last_time = 0;
    AHRSPacket ahrs_packet;

    while (1)
    {
        // software debounce is easier than using a capacitor
        if (trigger)
        {
            xQueuePeek(ahrs_queue, &ahrs_packet, portMAX_DELAY); // don't want to accidentally update clicks_on_straight with bad data from ahrs_queue so portMAX_DELAY is used

            trigger = false;
            long now = millis();

            if (now - last_time > DEBOUNCE_TIME)
            {
                clicks++;

                if ((ahrs_packet.heading_state == 0) || (ahrs_packet.heading_state == 2) || (ahrs_packet.heading_state == 4))
                {
                    clicks_on_straight++; // only update clicks_on_straight when the robot is not in a turning state
                    // clicks_on_straight is reset to zero in steerRobot()
                }

                last_time = now;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// task to control the servo: PID computation and turning at desired # of clicks
void steerRobot(void *param)
{
    static float initial_heading = 180.0;
    static float trench_heading = 90.0;
    static float final_heading = 180.0;

    while(1)
    {
        AHRSPacket ahrs_packet;
        xQueuePeek(ahrs_queue, &ahrs_packet, portMAX_DELAY);

        pid_input = ahrs_packet.filtered_heading;

        // target heading update sequence
        if (ahrs_packet.heading_state == 0) { target_heading = initial_heading; }
        else if (ahrs_packet.heading_state == 1) { target_heading = trench_heading; }
        else if (ahrs_packet.heading_state == 2) { target_heading = trench_heading; }
        else if (ahrs_packet.heading_state == 3) { target_heading = final_heading; }
        else if (ahrs_packet.heading_state == 4) { target_heading = final_heading; }
        else if (ahrs_packet.heading_state == 5) { continue; }

        if ((ahrs_packet.heading_state == 0) && (clicks_on_straight > CLICKS_BEFORE_TRENCH)) { ahrs_packet.heading_state = 1; } // initial heading state -> initial turning state

        else if (ahrs_packet.heading_state == 1) // initial turning state -> trench heading state
        {
            float angle_error = deltaTheta(target_heading, pid_input);
            if (abs(angle_error) < 5.0) { ahrs_packet.heading_state = 2; }

            clicks_on_straight = 0; // don't increment this variable while turning
        }

        else if ((ahrs_packet.heading_state == 2) && (clicks_on_straight > CLICKS_IN_TRENCH)) { ahrs_packet.heading_state = 3; } // trench heading state -> final turning state

        else if (ahrs_packet.heading_state == 3) // final turning state --> final heading state
        {
            float angle_error = deltaTheta(target_heading, pid_input);
            if (abs(angle_error) < 5.0) { ahrs_packet.heading_state = 4; }

            clicks_on_straight = 0; // don't increment this variable while turning
        }

        else if ((ahrs_packet.heading_state == 4) && (clicks_on_straight > CLICKS_AFTER_TRENCH)) { ahrs_packet.heading_state = 5; } // final turning state -> final heading state

        // PID computation and conversion to servo angles
        steering_correction.Compute();
        int servo_command = int(90.0 + pid_output);
        steering_servo.write(servo_command);

        // Queue overwriting because the heading_state variable may have been updated
        xQueueOverwrite(ahrs_queue, &ahrs_packet);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// task to control the solenoid. Very simple especially with vTaskDelay(). Runs at 1 Hz but depends on open_time and close_time
void fireSolenoid(void *param)
{
    AHRSPacket ahrs_packet;

    while(1)
    {        
        xQueuePeek(ahrs_queue, &ahrs_packet, 0);

        // if (ahrs_packet.heading_state == 5) { actuation = false;} // stop actuating piston after robot has reached final waypoint
        // if (millis() >= 72000) { actuation = false; } // stop actuating piston after 72 seconds (57 seconds after the 15-second calibration)

        if (actuation) { digitalWrite(SOLENOID_PIN, solenoid_state); } // fire solenoid only if conditions are met

        if (solenoid_state) { vTaskDelay(pdMS_TO_TICKS(OPEN_TIME)); }
        else { vTaskDelay(pdMS_TO_TICKS(CLOSE_TIME)); }

        solenoid_state = !solenoid_state;
    }
}

// task to output data to serial. Occurs at 10 Hz and has lowest task priority so it doesn't interfere with robot control
// formatted for VS Code serial plotter
void serialOutput(void *param)
{
    AHRSPacket ahrs_packet;
    sensorPacket sensor_packet; // can also pull data

    while(1)
    {
        xQueuePeek(ahrs_queue, &ahrs_packet, 0); // use 0 instead of portMAX_DELAY to prevent blocking other tasks. If no data is recieved by this task no big deal
        xQueuePeek(sensor_queue, &sensor_packet, 0); 

        Serial.print(">timestamp:" + String(ahrs_packet.timestamp));
        Serial.print(",gyro_rate:" + String(ahrs_packet.gyro_rate));
        Serial.print(",gyro_heading:" + String(ahrs_packet.gyro_heading));
        Serial.print(",mag_heading:" + String(ahrs_packet.mag_heading));
        Serial.print(",filtered_heading:" + String(ahrs_packet.filtered_heading));
        Serial.print(",heading_state:" + String(ahrs_packet.heading_state));
        Serial.print(",clicks:" + String(clicks));
        Serial.print(",clicks_on_straight:" + String(clicks_on_straight));
        Serial.print(",target_heading:" + String(target_heading));
        Serial.print(",servo_correction:" + String(pid_output));
        Serial.print(",solenoid_state:" + String(solenoid_state));
        Serial.print(",actuation:" + String(int(actuation)));
        Serial.println();

        // vTaskDelay() is easier to implement but doesn't execute at exactly the frequency desired. For example next task run might occur in 105 ms
        // for tasks like serial output, PID computation, and solenoid firing, a few ms of timing error is not a problem
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void setup()
{
    Serial.begin(115200);
    delay(2000); // can replace with while(!Serial)
    Wire.begin();

    EEPROM.begin(32); // only 16 bytes are needed for storing values so 32 bytes gives a good margin

    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(SOLENOID_PIN, OUTPUT);
    pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
    digitalWrite(BUZZER_PIN, LOW); // initially turn buzzer off

    attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_PIN), limitSwitchISR, RISING); // every time the status of this GPIO pin changes the interrupt will trigger (high -> low or low -> high)

    steering_servo.attach(SERVO_PIN);
    steering_correction.SetOutputLimits(-40.0, 40.0); // adjust limits to tune how tightly the robot turns
    steering_correction.SetMode(AUTOMATIC);

    // sensor calibration block start
    delayCalibration();
    initializeLSM6();
    initalizeLIS3MDL();
    delayCalibration();

    calibrateLIS3MDL();
    delayCalibration();
    calibrateLSM6();
    delayCalibration();

    zeroMagHeading();
    delayCalibration();
    // sensor calibration block stop

    // queue creation for sensor and AHRS queues
    sensor_queue = xQueueCreate(1, sizeof(sensorPacket));
    ahrs_queue = xQueueCreate(1, sizeof(AHRSPacket));

    // ahrs queue seeding 
    AHRSPacket initial_ahrs;
    initial_ahrs.timestamp = xTaskGetTickCount();
    initial_ahrs.gyro_heading = gyro_heading_offset;
    xQueueOverwrite(ahrs_queue, &initial_ahrs);

    // sensor queue seeding
    sensorPacket initial_sensor;
    initial_sensor.timestamp = xTaskGetTickCount();
    xQueueOverwrite(sensor_queue, &initial_sensor);

    // task creation. Parameters: task, name, stack size [bytes], parameters (usually NULL), priority (highest priority -> greatest #), task pointer (also usually NULL);
    xTaskCreate(readSensorsRTOS, "SENSE", 4096, NULL, 6, NULL);
    xTaskCreate(updateAHRS, "AHRS", 4096, NULL, 5, NULL);
    xTaskCreate(handleSwitch, "SWITCH", 4096, NULL, 4, NULL);
    xTaskCreate(steerRobot, "SERVO", 4096, NULL, 3, NULL);
    xTaskCreate(fireSolenoid, "FIRE", 4096, NULL, 2, NULL);
    xTaskCreate(serialOutput, "DATA", 4096, NULL, 1, NULL);
}

void loop() { } // nothing needs to be in loop() because FreeRTOS handles all