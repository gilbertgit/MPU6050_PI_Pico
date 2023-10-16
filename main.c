#include "stdio.h"
#include "pico/stdlib.h"
#include "haw/MPU6050.h"
#include "hardware/pwm.h"
#include <math.h>
#include "kalman.h"


#define RESTRICT_PITCH

float cal_x, cal_y, cal_z = 0;

#define RAD2DEG 57.2957795131

Kalman kalmanX; // Kalman filter instances
Kalman kalmanY;  

int16_t accX, accY, accZ; // Accelerometer values 
int16_t gyroX, gyroY, gyroZ; // Gyroscope values
int16_t tempRaw; // Temperature raw value

double gyroXangle, gyroYangle; // Angle from gyro integration
double compAngleX, compAngleY; // Angle from complementary filter  
double kalAngleX, kalAngleY; // Angle from Kalman filter

uint32_t timer;

// PID constants
double kp = 2.0; 
double ki = 0.5;
double kd = 0.0;

// Global variables
double motor1_pwm = 0;
double motor2_pwm = 0;
double last_error = 0;
double integral = 0;

struct Attitude
{
    float roll, pitch, yaw;
} attitude;

int DEG_45 = 490;
int DEG_0 = 690;
int DEG_NEG_45 = 890;

int MOTOR_PWM_MAX = 1000;
int MOTOR_PWM_MIN = 512;

uint PIN_OUT_1 = 14;
uint PIN_OUT_2 = 15;
uint PIN_OUT_3 = 10;
uint PIN_OUT_4 = 11;

uint channel_1, channel_2, channel_3, channel_4;
uint slice_1, slice_2, slice_3, slice_4;
int servo_deg_1, servo_deg_2, servo_deg_3, servo_deg_4;

uint THROT_PIN = 14;
uint THROT_CHANNEL, THROT_SLICE;

void init_motors() {
    gpio_set_function(THROT_PIN, GPIO_FUNC_PWM);
    THROT_SLICE = pwm_gpio_to_slice_num(THROT_PIN);
    THROT_CHANNEL = pwm_gpio_to_channel(THROT_PIN);

    pwm_set_clkdiv(THROT_SLICE, 256.0f);  /// Setting the divider to slow down the clock
    pwm_set_wrap(THROT_SLICE, 9804);      /// setting the Wrap time to 9764 (20 ms)
    pwm_set_enabled(THROT_SLICE, true);

    pwm_set_chan_level(THROT_SLICE, THROT_CHANNEL, MOTOR_PWM_MIN);
}

void init_servos() {
// Tell GPIO 14 that it's allocated to the PWM
    gpio_set_function(PIN_OUT_1, GPIO_FUNC_PWM);
    gpio_set_function(PIN_OUT_2, GPIO_FUNC_PWM);
    gpio_set_function(PIN_OUT_3, GPIO_FUNC_PWM);
    gpio_set_function(PIN_OUT_4, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to GPIO 
    slice_1 = pwm_gpio_to_slice_num(PIN_OUT_1);
    slice_2 = pwm_gpio_to_slice_num(PIN_OUT_2);
    slice_3 = pwm_gpio_to_slice_num(PIN_OUT_3);
    slice_4 = pwm_gpio_to_slice_num(PIN_OUT_4);

    channel_1 = pwm_gpio_to_channel(PIN_OUT_1);
    channel_2 = pwm_gpio_to_channel(PIN_OUT_2);
    channel_3 = pwm_gpio_to_channel(PIN_OUT_3);
    channel_4 = pwm_gpio_to_channel(PIN_OUT_4);

    pwm_set_clkdiv(slice_1, 256.0f);  /// Setting the divider to slow down the clock
    pwm_set_wrap(slice_1, 9804);      /// setting the Wrap time to 9764 (20 ms)
    pwm_set_enabled(slice_1, true);

    pwm_set_clkdiv(slice_2, 256.0f);  /// Setting the divider to slow down the clock
    pwm_set_wrap(slice_2, 9804);      /// setting the Wrap time to 9764 (20 ms)
    pwm_set_enabled(slice_2, true);

    pwm_set_clkdiv(slice_3, 256.0f);  /// Setting the divider to slow down the clock
    pwm_set_wrap(slice_3, 9804);      /// setting the Wrap time to 9764 (20 ms)
    pwm_set_enabled(slice_3, true);

    pwm_set_clkdiv(slice_4, 256.0f);  /// Setting the divider to slow down the clock
    pwm_set_wrap(slice_4, 9804);      /// setting the Wrap time to 9764 (20 ms)
    pwm_set_enabled(slice_4, true);

    pwm_set_chan_level(slice_1, channel_1, DEG_0);
    pwm_set_chan_level(slice_2, channel_2, DEG_0);
    pwm_set_chan_level(slice_3, channel_3, DEG_0);
    pwm_set_chan_level(slice_4, channel_4, DEG_0);

    servo_deg_1 = DEG_0;
    servo_deg_2 = DEG_0;
    servo_deg_3 = DEG_0;
    servo_deg_4 = DEG_0;

}

// TODO: void trim()

void move(uint slice, uint channel, int deg) {
    int max_deg = DEG_0;
    // out of range?
    if (deg < DEG_45) {
        max_deg = DEG_45;
    }
    else if (deg > DEG_NEG_45)
    {
        max_deg = DEG_NEG_45;
    }
    else {
        max_deg = deg;
    }

    if(channel == channel_1) {
        if ((servo_deg_2 > DEG_0 && deg < DEG_0) || (servo_deg_4 < DEG_0 && deg > DEG_0)) {
            max_deg = DEG_0;
        }
        servo_deg_1 = max_deg;
    }
    if (channel == channel_2) {
        if ((servo_deg_3 > DEG_0 && deg < DEG_0) || (servo_deg_1 < DEG_0 && deg > DEG_0)) {
            max_deg = DEG_0;
        }
        servo_deg_2 = max_deg;
    }
    if (channel == channel_3) {
        if ((servo_deg_4 > DEG_0 && deg < DEG_0) || (servo_deg_2 < DEG_0 && deg > DEG_0)) {
            max_deg = DEG_0;
        }
        servo_deg_3 = max_deg;
    }
    if (channel == channel_4) {
        if ((servo_deg_1 > DEG_0 && deg < DEG_0) || (servo_deg_3 < DEG_0 && deg > DEG_0)) {
            max_deg = DEG_0;
        }
        servo_deg_4 = max_deg;
    }

    pwm_set_chan_level(slice, channel, max_deg);
}

double constrain(int value, int min, int max) {
  if (value < min) return min;
  if (value > max) return max;
  return value;
}

// PID control function  
void pid_control(double target, double current) {

  // Calculate error
  double error = target - current;

  // Proportional term
  double proportional = kp * error;

  // Integral term
  
  integral = integral + (ki * error * 0.004);

  // Derivative term
  double derivative = kd * ((error) / 0.004);

  // Calculate output
  double output = proportional + integral + derivative;

  // Update last error
  last_error = error;

  // Apply output to motor
//   motor1_pwm += output;
//   motor2_pwm -= output;
  motor2_pwm = output;

  // Constrain PWM
//   motor1_pwm = constrain(motor1_pwm, -8, 8);
//   motor2_pwm = constrain(motor2_pwm, -8, 8);

}

int main()
{
    stdio_init_all();

    init_servos();
    init_motors();

    // Setup I2C properly
    gpio_init(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_init(PICO_DEFAULT_I2C_SCL_PIN);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    // Don't forget the pull ups! | Or use external ones
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    // Pass in the I2C driver (Important for dual-core operations). The second parameter is the address,
    // which can change if you connect pin A0 to GND or to VCC.
    mpu6050_t mpu6050 = mpu6050_init(i2c_default, MPU6050_ADDRESS_A0_GND);

    // Check if the MPU6050 can initialize
    if (mpu6050_begin(&mpu6050))
    {
        // Set scale of gyroscope
        mpu6050_set_scale(&mpu6050, MPU6050_SCALE_2000DPS);
        // Set range of accelerometer
        mpu6050_set_range(&mpu6050, MPU6050_RANGE_2G);

        // Enable temperature, gyroscope and accelerometer readings
        mpu6050_set_temperature_measuring(&mpu6050, true);
        mpu6050_set_gyroscope_measuring(&mpu6050, true);
        mpu6050_set_accelerometer_measuring(&mpu6050, true);

        // Enable free fall, motion and zero motion interrupt flags
        mpu6050_set_int_free_fall(&mpu6050, false);
        mpu6050_set_int_motion(&mpu6050, false);
        mpu6050_set_int_zero_motion(&mpu6050, false);

        // Set motion detection threshold and duration
        mpu6050_set_motion_detection_threshold(&mpu6050, 2);
        mpu6050_set_motion_detection_duration(&mpu6050, 5);

        // Set zero motion detection threshold and duration
        mpu6050_set_zero_motion_detection_threshold(&mpu6050, 4);
        mpu6050_set_zero_motion_detection_duration(&mpu6050, 2);

        // Wait for sensor to stabilize
        sleep_ms(100);

        kalman_init(&kalmanX);
        kalman_init(&kalmanY);

        // Set kalman starting angle
        double roll = 0; 
        double pitch = 0;
        kalman_setAngle(&kalmanX, roll);
        kalman_setAngle(&kalmanY, pitch);


        timer = get_absolute_time();

        // Calibrate accel
        for(int samples = 0; samples < 2000; samples++) {
            mpu6050_event(&mpu6050);
            mpu6050_vectorf_t *accel = mpu6050_get_accelerometer(&mpu6050);

            cal_x += accel->x;
            cal_y += accel->y;
            cal_z += accel->z;

            // printf("%f,%f,%f\n",accel->x, accel->y, accel->z);
            sleep_ms(1);
        }
        // printf("CALIBRATION ----\n");
        // printf("%f,%f,%f\n",cal_x, cal_y, cal_z);

        cal_x = cal_x / 2000.0f;
        cal_y = cal_y / 2000.0f;
        cal_z = cal_z / 2000.0f;

        mpu6050_calibrate_gyro(&mpu6050, 255); // 255 sample on gyro calibration

    }
    else
    {
        while (1)
        {
            // Endless loop
            printf("Error! MPU6050 could not be initialized. Make sure you've entered the correct address. And double check your connections.");
            sleep_ms(500);
        }
    }

    int i;
  
    // Calculate slope and intercept
    int motor_slope = (MOTOR_PWM_MIN - MOTOR_PWM_MAX) / (24 - (-24)); 
    int motor_intercept = MOTOR_PWM_MAX - (-24) * motor_slope;
    

    sleep_ms(5000);

    while (1)
    {
        // Fetch all data from the sensor | I2C is only used here
        mpu6050_event(&mpu6050);

        // Pointers to float vectors with all the results
        mpu6050_vectorf_t *accel = mpu6050_get_accelerometer(&mpu6050);
        mpu6050_vectorf_t *gyro = mpu6050_get_gyroscope(&mpu6050);

        // Activity struct holding all interrupt flags
        mpu6050_activity_t *activities = mpu6050_read_activities(&mpu6050);

        // Rough temperatures as float -- Keep in mind, this is not a temperature sensor!!!
        float tempC = mpu6050_get_temperature_c(&mpu6050);
        float tempF = mpu6050_get_temperature_f(&mpu6050);

        float accel_x = accel->x - cal_x;
        float accel_y = accel->y - cal_y;
        float accel_z = accel->z - cal_z;

        ///////////IMPORTANT - DO NOT LOOSE ////////////
        // Complementary filter
        // TODO: Not getting filter to work with atan2
        // float accelPitch = atan2(accel_y, accel_z) * RAD2DEG;
        // float accelRoll = atan2(accel_x, accel_z) * RAD2DEG;

        float _tau = 0.98f;
        float _dt = 0.004f;

        // TODO: Get real delta time
        
        attitude.pitch = _tau * (attitude.pitch + gyro->x * _dt) + 0.02 * accel_x;
        attitude.roll = _tau * (attitude.roll - gyro->y * _dt) + 0.02 * accel_y;
        attitude.yaw += gyro->z * _dt;
        /////////////////////////////////////////////


        /////////////////////////////////////////////
        // Kalman filter
        // Convert gyro values to deg/s
        double gyroXrate = gyro->x / 131.0;  
        double gyroYrate = gyro->y / 131.0; 

        // Calculate angles
        double dt = 0.004; // 400Hz
        double roll, pitch;

        #ifdef RESTRICT_PITCH
            roll = atan2(accel_y, accel_z) * RAD2DEG;
            pitch = atan(-accel_x / sqrt(accel_y * accel_y + accel_z * accel_z)) * RAD2DEG;  
        #else
            roll = atan(accY / sqrt(accel_x * accel_x + accel_z * accel_z)) * RAD2DEG;
            pitch = atan2(-accel_x, accel_z) * RAD2DEG;
        #endif

        kalAngleX = kalman_getAngle(&kalmanX, roll, gyroXrate, dt);
        kalAngleY = kalman_getAngle(&kalmanY, pitch, gyroYrate, dt);

        gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
        gyroYangle += gyroYrate * dt;

        // Reset the gyro angle when it has drifted too much
        if (gyroXangle < -180 || gyroXangle > 180)
            gyroXangle = kalAngleX;
        if (gyroYangle < -180 || gyroYangle > 180)
            gyroYangle = kalAngleY;

        /////////////////////////////////////////////

        pid_control(0.0, attitude.pitch);

        // get scalled value and contrain it
        int value = motor_slope * motor2_pwm + motor_intercept;
        int constrained_value = constrain(value, 550, MOTOR_PWM_MAX);

        // send PWM values
        pwm_set_chan_level(THROT_SLICE, THROT_CHANNEL, constrained_value);

        // Print all the measurements
        printf(">PID_Pitch:%.4f\n", motor2_pwm);
        printf(">PWM-2:%d\n", constrained_value);
        printf(">C-Pitch:%.4f\n", attitude.pitch);

        sleep_ms(10);
    }

    return 0;
}