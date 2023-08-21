#include "stdio.h"
#include "pico/stdlib.h"
#include "haw/MPU6050.h"
#include "hardware/pwm.h"
#include <math.h>

float cal_x, cal_y, cal_z = 0;

#define RAD2DEG 57.2957795131

struct Attitude
{
    float r, p, y;
} attitude;

int DEG_45 = 490;
int DEG_0 = 690;
int DEG_NEG_45 = 890;

uint PIN_OUT_1 = 14;
uint PIN_OUT_2 = 15;
uint PIN_OUT_3 = 10;
uint PIN_OUT_4 = 11;

uint channel_1, channel_2, channel_3, channel_4;
uint slice_1, slice_2, slice_3, slice_4;
int servo_deg_1, servo_deg_2, servo_deg_3, servo_deg_4;

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

int main()
{
    stdio_init_all();

    init_servos();

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

        // mpu6050_calibrate_gyro(&mpu6050, 5);

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

        // printf("%f,%f,%f\n",cal_x, cal_y, cal_z);
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
    int slope = (DEG_NEG_45 - DEG_45) / (10 - (-10)); 
    int intercept = DEG_45 - (-10) * slope;

    // return 0;

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

        // Complementary filter
        float accelPitch = atan2(accel_y, accel_z) * RAD2DEG;
        float accelRoll = atan2(accel_x, accel_z) * RAD2DEG;

        float _tau = 0.98f;
        float _dt = 0.004f;

        // float _dt = 0.000010f; // this is a guess
        // clock_t c_time = clock();
        // int diff = (double)(c_time - t);
        // float _dt = (float)diff / 1000000.f;
        // printf("Delta Time: %f\n", _dt);

        attitude.r = _tau * (attitude.r - gyro->y * _dt) + (1.f - _tau) * accel_y;
        attitude.p = _tau * (attitude.p + gyro->x * _dt) + (1.f - _tau) * accel_x;
        attitude.y += gyro->z * _dt;

        // Print all the measurements
        // printf("%f,%f,%f\n", gyro->x, gyro->y, gyro->z);
        printf("Roll: %.4f, Pitch: %.4f, Yall: %.4f\n", attitude.r, attitude.p, attitude.y);
        
        // printf("Accel: %f, %f, %f - Gyro: %f, %f, %f - Temp: %f°C - Temp: %f°F\n", accel->x, accel->y, accel->z, gyro->x, gyro->y, gyro->z, tempC, tempF);

        // // Print all motion interrupt flags
        // printf("Overflow: %d - Freefall: %d - Inactivity: %d, Activity: %d, DataReady: %d\n",
        //        activities->isOverflow,
        //        activities->isFreefall,
        //        activities->isInactivity,
        //        activities->isActivity,
        //        activities->isDataReady);

        // // Print all motion detect interrupt flags
        // printf("PosX: %d - NegX: %d -- PosY: %d - NegY: %d -- PosZ: %d - NegZ: %d\n",
        //        activities->isPosActivityOnX,
        //        activities->isNegActivityOnX,
        //        activities->isPosActivityOnY,
        //        activities->isNegActivityOnY,
        //        activities->isPosActivityOnZ,
        //        activities->isNegActivityOnZ);

        int value = slope * attitude.r + intercept;
        int value2 = slope * (attitude.r * -1) + intercept;

        int value3 = slope * attitude.p + intercept;
        int value4 = slope * (attitude.p * -1) + intercept;


        printf("%d\n", value);
        printf("%d\n", value2);
        // move(slice_1, channel_1, value);
        // move(slice_3, channel_3, value2);

        move(slice_2, channel_2, value3);
        move(slice_4, channel_4, value4);

        sleep_ms(10);
    }

    return 0;
}