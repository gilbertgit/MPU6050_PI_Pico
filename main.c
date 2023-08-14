#include "stdio.h"
#include "pico/stdlib.h"
#include "haw/MPU6050.h"
#include <math.h>

float cal_x, cal_y, cal_z = 0;

int main()
{
    stdio_init_all();

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
        mpu6050_set_range(&mpu6050, MPU6050_RANGE_16G);

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

        float x = accel->x;
        float y = accel->y;
        float z = accel->z;

        x = x - cal_x;
        y = y - cal_y;
        z = z - cal_z;

        // Print all the measurements
        // printf("%f,%f,%f\n", gyro->x, gyro->y, gyro->z);
        printf("%f,%f,%f\n", x, y, z);
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

        sleep_ms(100);
    }

    return 0;
}