#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/i2c.h"

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} mag_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} accel_t;

#define RAD_TO_DEG 57.295779513082320876798154814105

#define MAGNETOMETER_ADDR 0x1E // Magnetometer slave address
#define ACCELEROMETER_ADDR 0x19 // Accelerometer slave address
#define I2C_BAUD_RATE 400 // kHz

// Define i2c pins and channel
#define GPIO_SDA_PIN 26
#define GPIO_SCL_PIN 27
#define I2C_BUS &i2c1_inst

// Configuration Address for Datasheet
// Magnetometer
#define DATA_OUTPUT_RATE_CONFIG_REGISTER 0x00
#define GAIN_CONFIG_REGISTER 0x01
#define MODE_CONFIG_REGISTER 0x02
#define MAGNET_DATA_OUTPUT_X_HIGH 0x03

// Accelerometer
#define ACCELEROMETER_POWER_MODE_CONFIG_REGISTER 0x20
#define ACCEL_DATA_OUTPUT_X_LOW 0x28

// Configuration data
#define DATA_OUTPUT_RATE_15HZ_CONFIG 0x10
#define DATA_OUTPUT_RATE_220HZ_CONFIG 0x1C

#define GAIN_SETTING_1_3_GAUSS 0x20
#define GAIN_SETTING_4_0_GAUSS 0x80
#define GAIN_SETTING_8_1_GAUSS 0xE0

#define MODE_CONFIG_CONTINUOUS 0x00

// Default meaning normal power mode, and all axis enabled
#define ACCELEROMETER_DEFAULT_POWER_MODE_10_HZ 0x27
#define ACCELEROMETER_DEFAULT_POWER_MODE_50_HZ 0x47
#define ACCELEROMETER_DEFAULT_POWER_MODE_100_HZ 0x57
#define ACCELEROMETER_DEFAULT_POWER_MODE_400_HZ 0x77
#define ACCELEROMETER_DEFAULT_POWER_MODE_1344_HZ 0x97

// Define sensitivity
#define ACCEL_SENSITIVITY 0.001f // Sensitivity for +2g
#define GRAVITY 9.81

// Sensitivity for the magnetometer, for 1.3 Gauss range
#define MAG_SENSITIVITY_XY 1100
#define MAG_SENSITIVITY_Z 980

void init_i2c() {
    i2c_init(i2c_default, I2C_BAUD_RATE * 1000);
    gpio_set_function(GPIO_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(GPIO_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(GPIO_SDA_PIN);
    gpio_pull_up(GPIO_SCL_PIN);
}

void init_magnetometer() {
    uint8_t config_data_output_rate[2] = {DATA_OUTPUT_RATE_CONFIG_REGISTER, DATA_OUTPUT_RATE_15HZ_CONFIG};
    i2c_write_blocking(I2C_BUS, MAGNETOMETER_ADDR, config_data_output_rate, 2, false);

    uint8_t config_gain_setting[2] = {GAIN_CONFIG_REGISTER, GAIN_SETTING_1_3_GAUSS};
    i2c_write_blocking(I2C_BUS, MAGNETOMETER_ADDR, config_gain_setting, 2, false);
    
    uint8_t config_mode[2] = {MODE_CONFIG_REGISTER, MODE_CONFIG_CONTINUOUS};
    i2c_write_blocking(I2C_BUS, MAGNETOMETER_ADDR, config_mode, 2, false);
}

void init_accelerometer() {
    uint8_t config_power_mode[2] = {ACCELEROMETER_POWER_MODE_CONFIG_REGISTER, ACCELEROMETER_DEFAULT_POWER_MODE_100_HZ};
    i2c_write_blocking(I2C_BUS, ACCELEROMETER_ADDR, config_power_mode, 2, false);

}

// data register starts from 0x03 x high, x low, z high, z low, y high, y low
void read_magnetometer(mag_t *mag) {
    uint8_t buffer[6];
    uint8_t starting_addr = MAGNET_DATA_OUTPUT_X_HIGH | 0x80;
    i2c_write_blocking(I2C_BUS, MAGNETOMETER_ADDR, &starting_addr, 1, true);
    i2c_read_blocking(I2C_BUS, MAGNETOMETER_ADDR, buffer, 6, false);

    mag->x = (buffer[0] << 8) | buffer[1];
    mag->z = (buffer[2] << 8) | buffer[3];
    mag->y = (buffer[4] << 8) | buffer[5];
}

// data register starts from 0x28 x low, x high, y low, y high, z low, z high
void read_accelerometer(accel_t *accel) {
    uint8_t buffer[6];
    uint8_t starting_addr = ACCEL_DATA_OUTPUT_X_LOW | 0x80;
    i2c_write_blocking(I2C_BUS, ACCELEROMETER_ADDR, &starting_addr, 1, true);
    i2c_read_blocking(I2C_BUS, ACCELEROMETER_ADDR, buffer, 6, false);

    accel->x = (buffer[1] << 8) | buffer[0];
    accel->y = (buffer[3] << 8) | buffer[2];
    accel->z = (buffer[5] << 8) | buffer[4];
}

float calculate_angle(int16_t x, int16_t y) {
    float angle = 0;

    angle = atan2(y, x) * RAD_TO_DEG;

    if (angle < 0) {
        angle += 360;
    }

    return angle;
}

void convert_accel_to_mps2(accel_t *accel) {
    accel->x = accel->x / ACCEL_SENSITIVITY * GRAVITY;
    accel->y = accel->y / ACCEL_SENSITIVITY * GRAVITY;
    accel->z = accel->z / ACCEL_SENSITIVITY * GRAVITY;
}

void convert_mag_to_gauss(mag_t *mag) {
    mag->x = mag->x * MAG_SENSITIVITY_XY;
    mag->y = mag->y * MAG_SENSITIVITY_XY;
    mag->z = mag->z * MAG_SENSITIVITY_Z;
}

void calculate_tilt(float accel_x, float accel_y, float accel_z, float *roll, float *pitch) {
    *roll = atan2(accel_y, sqrt(accel_x * accel_x + accel_z * accel_z)) * RAD_TO_DEG;
    *pitch = atan2(-accel_x, sqrt(accel_y * accel_y + accel_z * accel_z)) * RAD_TO_DEG;
}

int main() {
    stdio_init_all();
    init_i2c();
    init_accelerometer();
    init_magnetometer();

    // init wifi
    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed");
        return -1;
    }

    mag_t mag;
    accel_t accel;
    float roll, pitch;

    while (true) {
        read_magnetometer(&mag);
        read_accelerometer(&accel);
        printf("Raw Accelerometer: x: %d, y: %d, z: %d\n", accel.x, accel.y, accel.z);
        calculate_tilt(accel.x, accel.y, accel.z, &roll, &pitch);
        printf("Roll: %f, Pitch: %f\n", roll, pitch);
        convert_accel_to_mps2(&accel);
        convert_mag_to_gauss(&mag);
        calculate_angle(mag.x, mag.y);

        printf("Magnetometer: x: %d, y: %d, z: %d\n", mag.x, mag.y, mag.z);
        printf("Accelerometer: x: %d, y: %d, z: %d\n", accel.x, accel.y, accel.z);
        printf("Heading: %f\n", calculate_angle(mag.x, mag.y));
        printf("------------------------------\n");

        sleep_ms(500);
    }

    return 0;
}
