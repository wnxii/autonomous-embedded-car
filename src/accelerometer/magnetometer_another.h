#ifndef MAGNETOMETER_H
#define MAGNETOMETER_H

#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// Define the I2C address of the GY-511 magnetometer (FIxed Value by Hardware, Cannot change)
#define MAGNETOMETER_ADDR 0x1E
#define ACCELEROMETER_ADDR 0x19
#define I2C_BAUD_RATE 100000 

#define I2C_CHANNEL i2c1
#define I2C_SDA_PIN 26
#define I2C_SCL_PIN 27
#define RADIANS_TO_DEGREES 57.29577 //180 devided by Pi

// Configuration Address from Datasheet
#define SAMPLE_CONFIG_ADDR 0x00
#define GAIN_CONFIG_ADDR 0x01
#define MODE_CONFIG_ADDR 0x02

// struct Magnetometer{
//     MessageBufferHandle_t* buffer;
//     TaskHandle_t* task_handler;
// };

void magnetometer_task(void* params);
void i2c_init_pins();
void magnetometer_init();
bool reading_magnetometer(int16_t* result);
float read_angle_difference(float curr_heading);
float calculate_angle(int16_t x, int16_t y);

#endif