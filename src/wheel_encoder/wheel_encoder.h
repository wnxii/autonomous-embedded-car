#ifndef WHEEL_ENCODER_H
#define WHEEL_ENCODER_H

#include <stdint.h>

// Pin definitions
#define LEFT_ENCODER_PIN 4 // GPIO pin for the encoder
#define RIGHT_ENCODER_PIN 5 // GPIO pin for the encoder

#define WHEEL_CIRCUMFERENCE 20.0f  // Wheel circumference in cm
#define PULSES_PER_REVOLUTION 40  // Number of pulses per wheel revolution
// #define ENCODER_CIRCUMFERENCE 8.5
// #define SCALING_FACTOR (WHEEL_CIRCUMFERENCE / ENCODER_CIRCUMFERENCE)

// Encoder data structure
typedef struct {
    uint32_t pulse_count;
    uint64_t timestamp;
} EncoderData;

extern volatile EncoderData left_data;
extern volatile EncoderData right_data;

// Mutexes for each encoder
extern SemaphoreHandle_t left_data_mutex;
extern SemaphoreHandle_t right_data_mutex;

// Encoder and GPIO initializations
void init_wheel_encoders();

// Distance measurement functions for each encoder
float get_left_distance();
float get_right_distance();

// Speed measurement functions for each encoder
float get_left_speed();
float get_right_speed();

// Encoder reset functions for each encoder
void reset_left_encoder();
void reset_right_encoder();

#endif
