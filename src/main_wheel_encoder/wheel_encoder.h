#ifndef WHEEL_ENCODER_H
#define WHEEL_ENCODER_H

#include <stdint.h>

// Initializes the wheel encoder and sets up the necessary GPIO and FreeRTOS tasks.
void init_wheel_encoder();

// Returns the total distance traveled based on encoder pulses.
float get_distance();

// Returns the current speed of the wheel based on encoder data.
float get_speed();

// Resets the encoder data (e.g., pulse count and timestamp).
void reset_encoder();

#endif
