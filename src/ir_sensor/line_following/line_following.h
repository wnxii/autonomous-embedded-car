#ifndef line_following_H
#define line_following_H

// Define GPIO pin for barcode IR sensor
#define LINE_SENSOR_PIN 28  // Using ADC for analog input

// Barcode detection threshold and time delay
#define MOVING_AVG_WINDOW 2         // Define window size for moving average
#define CONTRAST_THRESHOLD 1.4f     // Contrast threshold in volts for detecting black vs. white

void init_line_sensor();

extern volatile bool black_line_detected; // Flag to indicate if the line is detected

#endif