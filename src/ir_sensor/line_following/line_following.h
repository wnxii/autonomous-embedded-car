#ifndef line_following_H
#define line_following_H

// Define GPIO pin for barcode IR sensor
#define LINE_SENSOR_PIN 28  // Using ADC for analog input

// Barcode detection threshold and time delay
#define MOVING_AVG_WINDOW 2         // Define window size for moving average

void init_line_sensor();

// Global values to be used in main()
extern volatile bool black_line_detected; // Flag to indicate if the black line is detected
extern volatile int stop_running; // Store count of cycles that black line have not been detected.
extern volatile bool autonomous_running; // Flag to indicate autonomous status when black line is detected

#endif