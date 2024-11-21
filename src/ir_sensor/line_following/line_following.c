#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "line_following.h"
#include "queue.h"
#include "semphr.h"
#include <stdbool.h>
#include "../motor/motor.h"
#include "../barcode_scanner/barcode_scanner.h"

uint16_t line_adc_buffer[MOVING_AVG_WINDOW] = {0};  // ADC buffer for moving average
uint8_t line_buffer_index = 0;

// Queue handle for message passing
QueueHandle_t xDisplayQueue;

// Mutex for thread-safe access to shared variables
SemaphoreHandle_t black_line_mutex;
SemaphoreHandle_t autonomous_running_mutex;

volatile bool black_line_detected = false; // Global variable to store status of black line detection
// volatile int stop_running = 0; // Global variable to store count of cycles that black line have not been detected
volatile bool autonomous_running = false; // Global variable to store status of autonomous running
bool white_detected = false;

// Threshold variables
static uint32_t min_line_threshold = 4095;
static uint32_t max_line_threshold = 0;
static uint32_t line_contrast_threshold = 2048;  // Initial threshold (mid-range)


// Moving average for ADC readings
float get_line_moving_average_adc() {
    adc_select_input(1);
    uint16_t adc_value = adc_read();
    line_adc_buffer[line_buffer_index] = adc_value;
    line_buffer_index = (line_buffer_index + 1) % MOVING_AVG_WINDOW;

    // Calculate the average
    uint32_t sum = 0;
    for (int i = 0; i < MOVING_AVG_WINDOW; i++) {
        sum += line_adc_buffer[i];
    }
    return (uint16_t)(sum / MOVING_AVG_WINDOW); 
} 

void update_line_threshold(uint16_t adc_reading) {
    // Update min and max thresholds
    if (adc_reading < min_line_threshold) {
        min_line_threshold = adc_reading;
    }
    if (adc_reading > max_line_threshold) {
        max_line_threshold = adc_reading;
    }

    // Recalculate contrast threshold as the average of min and max
    line_contrast_threshold = (min_line_threshold + max_line_threshold) / 2;
    printf("Contrast Threshold: %u\n", line_contrast_threshold);
}

// Helper functions to safely access and modify shared variables
bool get_black_line_detected() {
    bool detected;
    xSemaphoreTake(black_line_mutex, portMAX_DELAY);
    detected = black_line_detected;
    xSemaphoreGive(black_line_mutex);
    return detected;
}

void set_black_line_detected(bool detected) {
    xSemaphoreTake(black_line_mutex, portMAX_DELAY);
    black_line_detected = detected;
    xSemaphoreGive(black_line_mutex);
    printf("Black Line Detected: %d \n", black_line_detected);
}

bool get_autonomous_running() {
    bool running;
    xSemaphoreTake(autonomous_running_mutex, portMAX_DELAY);
    running = autonomous_running;
    xSemaphoreGive(autonomous_running_mutex);
    return running;
}

void set_autonomous_running(bool running) {
    xSemaphoreTake(autonomous_running_mutex, portMAX_DELAY);
    autonomous_running = running;
    xSemaphoreGive(autonomous_running_mutex);
}

void vLineFollowingTask(void *pvParameters) {
    while (1) {
        // adc_select_input(1);
        // uint16_t current_ir_value = adc_read(); // Get averaged ADC value
        uint16_t current_ir_value = get_line_moving_average_adc(); // Get averaged ADC value
        update_line_threshold(current_ir_value);
        printf("Current IR Value: %d \n", current_ir_value);

        // Check if a line is detected and update the global flag
        // black_line_detected = current_ir_value >= line_contrast_threshold;
        set_black_line_detected(current_ir_value >= line_contrast_threshold);
        
        // Delay to allow other tasks to execute and reduce reading frequency
        vTaskDelay(pdMS_TO_TICKS(5));  // Adjust delay as needed
    }
}

void control_motor_on_line_task(void *pvParameters) {
    const int lost_line_threshold = 1000; // Number of cycles without line detection before stopping
    int search_direction = 1; // 1 for left, -1 for right
    const int max_turn_speed = 15;       // Maximum speed for turning
    const int base_turn_speed = 3;       // Base speed for gradual turns
    const int forward_speed = 15;        // Default forward speed
    int search_attempts = 0;
    int turn_time_ms = 100;               // Time in milliseconds for each turn direction
    TickType_t last_turn_time = xTaskGetTickCount(); // Track the last turn time


    while (1) {
        if (get_black_line_detected()) {
            // Line is detected; enable autonomous running
            set_autonomous_running(true);
            // stop_running = 0; // Reset the stop_running counter
            search_attempts = 0; // Reset search attempts

            // MOVE FORWARD to stay on the line
            move_car(FORWARD, forward_speed, forward_speed, 0);
            printf("Black line detected. Moving forward.\n");
        } else {
            if (get_autonomous_running()) {
                // Line not detected while in autonomous mode
                search_attempts++;

                // Gradual search pattern
                if (search_direction == 1) {
                    // STEER LEFT to search for the line
                    move_car(STEER_FORWARD_LEFT, base_turn_speed, max_turn_speed, 0.0);
                } else {
                    // STEER RIGHT to search for the line
                    move_car(STEER_FORWARD_RIGHT, max_turn_speed, base_turn_speed-2, 0.0);
                }


               // Switch direction after `turn_time_ms`
                if ((xTaskGetTickCount() - last_turn_time) >= pdMS_TO_TICKS(turn_time_ms)) {
                    search_direction = -search_direction;
                    last_turn_time = xTaskGetTickCount();
                    printf("Switching search direction.\n");
                }

                // Check if the threshold is reached
                if (search_attempts >= lost_line_threshold) {
                    set_autonomous_running(false);
                    move_car(STOP, 0.0, 0.0, 0.0); // Stop the car
                    printf("Line lost consistently. Stopping the car.\n");
                }
            } else {
                // Ensure the car remains stopped if autonomous mode is inactive
                move_car(STOP, 0.0, 0.0, 0.0);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(5)); // Short delay for responsive line checking
    }
} 

/* void control_motor_on_line_task(void *pvParameters) {
    const int lost_line_threshold = 1000;  // Number of cycles without line detection before stopping
    const int forward_speed = 20;          // Default forward speed
    const int max_turn_speed = 25;         // Maximum speed for turning
    const int base_turn_speed = 0;         // Base speed for gradual turns
    int search_attempts = 0;               // Counter for search attempts

    while (1) {
        // Read sensor states
        bool line_detected = get_black_line_detected();  // Line-following sensor detects black
        bool barcode_sensor_white = get_barcode_moving_average_adc() < line_contrast_threshold;  // Barcode sensor detects white
        bool line_following_sensor_white = !line_detected;  // Line-following sensor detects white

        if (line_detected) {
            // Line detected by line-following sensor, steer right
            printf("Line detected. Steering right.\n");
            move_car(STEER_FORWARD_RIGHT, max_turn_speed, base_turn_speed, 0.0);
        } else if ((line_following_sensor_white && barcode_sensor_white) || (line_following_sensor_white && scan_started)) {
            // Both sensors detect white, move forward
            printf("Both sensors detect white. Moving forward.\n");
            move_car(FORWARD, forward_speed, forward_speed, 0);
        } else {
            // Neither condition satisfied, continue searching
            printf("Searching for white on both sensors...\n");
            search_attempts++;
            move_car(STEER_FORWARD_LEFT, base_turn_speed, max_turn_speed, 0.0);
        }

        // Check if the threshold is reached
        if (search_attempts >= lost_line_threshold) {
            printf("Line lost consistently. Stopping the car.\n");
            move_car(STOP, 0.0, 0.0, 0.0);
            break;  // Stop the loop
        }

        vTaskDelay(pdMS_TO_TICKS(5));  // Short delay for responsive sensor readings
    }
} */



// Function to initialize the ADC for Line Following
void init_line_sensor() {
    // Create mutexes for shared variables
    black_line_mutex = xSemaphoreCreateMutex();
    autonomous_running_mutex = xSemaphoreCreateMutex();

    adc_init();
    adc_gpio_init(LINE_SENSOR_PIN);

    xTaskCreate(vLineFollowingTask, "Line Following Task", configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(control_motor_on_line_task, "Control Motor on Line", configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 1, NULL);
}

