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
#include "../wifi/client_server_socket/client_server_socket.h"


uint16_t line_adc_buffer[MOVING_AVG_WINDOW] = {0};  // ADC buffer for moving average
uint8_t line_buffer_index = 0;

// Mutex for thread-safe access to shared variables
SemaphoreHandle_t black_line_mutex;
SemaphoreHandle_t autonomous_running_mutex;

volatile bool black_line_detected = false; // Global variable to store status of black line detection
// volatile int stop_running = 0; // Global variable to store count of cycles that black line have not been detected
volatile bool autonomous_running = false; // Global variable to store status of autonomous running
bool white_detected = false;

// Threshold variables
// static uint32_t min_line_threshold = 4095;
// static uint32_t max_line_threshold = 0;
static uint32_t line_contrast_threshold = 1400;  // Initial threshold (mid-range)


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

/* void update_line_threshold(uint16_t adc_reading) {
    // Update min and max thresholds
    if (adc_reading < min_line_threshold) {
        min_line_threshold = adc_reading;
    }
    if (adc_reading > max_line_threshold) {
        max_line_threshold = adc_reading;
    }

    char message[100];

    // Recalculate contrast threshold as the average of min and max
    line_contrast_threshold = (min_line_threshold + max_line_threshold) / 2;
    snprintf(message, sizeof(message), "[DEBUG] Line Following - Contrast Threshold: %u\n", line_contrast_threshold);
    xQueueSend(xServerQueue, &message, portMAX_DELAY);

    printf("Contrast Threshold: %u\n", line_contrast_threshold);
} */

// Helper functions to safely access and modify shared variables
/* bool get_black_line_detected() {
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
    char message[100];
    snprintf(message, sizeof(message), "[DEBUG] Line Following - Black Line Detected: %d \n", black_line_detected);
    xQueueSend(xServerQueue, &message, portMAX_DELAY);
    // printf("Black Line Detected: %d \n", black_line_detected);
} */

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

/* void vLineFollowingTask(void *pvParameters) {
    while (1) {
        // adc_select_input(1);
        // uint16_t current_ir_value = adc_read(); // Get averaged ADC value
        uint16_t current_ir_value = get_line_moving_average_adc(); // Get averaged ADC value
        update_line_threshold(current_ir_value);
        char message[100];
        // snprintf(message, sizeof(message), "[DEBUG] Line Following - Current IR Value: %d \n", current_ir_value);
        // xQueueSend(xServerQueue, &message, portMAX_DELAY);
        // printf("Current IR Value: %d \n", current_ir_value);

        // Check if a line is detected and update the global flag
        black_line_detected = current_ir_value >= line_contrast_threshold;
        // set_black_line_detected(current_ir_value >= line_contrast_threshold);
        
        // Delay to allow other tasks to execute and reduce reading frequency
        vTaskDelay(10);  // Adjust delay as needed
    }
} */

/* void control_motor_on_line_task(void *pvParameters) {
    const int max_turn_speed = 30;  // Maximum speed for turning
    const int base_speed = 1;     // Base speed for forward movement
    // const int forward_speed = 20;

    while (1) {
        if (black_line_detected) {
            // Black line detected, turn right
            printf("Black line detected, turning right.\n");
            move_car(STEER_FORWARD_RIGHT, base_speed, max_turn_speed, 0.0);
        } else {
            // White space detected, turn left
            printf("White space detected, turning left.\n");
            move_car(STOP, 0.0, 0.0, 0.0);
            move_car(STEER_FORWARD_LEFT, max_turn_speed, base_speed, 0.0);
        }

        // move_car(FORWARD, forward_speed, forward_speed, 0);

        // Short delay to allow for sensor updates
        vTaskDelay(10);
    }
} */

/* void control_motor_on_line_task(void *pvParameters) {
    const int lost_line_threshold = 1000;
    const int max_turn_speed = 20;
    const int forward_speed = 20;
    const int micro_turn_speed = 1;
    int search_attempts = 0;
    bool black_line_detected = false;
    bool was_on_black_line = false; // New flag to track if we were previously on black line
    char message[100];
    
    enum SearchState {
        FORWARD,
        MICRO_RIGHT,
        COMPENSATE_LEFT,
        SEARCH_LEFT
    } state = FORWARD;
    
    TickType_t adjustment_start_time = 0;
    const int micro_adjust_duration = 500;
    const int left_compensate_duration = 1000;

    while (1) {
        uint16_t current_ir_value = get_line_moving_average_adc(); // Get averaged ADC value
        update_line_threshold(current_ir_value);
        // snprintf(message, sizeof(message), "[DEBUG] Line Following - Current IR Value: %d \n", current_ir_value);
        // xQueueSend(xServerQueue, &message, portMAX_DELAY);
        // printf("Current IR Value: %d \n", current_ir_value);

        // Check if a line is detected and update the global flag
        black_line_detected = current_ir_value >= line_contrast_threshold;
        snprintf(message, sizeof(message), "[DEBUG] Line Following - Black Line Detected: %d \n", black_line_detected);
        xQueueSend(xServerQueue, &message, portMAX_DELAY);
        printf("[DEBUG] Line Following - Black Line Detected: %d \n", black_line_detected);

        if (black_line_detected) {
            // On black line - move forward
            set_autonomous_running(true);
            search_attempts = 0;
            state = FORWARD;
            was_on_black_line = true; // Set flag when on black line
            move_car(FORWARD, forward_speed, forward_speed, 0);
            printf("[DEBUG] Line Following - Move Forward Black Line \n");
        } else if (get_autonomous_running()) {
            // White detected
            if (was_on_black_line) {
                // Just left black line, reset to initial search state
                state = MICRO_RIGHT;
                adjustment_start_time = xTaskGetTickCount();
                was_on_black_line = false;
                // snprintf(message, sizeof(message), "[DEBUG] Line Following - Initial white detected, starting search pattern \n");
                // xQueueSend(xServerQueue, &message, portMAX_DELAY);
                // printf("Left black line - starting new search pattern\n");
            }

            switch (state) {
                case FORWARD:
                    // Initialize micro left turn
                    state = MICRO_RIGHT;
                    adjustment_start_time = xTaskGetTickCount();
                    // printf("White detected - micro adjusting left\n");
                    break;

                case MICRO_RIGHT:
                    // Perform micro left turn
                    move_car(STEER_FORWARD_RIGHT, micro_turn_speed, max_turn_speed, 0.0);
                    if (xTaskGetTickCount() - adjustment_start_time >= pdMS_TO_TICKS(micro_adjust_duration)) {
                        if (black_line_detected) {
                            state = FORWARD;
                        } else {
                            state = COMPENSATE_LEFT;
                            adjustment_start_time = xTaskGetTickCount();
                        }
                    }
                    break;

                case COMPENSATE_LEFT:
                    // Turn right double the amount
                    move_car(STEER_FORWARD_LEFT, max_turn_speed, micro_turn_speed, 0.0);
                    if (xTaskGetTickCount() - adjustment_start_time >= pdMS_TO_TICKS(left_compensate_duration)) {
                        if (!black_line_detected) {
                            state = SEARCH_LEFT;
                            search_attempts++;
                        }
                    }
                    break;

                case SEARCH_LEFT:
                    // Continue turning right to search
                    move_car(STEER_FORWARD_LEFT, max_turn_speed, 0, 0.0);
                    if (search_attempts >= lost_line_threshold) {
                        set_autonomous_running(false);
                        move_car(STOP, 0.0, 0.0, 0.0);
                        printf("Line lost completely. Stopping.\n");
                    }
                    break;
            }
        } else {
            move_car(STOP, 0.0, 0.0, 0.0);
        }

        // vTaskDelay(10);
    }
} */


/* void control_motor_on_line_task(void *pvParameters) {
    const int lost_line_threshold = 1000; // Number of cycles without line detection before stopping
    int search_direction = 1;             // 1 for left, -1 for right
    const int forward_speed = 15;         // Default forward speed
    const int max_angle = 45;             // Maximum search angle (degrees)
    int search_attempts = 0;              // Counter for search attempts
    int turn_angle = 5;                   // Starting angle for turning
    const int turn_increment = 5;         // Increment for each search attempt
    int turn_time_ms = 100;               // Time in milliseconds for each turn direction
    TickType_t last_turn_time = xTaskGetTickCount(); // Track the last turn time

    while (1) {
        if (get_black_line_detected()) {
            // Line is detected; enable autonomous running
            set_autonomous_running(true);
            search_attempts = 0;            // Reset search attempts
            turn_angle = 5;                 // Reset angle to initial value

            // MOVE FORWARD to stay on the line
            move_car(FORWARD, forward_speed, forward_speed, 0);
            printf("Black line detected. Moving forward.\n");
        } else {
            if (get_autonomous_running()) {
                // Line not detected while in autonomous mode
                search_attempts++;

                // Gradually increase the turn angle with each attempt
                // turn_angle = search_attempts * turn_increment);
                if (turn_angle > max_angle) {
                    turn_angle = max_angle; // Cap the angle to the maximum value
                }

                if (!get_black_line_detected()) { // Continue turning only if black line is still not detected
                    if (search_direction == 1) {
                        // STEER LEFT with increasing angle
                        move_car(PIVOT_LEFT, 0.0, 0.0, turn_angle);
                        printf("Pivoting left. Angle: %d\n", turn_angle);
                    } else {
                        // STEER RIGHT with increasing angle
                        move_car(PIVOT_RIGHT, 0.0, 0.0, turn_angle + 5);
                        printf("Pivoting right. Angle: %d\n", turn_angle + 5);
                    }

                    // Switch direction after `turn_time_ms`
                    if ((xTaskGetTickCount() - last_turn_time) >= pdMS_TO_TICKS(turn_time_ms)) {
                        search_direction = -search_direction;
                        last_turn_time = xTaskGetTickCount();
                        printf("Switching search direction.\n");
                    }
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
} */


/* void control_motor_on_line_task(void *pvParameters) {
    const int lost_line_threshold = 10000;  // Number of cycles without line detection before stopping
    const float forward_speed = 25.0f;      // Speed for forward motion
    const float pivot_speed = 0.0f;        // Speed for pivot motion
    // const int pivot_duration_ms = 150;      // Duration for each pivot turn in milliseconds
    int search_attempts = 0;                // Counter for search attempts
    bool searching_right = true;            // Keep track of the current pivot direction

    while (1) {
        uint16_t current_ir_value = get_line_moving_average_adc(); // Get averaged ADC value
        // update_line_threshold(current_ir_value);
        black_line_detected = current_ir_value >= line_contrast_threshold;

        printf("[DEBUG] Line Following - Black Line Detected: %d \n", black_line_detected);

        if (black_line_detected) {
            // Line detected, reset search state and move forward
            printf("Black line detected. Moving forward.\n");
            move_car(FORWARD, forward_speed, forward_speed, 0.0f);
            search_attempts = 0;           // Reset search attempts
            searching_right = true;        // Reset to default search direction
        } else {
            // Line not detected, perform pivot search
            if (search_attempts < lost_line_threshold) {
                if (searching_right) {
                    printf("Searching: Pivot right for %d ms.\n", 50);
                    move_car(STOP, 0.0, 0.0, 0.0);
                    move_car(PIVOT_RIGHT, pivot_speed, pivot_speed, 50);
                } else {
                    printf("Searching: Pivot left for %d ms.\n", 50);
                    move_car(STOP, 0.0, 0.0, 0.0);
                    move_car(PIVOT_LEFT, pivot_speed, pivot_speed, 50);
                }

                // Alternate search direction
                searching_right = !searching_right;

                // Increment search attempts
                search_attempts++;
            } else {
                // If the line is consistently lost, stop the car
                printf("Line lost consistently. Stopping the car.\n");
                move_car(STOP, 0.0f, 0.0f, 0.0f);
                break;  // Exit the loop
            }
        }

        // vTaskDelay(pdMS_TO_TICKS(10));  // Short delay for responsive sensor readings
    }
} */

/* void control_motor_on_line_task(void *pvParameters) {
    const int lost_line_threshold = 10000;   // Total cycles before stopping the car
    const int direction_cycle_limit = 5;  // Number of cycles in one direction before switching
    const float forward_speed = 25.0f;      // Speed for forward motion
    const float pivot_speed = 15.0f;        // Speed for pivot motion
    const int pivot_duration_ms = 15;       // Duration for each pivot turn in milliseconds
    int search_attempts = 0;                // Total search attempts
    int direction_cycles = 0;               // Cycles in the current direction
    bool searching_right = true;            // Current search direction

    while (1) {
        uint16_t current_ir_value = get_line_moving_average_adc(); // Get averaged ADC value
        black_line_detected = current_ir_value >= line_contrast_threshold;

        printf("[DEBUG] Line Following - Black Line Detected: %d \n", black_line_detected);

        if (black_line_detected) {
            // Line detected: reset search state and move forward
            printf("Black line detected. Moving forward.\n");
            move_car(FORWARD, forward_speed, forward_speed, 0.0f);
            search_attempts = 0;           // Reset search attempts
            direction_cycles = 0;          // Reset direction cycle count
            searching_right = true;        // Reset to default search direction
        } else {
            // Line not detected: perform pivot search
            if (search_attempts < lost_line_threshold) {
                if (direction_cycles < direction_cycle_limit) {
                    printf("Searching: Pivot %s for %d ms.\n",
                           searching_right ? "right" : "left", pivot_duration_ms);

                    move_car(STOP, 0.0, 0.0, 0.0);  // Stop before pivoting
                    if (searching_right) {
                        move_car(PIVOT_RIGHT, pivot_speed, pivot_speed, pivot_duration_ms);
                    } else {
                        move_car(PIVOT_LEFT, pivot_speed, pivot_speed, pivot_duration_ms);
                    }

                    direction_cycles++;  // Increment cycles in the current direction
                } else {
                    // Switch direction after reaching cycle limit
                    printf("Switching direction to %s.\n", searching_right ? "left" : "right");
                    searching_right = !searching_right;
                    direction_cycles = 0;  // Reset direction cycle count
                }

                // Increment search attempts
                search_attempts++;
            } else {
                // Line consistently lost: stop the car
                printf("Line lost consistently. Stopping the car.\n");
                move_car(STOP, 0.0f, 0.0f, 0.0f);
                break;  // Exit the loop
            }
        }

        // Small delay for responsive sensor readings
        vTaskDelay(pdMS_TO_TICKS(10));
    }
} */

void control_motor_on_line_task(void *pvParameters) {
    const int lost_line_threshold = 200;       // Total cycles before stopping the car
    const int right_scan_cycles = 5;          // Number of cycles to scan to the right
    const int left_scan_cycles = right_scan_cycles * 2; // Double the cycles for left scan
    const float forward_speed = 25.0f;        // Speed for forward motion
    const float pivot_speed = 0.0f;           // Speed for pivot motion
    const int pivot_duration_ms = 20;         // Duration for each pivot turn in milliseconds
    int search_attempts = 0;                  // Total search attempts
    int right_cycles_remaining = right_scan_cycles; // Remaining cycles to scan right
    int left_cycles_remaining = left_scan_cycles;   // Remaining cycles to scan left
    bool scanning_right = true;               // Flag to prioritize right scanning first

    xSemaphoreTake(wifiConnectedSemaphore, portMAX_DELAY);

    while (1) {
        uint16_t current_ir_value = get_line_moving_average_adc(); // Get averaged ADC value
        black_line_detected = current_ir_value >= line_contrast_threshold;

        // printf("[DEBUG] Line Following - Black Line Detected: %d \n", black_line_detected);

        if (black_line_detected) {
            // Line detected: reset search state and move forward
            // printf("Black line detected. Moving forward.\n");
            move_car(FORWARD, forward_speed, forward_speed, 0.0f);
            search_attempts = 0;             // Reset search attempts
            right_cycles_remaining = right_scan_cycles; // Reset right scan cycles
            left_cycles_remaining = left_scan_cycles;   // Reset left scan cycles
            scanning_right = true;          // Start with right scan again
        } else {
            // Line not detected: perform pivot search
            if (search_attempts < lost_line_threshold) {
                if (scanning_right && right_cycles_remaining > 0) {
                    // Perform a right scan
                    // printf("Searching: Pivot right for %d ms. Remaining right cycles: %d\n", pivot_duration_ms, right_cycles_remaining);
                    move_car(PIVOT_RIGHT, pivot_speed, pivot_speed, pivot_duration_ms);
                    right_cycles_remaining--;  // Decrement right scan cycles

                    if (right_cycles_remaining == 0) {
                        scanning_right = false; // Switch to left after right cycles are exhausted
                    }
                } else if (!scanning_right && left_cycles_remaining > 0) {
                    // Perform a left scan
                    // printf("Searching: Pivot left for %d ms. Remaining left cycles: %d\n", pivot_duration_ms, left_cycles_remaining);
                    move_car(PIVOT_LEFT, pivot_speed, pivot_speed, pivot_duration_ms);
                    left_cycles_remaining--;  // Decrement left scan cycles
                } else {
                    // Reset right and left scan cycles if both are exhausted
                    right_cycles_remaining = right_scan_cycles;
                    left_cycles_remaining = left_scan_cycles;
                    scanning_right = true; // Restart with right scanning
                }

                // Increment search attempts
                search_attempts++;
            } else {
                // Line consistently lost: stop the car
                // printf("Line lost consistently. Stopping the car.\n");
                move_car(STOP, 0.0f, 0.0f, 0.0f);
                continue;  // Exit the loop
            }
        }

        // Small delay for responsive sensor readings
        taskYIELD();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}




// Function to initialize the ADC for Line Following
void init_line_sensor() {
    // Create mutexes for shared variables
    black_line_mutex = xSemaphoreCreateMutex();
    autonomous_running_mutex = xSemaphoreCreateMutex();

    adc_init();
    adc_gpio_init(LINE_SENSOR_PIN);

    // xTaskCreate(vLineFollowingTask, "Line Following Task", configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 3, NULL);
    xTaskCreate(control_motor_on_line_task, "Control Motor on Line", configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 3, NULL);
}

