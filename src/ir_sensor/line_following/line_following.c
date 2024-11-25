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
volatile bool autonomous_running = false; // Global variable to store status of autonomous running
bool white_detected = false;

// Threshold variables
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
    const int black_line_threshold = 50;    // Threshold for black line detection
    int black_line_count = 0;                 // Counter for black line detection
    bool is_previous_black_line = false;      // Flag to track previous black line status
    int compensate_duration = 0; // duration for compensating after kill switch, +ve left, -ve right

    xSemaphoreTake(wifiConnectedSemaphore, portMAX_DELAY);

    while (1) {
        uint16_t current_ir_value = get_line_moving_average_adc(); // Get averaged ADC value
        black_line_detected = current_ir_value >= line_contrast_threshold;

        if (black_line_detected) {
            if (is_previous_black_line) {
                black_line_count++;
                if (black_line_count >= black_line_threshold) {
                    set_autonomous_running(true);
                    xQueueSend(xServerQueue, "Current State - Autonomous Mode", portMAX_DELAY);

                    continue;
                }
            } else {
                black_line_count = 0;
            }
            // Line detected: reset search state and move forward
            move_car(FORWARD, forward_speed, forward_speed, 0.0f);
            search_attempts = 0;             // Reset search attempts
            right_cycles_remaining = right_scan_cycles; // Reset right scan cycles
            left_cycles_remaining = left_scan_cycles;   // Reset left scan cycles
            scanning_right = true;          // Start with right scan again
            compensate_duration = 0;
            
        } else {
            if (get_autonomous_running() == false)
                continue;
            // Line not detected: perform pivot search
            if (search_attempts < lost_line_threshold) {
                if (scanning_right && right_cycles_remaining > 0) {
                    // Perform a right scan
                    move_car(PIVOT_RIGHT, pivot_speed, pivot_speed, pivot_duration_ms);
                    right_cycles_remaining--;  // Decrement right scan cycles

                    if (right_cycles_remaining == 0) {
                        scanning_right = false; // Switch to left after right cycles are exhausted
                    }
                    compensate_duration += pivot_duration_ms;
                } else if (!scanning_right && left_cycles_remaining > 0) {
                    // Perform a left scan
                    move_car(PIVOT_LEFT, pivot_speed, pivot_speed, pivot_duration_ms);
                    left_cycles_remaining--;  // Decrement left scan cycles
                    compensate_duration -= pivot_duration_ms;
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
                printf("Line lost consistently. Stopping the car.\n");
                printf("[DEBUG] COMPENSATE DURATION: %d\n", compensate_duration);
                if (compensate_duration > 0) {
                    move_car(PIVOT_RIGHT, pivot_speed, pivot_speed, compensate_duration);
                }
                else if (compensate_duration < 0) {
                    move_car(PIVOT_LEFT, pivot_speed, pivot_speed, -compensate_duration);
                }

                move_car(STOP, 0.0f, 0.0f, 0.0f);
                set_autonomous_running(false);
                black_line_count = 0;
                continue;  // Exit the loop
            }
        }
        is_previous_black_line = black_line_detected;

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
    xTaskCreate(control_motor_on_line_task, "Control Motor on Line", configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 4, NULL);
}

