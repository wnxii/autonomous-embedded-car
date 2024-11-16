#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "line_following.h"
#include "queue.h"
#include <stdbool.h>
#include "../motor/motor.h"

uint16_t line_adc_buffer[MOVING_AVG_WINDOW] = {0};  // ADC buffer for moving average
uint8_t line_buffer_index = 0;

// Queue handle for message passing
QueueHandle_t xDisplayQueue;


volatile bool black_line_detected = false; // Global variable to store status of black line detection
volatile int stop_running = 0; // Global variable to store count of cycles that black line have not been detected
volatile bool autonomous_running = false; // Global variable to store status of autonomous running

// Threshold variables
static uint32_t min_threshold = 4095;
static uint32_t max_threshold = 0;
static uint32_t contrast_threshold = 2048;  // Initial threshold (mid-range)


// Moving average for ADC readings
/* float get_line_moving_average_adc() {
    adc_select_input(2);
    uint16_t adc_value = adc_read();
    line_adc_buffer[line_buffer_index] = adc_value;
    line_buffer_index = (line_buffer_index + 1) % MOVING_AVG_WINDOW;

    // Calculate the average
    uint32_t sum = 0;
    for (int i = 0; i < MOVING_AVG_WINDOW; i++) {
        sum += line_adc_buffer[i];
    }
    return (sum / MOVING_AVG_WINDOW); 
} */

void update_threshold(uint16_t adc_reading) {
    // Update min and max thresholds
    if (adc_reading < min_threshold) {
        min_threshold = adc_reading;
    }
    if (adc_reading > max_threshold) {
        max_threshold = adc_reading;
    }

    // Recalculate contrast threshold as the average of min and max
    contrast_threshold = (min_threshold + max_threshold) / 2;
    printf("Contrast Threshold: %u\n", contrast_threshold);
}

void vLineFollowingTask(void *pvParameters) {
    while (1) {
        adc_select_input(2);
        float current_ir_value = adc_read(); // Get averaged ADC value
        update_threshold(current_ir_value);
        printf("Current IR Value: %.2f \n", current_ir_value);

        printf("Black Line Detected: %d \n", black_line_detected);

       // Check if a line is detected and update the global flag
       black_line_detected = current_ir_value >= contrast_threshold;
        
        // Delay to allow other tasks to execute and reduce reading frequency
        vTaskDelay(pdMS_TO_TICKS(50));  // Adjust delay as needed
    }
}

void control_motor_on_line_task(void *pvParameters) {
    while (1) {
        // Check if line following mode is active
        if (black_line_detected) {
            // Line is detected; MOVE FORWARD to stay on the line
            move_car(FORWARD, 10, 0, 0);
            autonomous_running = true;
            stop_running = 0;
        } else {      
            // Line not detected; STEER LEFT to search for the line
            move_car(STEER_FORWARD_LEFT, 10, 15, 0);
            vTaskDelay(pdMS_TO_TICKS(200));
            // If line still not detected; STEER RIGHT to search for the line
            if (!black_line_detected) {
                move_car(STEER_FORWARD_RIGHT, 15, 10, 0);
                vTaskDelay(pdMS_TO_TICKS(200)); 
            }  
        }
            stop_running += 1;
        vTaskDelay(pdMS_TO_TICKS(10)); // Short delay for responsive line checking
    }
}


// Function to initialize the ADC for Line Following
void init_line_sensor() {
    adc_init();
    adc_gpio_init(LINE_SENSOR_PIN);

    xTaskCreate(vLineFollowingTask, "Line Following Task", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(control_motor_on_line_task, "Control Motor on Line", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
}

