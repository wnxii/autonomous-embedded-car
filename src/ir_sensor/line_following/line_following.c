#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "line_following.h"
#include "queue.h"
#include <stdbool.h>

uint16_t line_adc_buffer[MOVING_AVG_WINDOW] = {0};  // ADC buffer for moving average
uint8_t line_buffer_index = 0;

// Queue handle for message passing
QueueHandle_t xDisplayQueue;

volatile bool black_line_detected = false;

// Moving average for ADC readings
float get_line_moving_average_adc() {
    adc_select_input(2);
    uint16_t adc_value = adc_read();
    line_adc_buffer[line_buffer_index] = adc_value;
    line_buffer_index = (line_buffer_index + 1) % MOVING_AVG_WINDOW;

    // Calculate the average
    uint32_t sum = 0;
    for (int i = 0; i < MOVING_AVG_WINDOW; i++) {
        sum += line_adc_buffer[i];
    }
    return (sum / MOVING_AVG_WINDOW) * (3.3f / 4095);  // Convert to volts
}


void vLineFollowingTask(void *pvParameters) {
    while (1) {
        // char message[100]; // Message buffer
        float current_ir_value = get_line_moving_average_adc(); // Get averaged ADC value in volts
        printf("Current IR Value: %.2f \n", current_ir_value);

        printf("Black Line Detected: %d \n", black_line_detected);

       // Check if a line is detected and update the global flag
       black_line_detected = current_ir_value >= LINE_CONTRAST_THRESHOLD;
        
        // Delay to allow other tasks to execute and reduce reading frequency
        vTaskDelay(pdMS_TO_TICKS(50));  // Adjust delay as needed
    }
}



// Function to initialize the ADC for Line Following
void init_line_sensor() {
    adc_init();
    adc_gpio_init(LINE_SENSOR_PIN);

    xTaskCreate(vLineFollowingTask, "Line Following Task", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
}

