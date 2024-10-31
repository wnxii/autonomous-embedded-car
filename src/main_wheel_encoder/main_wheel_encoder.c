#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "FreeRTOS.h"
#include "task.h"

// Function prototypes from wheel_encoder.c
void init_wheel_encoder();
float get_distance();
float get_speed();
void reset_encoder();

// Display task to show encoder readings
void display_task(void *params) {
    TickType_t last_wake_time = xTaskGetTickCount();
    
    while (1) {
        // Get current distance and speed
        float distance = get_distance();
        float speed = get_speed();
        
        // Print the results
        printf("Distance: %.2f cm, Speed: %.2f cm/s\n", distance, speed);
        
        // Run every 500ms
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(500));
    }
}

int main() {
    stdio_init_all();
    printf("Wheel Encoder Demo with FreeRTOS\n");
    
    // Initialize the wheel encoder
    init_wheel_encoder();
    
    // Create display task
    xTaskCreate(display_task, "Display Task", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
    
    // Start FreeRTOS scheduler
    vTaskStartScheduler();
    
    // We should never get here
    while (1) {
        tight_loop_contents();
    }
    
    return 0;
}