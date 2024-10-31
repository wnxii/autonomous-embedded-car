#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "FreeRTOS.h"
#include "task.h"

// Function prototypes from ultrasonic_sensor.c
void init_ultrasonic_sensor();
float measure_distance();
bool is_obstacle_detected(float safety_threshold);

#define SAFETY_THRESHOLD 20.0f // 20 cm safety threshold

// Display task to show ultrasonic sensor readings
void display_task(void *params) {
    TickType_t last_wake_time = xTaskGetTickCount();
    
    while (1) {
        // Measure distance
        float distance = measure_distance();
        
        if (distance < 0) {
            printf("Error: Measurement timeout or out of range\n");
        } else {
            printf("Distance: %.2f cm\n", distance);
            
            if (is_obstacle_detected(SAFETY_THRESHOLD)) {
                printf("WARNING: Obstacle detected within safety threshold!\n");
            }
        }
        
        // Run every 500ms
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(500));
    }
}

int main() {
    stdio_init_all();
    printf("Ultrasonic Sensor Demo with FreeRTOS\n");
    
    // Initialize the ultrasonic sensor
    init_ultrasonic_sensor();
    
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