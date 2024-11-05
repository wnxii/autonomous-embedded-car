#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "FreeRTOS.h"
#include "task.h"
#include "wheel_encoder.h"


/* // Display task to show encoder readings
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
} */

// Display task to show encoder readings for both wheels
void display_task(void *params) {
    TickType_t last_wake_time = xTaskGetTickCount();
    
    while (1) {
        // Get current distance and speed for both left and right encoders
        float left_distance = get_left_distance();
        float right_distance = get_right_distance();
        float left_speed = get_left_speed();
        float right_speed = get_right_speed();
        
        // Print the results
        printf("Left Distance: %.2f cm, Left Speed: %.2f cm/s\n", left_distance, left_speed);
        printf("Right Distance: %.2f cm, Right Speed: %.2f cm/s\n", right_distance, right_speed);
        int left_state = gpio_get(LEFT_ENCODER_PIN);
        int right_state = gpio_get(RIGHT_ENCODER_PIN);
        printf("GPIO %d State: %d\n", LEFT_ENCODER_PIN, left_state);
        printf("GPIO %d State: %d\n", RIGHT_ENCODER_PIN, right_state);
        // Run every 500ms
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(500));
    }
}

int main() {
    stdio_init_all();
    printf("Wheel Encoder Demo with FreeRTOS\n");
    
    // Initialize the wheel encoders for both left and right
    init_wheel_encoders();

    reset_right_encoder();
    reset_left_encoder();
    
    // Create display task
    xTaskCreate(display_task, "Display Task", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
    
    // Start FreeRTOS scheduler
    vTaskStartScheduler();
    
    // Should never reach here
    while (1) {
        tight_loop_contents();
    }
    
    return 0;
}