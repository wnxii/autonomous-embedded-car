#include "FreeRTOS.h"
#include "task.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <stdio.h>
#include <stdbool.h>
#include "semphr.h"
#include <math.h>
#include "../wheel_encoder/wheel_encoder.h" // Adjust this path if necessary
#include "../ultrasonic_sensor/ultrasonic_sensor.h"
#include "../motor/motor.h"
#include "../ir_sensor/line_following/line_following.h"
#include "main_station_1.h"

// Init all sensors required for station 1
void init_hardware() {
    init_wheel_encoders();
    init_ultrasonic_sensor();
    init_motor();
}

// Callback function for encoder and ultrasonic pins 
void sensors_callback(uint gpio, uint32_t events) {
    if (gpio == LEFT_ENCODER_PIN) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        if (xSemaphoreTakeFromISR(left_data_mutex, &xHigherPriorityTaskWoken) == pdTRUE) {
            left_data.pulse_count++;
            left_data.timestamp = time_us_64();
            xSemaphoreGiveFromISR(left_data_mutex, &xHigherPriorityTaskWoken);
        }

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    } 
    else if (gpio == RIGHT_ENCODER_PIN) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        if (xSemaphoreTakeFromISR(right_data_mutex, &xHigherPriorityTaskWoken) == pdTRUE) {
            right_data.pulse_count++;
            right_data.timestamp = time_us_64();
            xSemaphoreGiveFromISR(right_data_mutex, &xHigherPriorityTaskWoken);
        }

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    } 
    else if (gpio == ECHO_PIN) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        uint64_t current_time = time_us_64();
        
        if (xSemaphoreTakeFromISR(measurement_mutex, &xHigherPriorityTaskWoken) == pdTRUE) {
            if (events & GPIO_IRQ_EDGE_RISE) {
                if (current_measurement.waiting_for_echo) {
                    current_measurement.start_time = current_time;
                }
            } else if (events & GPIO_IRQ_EDGE_FALL) {
                if (current_measurement.start_time > 0) {
                    current_measurement.end_time = current_time;
                    current_measurement.measurement_done = true;
                    current_measurement.waiting_for_echo = false;
                }
            }
            xSemaphoreGiveFromISR(measurement_mutex, &xHigherPriorityTaskWoken);
        }
        
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

// Helper function to get average distance traveled by both wheels
float get_average_distance() {
    float average_distance = (get_left_distance() + get_right_distance()) / 2.0f;
    // printf("Average Distance: %f", average_distance);
    return average_distance;
}


// Task for car movement
void car_movement_task(void *pvParameters) {
    // Initialize movement system
    float target_speed = 32.5f;
    while(1) {
        reset_left_encoder();
        reset_right_encoder();
        
        // Move forward until an obstacle is detected 10cm away
        move_car(FORWARD, target_speed, 0.0f);
        while (!is_obstacle_detected(SAFETY_THRESHOLD))
        {
           vTaskDelay(5);
        }
        
        // Stop the car
        move_car(STOP, 0.0f, 0.0f);
        printf("Obstacle detected within 10 cm. Stopping.\n");
        vTaskDelay(pdMS_TO_TICKS(500)); // Pause briefly before turning
        
        // Turn right 90 degrees
        printf("Turning right 90 degrees\n");
        reset_left_encoder();
        reset_right_encoder();
        move_car(PIVOT_RIGHT, target_speed, 90.0f);  // Perform 90-degree turn
        while (turning_active) {
            vTaskDelay(pdMS_TO_TICKS(10)); // Small delay to prevent busy-waiting
        }
        move_car(STOP, 0.0f, 0.0f); // Stop after completing the turn
        vTaskDelay(pdMS_TO_TICKS(500)); // Pause briefly before moving forward

        // Move forward 90 cm
        printf("Moving forward 90 cm\n");
        reset_left_encoder();
        reset_right_encoder();
        double start_timestamp = time_us_64() / 1000000.0; // Start time - Converts microseconds to seconds
        move_car(FORWARD, target_speed, 0.0f);  // Set speed (e.g., 20 cm/s)
        while (get_average_distance() < 90.0f) { 
            vTaskDelay(pdMS_TO_TICKS(5)); // Delay to periodically check distance
        }
        double end_timestamp = time_us_64() / 1000000.0; // End time - Converts microseconds to seconds
        double time_diff = end_timestamp - start_timestamp;
        float average_speed = 90 / time_diff;
        printf("Average speed: %f \n", average_speed);
        
        // Stop the car
        move_car(STOP, 0.0f, 0.0f); // Stop after reaching target distance
        printf("Reached 90 cm. Stopping.\n");
        vTaskDelay(pdMS_TO_TICKS(500)); // Small pause

        break; // End the task or repeat as needed
    }
}

// Main function
int main() {
    stdio_init_all();
    printf("Starting car control system\n");

    // Initialize sensors
    init_hardware();

        // Create car movement task
    xTaskCreate(car_movement_task, "Robot Movement", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
    gpio_set_irq_enabled_with_callback(ECHO_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &sensors_callback);

    // Start FreeRTOS scheduler
    vTaskStartScheduler();

    // This point will not be reached because the scheduler is running
    while (1) {
        tight_loop_contents();
    }

    return 0;
}