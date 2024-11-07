#include "FreeRTOS.h"
#include "task.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <stdio.h>
#include <stdbool.h>
#include "semphr.h"
#include <math.h>
#include "../main_wheel_encoder/wheel_encoder.h" // Adjust this path if necessary
#include "../motor/motor.h"
#include "../ir_sensor/barcode_scanner/barcode_scanner.h"
#include "../ir_sensor/line_following/line_following.h"
#include "../wifi/barcode_client_socket/barcode_client_socket.h"
#include "main_station_2.h"

void init_hardware() {
    init_wheel_encoders();
    init_motor();
    init_barcode_wifi();
    init_barcode();
    init_line_sensor();
}

void sensors_callback(uint gpio, uint32_t events) {
    if (gpio == LEFT_ENCODER_PIN) {
        // printf("Left encoder callback triggered - GPIO %d\n", gpio);
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        if (xSemaphoreTakeFromISR(left_data_mutex, &xHigherPriorityTaskWoken) == pdTRUE) {
            left_data.pulse_count++;
            left_data.timestamp = time_us_64();
            xSemaphoreGiveFromISR(left_data_mutex, &xHigherPriorityTaskWoken);
        }

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    } 
    else if (gpio == RIGHT_ENCODER_PIN) {
        // printf("Right encoder callback triggered - GPIO %d\n", gpio);
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        if (xSemaphoreTakeFromISR(right_data_mutex, &xHigherPriorityTaskWoken) == pdTRUE) {
            right_data.pulse_count++;
            right_data.timestamp = time_us_64();
            xSemaphoreGiveFromISR(right_data_mutex, &xHigherPriorityTaskWoken);
        }

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    } 
}

// Task for robot movement
void car_movement_task(void *pvParameters) {
    // Initialize movement system
    float target_speed = 10.0f;
    int lost_line_counter = 0;
    const int lost_line_threshold = 10; // Number of cycles without line detection before stopping

    while(1) {
         // Control logic for line following
        if (black_line_detected) {
            // Line detected; move forward along the line
            move_car(FORWARD, target_speed, 0.0f);
            lost_line_counter = 0; // Reset the counter when the line is detected
        } else {
            // Increment the lost line counter
            lost_line_counter++;
            // Take corrective action if line is lost but threshold not reached
            if (lost_line_counter < lost_line_threshold) {
                // Line lost; take corrective action with a small left turn
                move_car(LEFT, target_speed, 0.0f);
                vTaskDelay(pdMS_TO_TICKS(50)); // Small delay to prevent busy-waiting

                // If line is still lost, attempt a small correction to the right
                if (!black_line_detected) {
                    move_car(RIGHT, target_speed, 0.0f);
                    vTaskDelay(pdMS_TO_TICKS(50)); // Small delay to prevent busy-waiting
                }
            } else {
                // Stop the car if the line has been undetected for too long
                move_car(STOP, 0.0f, 0.0f);
                printf("Line lost consistently. Stopping the car.\n");
                break; // Optionally end the task here if no recovery is expected
            }
        }
/* 
        if (scan_started && error_scanning) {
            move_car(STOP, target_speed, 0.0f);
            move_car(BACKWARD, target_speed, 0.0f);
            vTaskDelay(pdMS_TO_TICKS(3000));
            move_car(STOP, target_speed, 0.0f);
            reset_barcode();
        } */
        
        vTaskDelay(pdMS_TO_TICKS(50)); // Loop delay for stability
    }
}

// Main function
int main() {
    stdio_init_all();
    printf("Starting car control system\n");
    // init_robot_movement();

    // Create robot movement task
    init_hardware();
    xTaskCreate(car_movement_task, "Robot Movement", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
    gpio_set_irq_enabled_with_callback(LEFT_ENCODER_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &sensors_callback);

    // Start FreeRTOS scheduler
    vTaskStartScheduler();

    // This point will not be reached because the scheduler is running
    while (1) {
        tight_loop_contents();
    }

    return 0;
}