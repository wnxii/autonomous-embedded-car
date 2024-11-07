#include "FreeRTOS.h"
#include "task.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <stdio.h>
#include <stdbool.h>
#include "semphr.h"
#include <math.h>
#include "../wheel_encoder/wheel_encoder.h"
#include "../motor/motor.h"
#include "../ir_sensor/barcode_scanner/barcode_scanner.h"
#include "../ir_sensor/line_following/line_following.h"
#include "../wifi/barcode_client_socket/barcode_client_socket.h"
#include "main_station_2.h"

// Init all sensors required for station 2
void init_hardware() {
    init_wheel_encoders();
    init_motor();
    init_barcode_wifi();
    init_barcode();
    init_line_sensor();
}

// Callback function for encoder pins
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

// Task for car movement
void car_movement_task(void *pvParameters) {
    // Initialize movement system
    const int lost_line_threshold = 30; // Number of cycles without line detection before stopping
    
    while(1) {
        if(connected) {// Check that car is connected to Wifi and server

            move_car(MOTOR_ON_LINE, 10.0, 0); // Enable line following
            
            while (stop_running <= lost_line_threshold) { // Stop line following once car have gone past the line
                vTaskDelay(50);
            }

            move_car(STOP, 0.0f, 0.0f); // Stop the car
            printf("Line lost consistently. Stopping the car.\n");

            break; // Exit the task if no recovery is expected
        } 
    }
}

// Main function
int main() {
    stdio_init_all();
    printf("Starting car control system\n");

    // Initialize sensors
    init_hardware();

    // Create car movement task
    xTaskCreate(car_movement_task, "Car Movement", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
    gpio_set_irq_enabled_with_callback(LEFT_ENCODER_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &sensors_callback);

    // Start FreeRTOS scheduler
    vTaskStartScheduler();

    // This point will not be reached because the scheduler is running
    while (1) {
        tight_loop_contents();
    }

    return 0;
}