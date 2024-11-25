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
#include "../wifi/client_server_socket/client_server_socket.h"
#include "../ultrasonic_sensor/ultrasonic_sensor.h"
#include "main.h"

bool obstacle_detected = false; // Flag to track if an obstacle is detected

// Init all sensors required for station 2
void init_hardware() {
    // char message[100]; // Message buffer

    printf("[DEBUG] [1/7] INITIALIZING SENSOR QUEUES\n");
    init_sensor_queues();
    sleep_ms(1000);

    printf("[DEBUG] [2/7] INITIALIZING UDP DASHBOARD AND REMOTE SOCKET\n");
    init_wifi();
    sleep_ms(1000);

    printf("[DEBUG] [3/7] INITIALIZING WHEEL ENCODERS\n");
    init_wheel_encoders();
    sleep_ms(1000);

    printf("[DEBUG] [4/7] INITIALIZING MOTOR\n");
    init_motor();
    sleep_ms(1000);

    printf("[DEBUG] [5/7] INITIALIZING ULTRASONIC SENSOR\n");
    init_ultrasonic_sensor();
    sleep_ms(1000);

    printf("[DEBUG] [6/7] INITIALIZING LINE SENSOR\n");
    init_line_sensor();
    sleep_ms(1000);

    printf("[DEBUG] [7/7] INITIALIZING BARCODE SCANNER\n");
    init_barcode();
    sleep_ms(1000);

    printf("[DEBUG] HARDWARE INITIALIZATION COMPLETE\n");

    // set_autonomous_running(false);
    

/*     snprintf(message, sizeof(message), "[DEBUG] [1/7] INITIALIZING SENSOR QUEUES\n");
    xQueueSend(xServerQueue, &message, portMAX_DELAY);
    sleep_ms(1000);

    snprintf(message, sizeof(message), "[DEBUG] [2/7] INITIALIZING UDP DASHBOARD AND REMOTE SOCKET\n");
    xQueueSend(xServerQueue, &message, portMAX_DELAY);
    sleep_ms(1000);   */


    /* snprintf(message, sizeof(message), "[DEBUG] [3/7] INITIALIZING WHEEL ENCODERS\n");
    xQueueSend(xServerQueue, &message, portMAX_DELAY);
    init_wheel_encoders();
    sleep_ms(1000);

    snprintf(message, sizeof(message), "[DEBUG] [4/7] INITIALIZING MOTOR\n");
    xQueueSend(xServerQueue, &message, portMAX_DELAY);
    init_motor();
    sleep_ms(1000); */

/*     snprintf(message, sizeof(message), "[DEBUG] [5/7] INITIALIZING BARCODE SCANNER\n");
    xQueueSend(xServerQueue, &message, portMAX_DELAY);
    init_barcode();
    sleep_ms(1000);

    snprintf(message, sizeof(message), "[DEBUG] [6/7] INITIALIZING LINE SENSOR\n");
    xQueueSend(xServerQueue, &message, portMAX_DELAY);
    init_line_sensor();
    sleep_ms(1000); */

    /* snprintf(message, sizeof(message), "[DEBUG] [7/7] INITIALIZING ULTRASONIC SENSOR\n");
    xQueueSend(xServerQueue, &message, portMAX_DELAY);
    init_ultrasonic_sensor();
    sleep_ms(1000);



    snprintf(message, sizeof(message), "[DEBUG] HARDWARE INITIALIZATION COMPLETE\n");
    xQueueSend(xServerQueue, &message, portMAX_DELAY); */


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
    MotorControl control;

    xSemaphoreTake(wifiConnectedSemaphore, portMAX_DELAY);
    
    // Code for Remote Control
    while (1)
    {
        if (get_autonomous_running()) {
            continue;
        }

        xQueueSend(xServerQueue, "Current State - Remote\n", portMAX_DELAY);
        
        // Check for obstacle detection
        if (is_obstacle_detected(SAFETY_THRESHOLD)) {
            obstacle_detected = true;
            move_car(STOP, 0.0, 0.0, 0.0); // Stop the car immediately
        }

        // If obstacle is detected, only allow backward movement
        if (obstacle_detected) {
            if (remote_target_speed < 0) { // Allow backward movement only
                control = map_remote_output_to_direction(remote_target_speed, remote_steering);
                if (control.direction == BACKWARD || control.direction == STEER_BACKWARD_LEFT || control.direction == STEER_BACKWARD_RIGHT) {
                    move_car(control.direction, control.left_wheel_speed, control.right_wheel_speed, 0.0);
                    obstacle_detected = false;
                } else {
                    move_car(STOP, 0.0, 0.0, 0.0); // Stop if forward movement is attempted
                    printf("Forward movement blocked due to obstacle.\n");
                }
            } else {
                move_car(STOP, 0.0, 0.0, 0.0); // Stop if forward movement is attempted
                printf("Forward movement blocked due to obstacle.\n");
            }
        } else {
            // Normal operation when no obstacle is detected
            control = map_remote_output_to_direction(remote_target_speed, remote_steering);
            move_car(control.direction, control.left_wheel_speed, control.right_wheel_speed, 0.0);
        }
        taskYIELD();
        vTaskDelay(50);
    }

    // Code for Autonomous Line Following and Barcode Scanning
    
    // while(1) {
    //     move_car(FORWARD, 35.0, 35.0, 0.0);
    // }
}

// Main function
int main() {
    stdio_init_all();
    printf("Starting car control system\n");

    // Initialize sensors
    init_hardware();

    // Create car movement task
    xTaskCreate(car_movement_task, "Car Movement", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 4, NULL);
    gpio_set_irq_enabled_with_callback(LEFT_ENCODER_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &sensors_callback);

    // Start FreeRTOS scheduler
    vTaskStartScheduler();

    // This point will not be reached because the scheduler is running
    while (1) {
        tight_loop_contents();
    }

    return 0;
}
