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

/**
 * @file main.c
 * @brief Main control system for the robotic car.
 *
 * This file contains the main logic for initializing hardware components,
 * handling sensor callbacks, and managing car movement tasks.
 */

bool obstacle_detected = false; // Flag to track if an obstacle is detected

/**
 * @brief Initialize all required hardware components.
 *
 * This function initializes various sensors and components required for
 * the operation of the robotic car, including sensor queues, UDP dashboard,
 * wheel encoders, motor, ultrasonic sensor, line sensor, and barcode scanner.
 */
void init_hardware() {
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
}

/**
 * @brief Callback function for handling sensor interrupts.
 *
 * This function is triggered by GPIO interrupts and handles events for
 * left and right encoder pins, as well as the ultrasonic sensor's echo pin.
 *
 * @param gpio The GPIO pin number that triggered the interrupt.
 * @param events The type of event that occurred (e.g., rising or falling edge).
 */
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

/**
 * @brief Calculate the average distance traveled by both wheels.
 *
 * This helper function computes the average distance based on the
 * distances traveled by the left and right wheels.
 *
 * @return The average distance traveled by both wheels.
 */
float get_average_distance() {
    float average_distance = (get_left_distance() + get_right_distance()) / 2.0f;
    return average_distance;
}

/**
 * @brief Task for managing car movement.
 *
 * This task handles the movement of the car, including remote control
 * and obstacle detection. It allows backward movement when an obstacle
 * is detected and stops the car if forward movement is attempted.
 *
 * @param pvParameters Parameters for the task (not used).
 */
void car_movement_task(void *pvParameters) {
    MotorControl control;

    xSemaphoreTake(wifiConnectedSemaphore, portMAX_DELAY);
    
    while (1) {
        if (get_autonomous_running()) {
            continue;
        }
        
        if (is_obstacle_detected(SAFETY_THRESHOLD)) {
            obstacle_detected = true;
            move_car(STOP, 0.0, 0.0, 0.0);
        }

        if (obstacle_detected) {
            if (remote_target_speed < 0) {
                control = map_remote_output_to_direction(remote_target_speed, remote_steering);
                if (control.direction == BACKWARD || control.direction == STEER_BACKWARD_LEFT || control.direction == STEER_BACKWARD_RIGHT) {
                    move_car(control.direction, control.left_wheel_speed, control.right_wheel_speed, 0.0);
                    obstacle_detected = false;
                } else {
                    move_car(STOP, 0.0, 0.0, 0.0);
                    printf("Forward movement blocked due to obstacle.\n");
                }
            } else {
                move_car(STOP, 0.0, 0.0, 0.0);
                printf("Forward movement blocked due to obstacle.\n");
            }
        } else {
            control = map_remote_output_to_direction(remote_target_speed, remote_steering);
            move_car(control.direction, control.left_wheel_speed, control.right_wheel_speed, 0.0);
        }
        taskYIELD();
        vTaskDelay(50);
    }
}

/**
 * @brief Main function for the car control system.
 *
 * This function initializes the system, creates the car movement task,
 * and starts the FreeRTOS scheduler.
 *
 * @return Exit status (not used as the scheduler takes over).
 */
int main() {
    stdio_init_all();
    printf("Starting car control system\n");

    init_hardware();

    xTaskCreate(car_movement_task, "Car Movement", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 4, NULL);
    gpio_set_irq_enabled_with_callback(LEFT_ENCODER_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &sensors_callback);

    vTaskStartScheduler();

    while (1) {
        tight_loop_contents();
    }

    return 0;
}
