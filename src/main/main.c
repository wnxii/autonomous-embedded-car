#include "pico/cyw43_arch.h"
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
// #include "../wifi/server_socket/server_socket.h"
#include "../ultrasonic_sensor/ultrasonic_sensor.h"
#include "main.h"

bool obstacle_detected = false; // Flag to track if an obstacle is detected

// Callback function for all GPIO interrupts
void sensors_callback(uint gpio, uint32_t events) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (gpio == LEFT_ENCODER_PIN) {
        // printf("Left encoder callback triggered - GPIO %d\n", gpio);
        if (xSemaphoreTakeFromISR(left_data_mutex, &xHigherPriorityTaskWoken) == pdTRUE) {
            left_data.pulse_count++;
            left_data.timestamp = time_us_64();
            xSemaphoreGiveFromISR(left_data_mutex, &xHigherPriorityTaskWoken);
        }
    } 
    else if (gpio == RIGHT_ENCODER_PIN) {
        // printf("Right encoder callback triggered - GPIO %d\n", gpio);
        if (xSemaphoreTakeFromISR(right_data_mutex, &xHigherPriorityTaskWoken) == pdTRUE) {
            right_data.pulse_count++;
            right_data.timestamp = time_us_64();
            xSemaphoreGiveFromISR(right_data_mutex, &xHigherPriorityTaskWoken);
        }
    } 
    else if (gpio == ECHO_PIN) {
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
    }
    else if (gpio == BTN_PIN && (events & GPIO_IRQ_EDGE_FALL)) {
        // Button press handling is done in the button task using polling
        // This is just to properly register the interrupt
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// Function to initialize all GPIO interrupts
void init_gpio_interrupts() {
    printf("[DEBUG] Initializing GPIO interrupts...\n");
    
    // Initialize GPIO pins first
    gpio_init(LEFT_ENCODER_PIN);
    gpio_set_dir(LEFT_ENCODER_PIN, GPIO_IN);
    gpio_pull_up(LEFT_ENCODER_PIN);

    gpio_init(RIGHT_ENCODER_PIN);
    gpio_set_dir(RIGHT_ENCODER_PIN, GPIO_IN);
    gpio_pull_up(RIGHT_ENCODER_PIN);

    gpio_init(ECHO_PIN);
    gpio_set_dir(ECHO_PIN, GPIO_IN);

    gpio_init(BTN_PIN);
    gpio_set_dir(BTN_PIN, GPIO_IN);
    gpio_pull_up(BTN_PIN);

    // Set up GPIO interrupts with callback for all pins that need interrupts
    gpio_set_irq_enabled_with_callback(LEFT_ENCODER_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &sensors_callback);
    gpio_set_irq_enabled(RIGHT_ENCODER_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(ECHO_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(BTN_PIN, GPIO_IRQ_EDGE_FALL, true);

    printf("[DEBUG] GPIO interrupts initialized\n");
}

// Init all sensors required for station 2
void init_hardware() {
    // Initialize queues first so they're available for all tasks
    printf("[DEBUG] [1/8] INITIALIZING SENSOR QUEUES\n");
    init_sensor_queues();
    sleep_ms(1000);

    printf("[DEBUG] [2/8] INITIALIZING WHEEL ENCODERS\n");
    init_wheel_encoders();
    sleep_ms(1000);

    printf("[DEBUG] [3/8] INITIALIZING MOTOR\n");
    init_motor();
    sleep_ms(1000);

    printf("[DEBUG] [4/8] INITIALIZING BARCODE SCANNER\n");
    init_barcode();
    sleep_ms(1000);

    printf("[DEBUG] [5/8] INITIALIZING LINE SENSOR\n");
    // init_line_sensor();
    sleep_ms(1000);

    printf("[DEBUG] [6/8] INITIALIZING ULTRASONIC SENSOR\n");
    init_ultrasonic_sensor();
    sleep_ms(1000);

    printf("[DEBUG] [7/8] INITIALIZING UDP CLIENT SOCKET\n");
    init_barcode_wifi();
    sleep_ms(1000);

    // printf("[8/8] INITIALIZING UDP SERVER SOCKET\n");
    // init_server_socket();
    // sleep_ms(1000);

    printf("[DEBUG] HARDWARE INITIALIZATION COMPLETE\n");
}

// Helper function to get average distance traveled by both wheels
float get_average_distance() {
    float average_distance = (get_left_distance() + get_right_distance()) / 2.0f;
    return average_distance;
}

// Task for car movement
void car_movement_task(void *pvParameters) {
    printf("[DEBUG] Starting car movement task\n");
    
    // Initialize movement system
    MotorControl control;
    
    // Code for Remote Control
    while (true)
    {
        // Check for obstacle detection
        if (is_obstacle_detected(SAFETY_THRESHOLD)) {
            obstacle_detected = true;
            move_car(STOP, 0.0, 0.0, 0.0); // Stop the car immediately
            printf("[DEBUG] Obstacle detected - stopping car\n");
        }

        // If obstacle is detected, only allow backward movement
        if (obstacle_detected) {
            if (remote_target_speed < 0) { // Allow backward movement only
                control = map_remote_output_to_direction(remote_target_speed, remote_steering);
                if (control.direction == BACKWARD || control.direction == STEER_BACKWARD_LEFT || control.direction == STEER_BACKWARD_RIGHT) {
                    move_car(control.direction, control.left_wheel_speed, control.right_wheel_speed, 0.0);
                    obstacle_detected = false;
                    printf("[DEBUG] Moving backward - obstacle cleared\n");
                } else {
                    move_car(STOP, 0.0, 0.0, 0.0); // Stop if forward movement is attempted
                    printf("[DEBUG] Forward movement blocked due to obstacle\n");
                }
            } else {
                move_car(STOP, 0.0, 0.0, 0.0); // Stop if forward movement is attempted
                printf("[DEBUG] Forward movement blocked due to obstacle\n");
            }
        } else {
            // Normal operation when no obstacle is detected
            control = map_remote_output_to_direction(remote_target_speed, remote_steering);
            move_car(control.direction, control.left_wheel_speed, control.right_wheel_speed, 0.0);
        }

        vTaskDelay(50);
    } 
}

// Main function
int main() {
    stdio_init_all();
    printf("[DEBUG] Starting car control system\n");

    // Initialize GPIO interrupts first
    init_gpio_interrupts();

    // Initialize all other hardware
    init_hardware();

    // Create car movement task
    xTaskCreate(car_movement_task, "Car Movement", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
    printf("[DEBUG] Car movement task created\n");

    // Start FreeRTOS scheduler
    printf("[DEBUG] Starting FreeRTOS scheduler\n");
    vTaskStartScheduler();

    // This point will not be reached because the scheduler is running
    while (1) {
        tight_loop_contents();
    }

    return 0;
}
