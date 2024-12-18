#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "ultrasonic_sensor.h"
#include <stdio.h>
#include <math.h>
#include "../wifi/client_server_socket/client_server_socket.h"

#define MAX_DISTANCE 400 // Maximum distance in cm
#define TIMEOUT_US 23200 // Timeout in microseconds (based on MAX_DISTANCE)
#define NOISE_THRESHOLD 35.0f  // Adjust based on your application

// FreeRTOS handles
static QueueHandle_t distance_queue;
SemaphoreHandle_t measurement_mutex;
volatile MeasurementData current_measurement = {0, 0, false, false};

/**
 * @brief Main task function for ultrasonic sensor operation
 * @param params Task parameters (unused)
 * 
 * Continuously measures distance using HC-SR04 ultrasonic sensor:
 * 1. Resets measurement state
 * 2. Generates 10μs trigger pulse
 * 3. Waits for echo response with timeout
 * 4. Calculates distance based on echo duration
 * 5. Updates distance queue for other tasks to read
 * Runs every 100ms
 */
void ultrasonic_task(void *params) {
    TickType_t last_wake_time = xTaskGetTickCount();
    float distance;
    char message[100];

    xSemaphoreTake(wifiConnectedSemaphore, portMAX_DELAY);
    
    while (1) {
        // Reset measurement state
        if (xSemaphoreTake(measurement_mutex, portMAX_DELAY) == pdTRUE) {
            current_measurement.start_time = 0;
            current_measurement.end_time = 0;
            current_measurement.measurement_done = false;
            current_measurement.waiting_for_echo = true;
            xSemaphoreGive(measurement_mutex);
        }
        
        // Trigger pulse - ensure proper timing
        gpio_put(TRIGGER_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(2));  // 2ms delay to ensure signal is clear
        gpio_put(TRIGGER_PIN, 1);
        busy_wait_us(10);  // Precise 10 microseconds trigger pulse
        gpio_put(TRIGGER_PIN, 0);
        
        // Wait for measurement or timeout
        TickType_t start_tick = xTaskGetTickCount();
        bool timeout = false;
        bool measurement_complete = false;
        
        // Wait up to 50ms for the measurement
        while (!measurement_complete && !timeout) {
            if (xSemaphoreTake(measurement_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                measurement_complete = current_measurement.measurement_done;
                xSemaphoreGive(measurement_mutex);
            }
            
            if ((xTaskGetTickCount() - start_tick) > pdMS_TO_TICKS(50)) {
                timeout = true;
            }
            
            vTaskDelay(1);
        }
        
        // Calculate and send distance
        if (measurement_complete) {
            if (xSemaphoreTake(measurement_mutex, portMAX_DELAY) == pdTRUE) {
                uint64_t duration = current_measurement.end_time - current_measurement.start_time;
                
                // Valid pulse should be between 150us (2.5cm) and 25ms (400cm)
                if (duration > 150 && duration < 25000) {
                    distance = (duration * 0.0343) / 2.0;
                    
                    if (distance > MAX_DISTANCE) {
                        distance = -1.0f;
                    }
                } else {
                    distance = -1.0f;  // Invalid pulse width
                }
                
                xSemaphoreGive(measurement_mutex);
            }
        } else {
            distance = -1.0f;  // Timeout or error
        }
        
        // Clear old value from queue and send new one
        float prev_distance;
        while (xQueueReceive(distance_queue, &prev_distance, 0) == pdTRUE);  // Clear queue

        xQueueSend(distance_queue, &distance, 0);

        snprintf(message, sizeof(message), "ULTRA: Distance to object = %.2f", distance + 4.0);
        xQueueSend(xServerQueue, &message, portMAX_DELAY); 
        
        taskYIELD();
        // Run every 100ms
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(100));
    }
}

/**
 * @brief Initializes the ultrasonic sensor and creates required RTOS objects
 * 
 * Sets up:
 * - FreeRTOS queue for distance measurements
 * - Mutex for thread-safe measurement access
 * - GPIO pins for trigger and echo
 * - Interrupt handlers for echo pin
 * - Creates ultrasonic measurement task
 */
void init_ultrasonic_sensor() {
    // Create FreeRTOS objects
    distance_queue = xQueueCreate(1, sizeof(float));  // Reduced to size 1
    measurement_mutex = xSemaphoreCreateMutex();
    
    // Initialize GPIO
    gpio_init(TRIGGER_PIN);
    gpio_init(ECHO_PIN);
    gpio_set_dir(TRIGGER_PIN, GPIO_OUT);
    gpio_set_dir(ECHO_PIN, GPIO_IN);
    
    // Start with trigger pin low
    gpio_put(TRIGGER_PIN, 0);
    
    gpio_set_irq_enabled(ECHO_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                                     true);
    // Create ultrasonic task
    xTaskCreate(ultrasonic_task, 
               "Ultrasonic Task", 
               configMINIMAL_STACK_SIZE * 2,  // Increased stack size
               NULL, 
               tskIDLE_PRIORITY + 4, 
               NULL);
}

/**
 * @brief Retrieves the most recent distance measurement
 * @return float Distance in cm, or -1.0 if no valid measurement available
 * 
 * Thread-safe function to peek at the latest distance measurement
 * from the sensor's queue without removing it
 */
float measure_distance() {
    float distance = -1.0f;
    if (xQueuePeek(distance_queue, &distance, 0) != pdTRUE) {
        return -1.0f;
    }
    return distance;
}

/**
 * @brief Checks if an obstacle is detected within the safety threshold
 * @param safety_threshold Distance threshold in cm to detect obstacles
 * @return bool True if obstacle detected within threshold, false otherwise
 * 
 * Uses the latest distance measurement to determine if an obstacle
 * is present within the specified safety threshold distance
 */
bool is_obstacle_detected(float safety_threshold) {
    char message[100];
    float distance = measure_distance();
    bool obstacle_detected = distance > 0 && distance <= safety_threshold;
    if (obstacle_detected) {
        snprintf(message, sizeof(message), "ULTRA: Obstacle detected %.2f away.", distance);
        xQueueSend(xServerQueue, &message, portMAX_DELAY);
    }
    return (obstacle_detected);
}