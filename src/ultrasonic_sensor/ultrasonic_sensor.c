#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "ultrasonic_sensor.h"
#include "../wifi/barcode_client_socket/barcode_client_socket.h"
#include <stdio.h>
#include <math.h>

#define MAX_DISTANCE 400 // Maximum distance in cm
#define TIMEOUT_US 23200 // Timeout in microseconds (based on MAX_DISTANCE)
#define NOISE_THRESHOLD 35.0f  // Adjust based on your application

// FreeRTOS handles
static QueueHandle_t distance_queue;
SemaphoreHandle_t measurement_mutex;
volatile MeasurementData current_measurement = {0, 0, false, false};

void ultrasonic_task(void *params) {
    TickType_t last_wake_time = xTaskGetTickCount();
    float distance;
    char message[100];
    
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

        if (xQueueSend(distance_queue, &distance, 0) == pdTRUE) {
            // Send to UDP client queue
            bool obstacle = (distance > 0 && distance <= 30.0f); // 30cm safety threshold
            snprintf(message, sizeof(message), "DIST=%.2f,OBS=%d", distance, obstacle);
            xQueueSend(xUltrasonicQueue, &message, 0);
            printf("Ultrasonic Distance: %.2f cm, Obstacle: %d\n", distance, obstacle);
        }
        
        // Run every 100ms
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10));
    }
}

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
    
    gpio_set_irq_enabled(ECHO_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

    // Create ultrasonic task
    xTaskCreate(ultrasonic_task, 
               "Ultrasonic Task", 
               configMINIMAL_STACK_SIZE * 2,  // Increased stack size
               NULL, 
               tskIDLE_PRIORITY + 1, 
               NULL);
}

float measure_distance() {
    float distance = -1.0f;
    if (xQueuePeek(distance_queue, &distance, 0) != pdTRUE) {
        return -1.0f;
    }
    return distance;
}

bool is_obstacle_detected(float safety_threshold) {
    float distance = measure_distance();
    printf("Ultrasonic Distance: %f \n", distance);
    return (distance > 0 && distance <= safety_threshold);
}
