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

// External queue declaration
extern QueueHandle_t xServerQueue;

void ultrasonic_task(void *params) {
    printf("[DEBUG] Starting ultrasonic task\n");
    
    if (!xServerQueue) {
        printf("[ERROR] Server queue not initialized\n");
        vTaskDelete(NULL);
        return;
    }

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
        
        // Trigger pulse with precise timing
        gpio_put(TRIGGER_PIN, 0);
        busy_wait_us(5);  // 5us LOW to ensure clean pulse
        gpio_put(TRIGGER_PIN, 1);
        busy_wait_us(10); // Exactly 10us pulse
        gpio_put(TRIGGER_PIN, 0);
        
        // Wait for measurement with longer timeout
        TickType_t start_tick = xTaskGetTickCount();
        bool timeout = false;
        bool measurement_complete = false;
        
        while (!measurement_complete && !timeout) {
            if (xSemaphoreTake(measurement_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                measurement_complete = current_measurement.measurement_done;
                xSemaphoreGive(measurement_mutex);
            }
            
            if ((xTaskGetTickCount() - start_tick) > pdMS_TO_TICKS(100)) { // Increased timeout to 100ms
                timeout = true;
            }
            
            vTaskDelay(1);
        }
        
        // Calculate distance
        if (measurement_complete) {
            if (xSemaphoreTake(measurement_mutex, portMAX_DELAY) == pdTRUE) {
                uint64_t duration = current_measurement.end_time - current_measurement.start_time;
                
                if (duration > 150 && duration < 25000) {
                    distance = (duration * 0.0343) / 2.0;
                    
                    if (distance > MAX_DISTANCE) {
                        distance = -1.0f;
                    }
                } else {
                    distance = -1.0f;
                }
                
                xSemaphoreGive(measurement_mutex);
            }
        } else {
            distance = -1.0f;
        }
        
        // Update distance queue
        float prev_distance;
        while (xQueueReceive(distance_queue, &prev_distance, 0) == pdTRUE);
        xQueueSend(distance_queue, &distance, 0);

        // Send to UDP queue periodically
        bool obstacle = (distance > 0 && distance <= 30.0f);
        snprintf(message, sizeof(message), "ULTRA:DIST=%.2f,OBS=%d", distance, obstacle);
        if (xQueueSend(xServerQueue, &message, 0) != pdTRUE) {
            printf("[DEBUG] Failed to send ultrasonic data to server queue\n");
        } else {
            printf("[DEBUG] Sent ultrasonic data: Distance=%.2f cm, Obstacle=%d\n", distance, obstacle);
        }
        
        // Run every 200ms to allow more time for echo
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(200));
    }
}

void init_ultrasonic_sensor() {
    printf("[DEBUG] Initializing ultrasonic sensor\n");
    
    // Create FreeRTOS objects
    distance_queue = xQueueCreate(1, sizeof(float));
    if (distance_queue == NULL) {
        printf("[ERROR] Failed to create distance queue\n");
        return;
    }
    printf("[DEBUG] Created distance queue\n");
    
    measurement_mutex = xSemaphoreCreateMutex();
    if (measurement_mutex == NULL) {
        printf("[ERROR] Failed to create measurement mutex\n");
        return;
    }
    printf("[DEBUG] Created measurement mutex\n");
    
    // Initialize GPIO
    gpio_init(TRIGGER_PIN);
    gpio_init(ECHO_PIN);
    gpio_set_dir(TRIGGER_PIN, GPIO_OUT);
    gpio_set_dir(ECHO_PIN, GPIO_IN);
    
    // Start with trigger pin low
    gpio_put(TRIGGER_PIN, 0);
    printf("[DEBUG] GPIO initialized\n");
    
    // Create ultrasonic task
    TaskHandle_t ultrasonic_handle = NULL;
    xTaskCreate(ultrasonic_task, 
               "Ultrasonic Task", 
               configMINIMAL_STACK_SIZE * 2,
               NULL, 
               tskIDLE_PRIORITY + 1, 
               &ultrasonic_handle);
               
    if (ultrasonic_handle == NULL) {
        printf("[ERROR] Failed to create ultrasonic task\n");
        return;
    }
    printf("[DEBUG] Created ultrasonic task\n");
    
    printf("[DEBUG] Ultrasonic sensor initialization complete\n");
}

float measure_distance() {
    float distance = -1.0f;
    xQueuePeek(distance_queue, &distance, 0);
    return distance;
}

bool is_obstacle_detected(float safety_threshold) {
    float distance = measure_distance();
    return (distance > 0 && distance <= safety_threshold);
}
