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

// External queue declaration
extern QueueHandle_t xServerQueue;

// Function to measure distance using polling
float get_pulse_duration() {
    uint64_t start_time = 0;
    uint64_t end_time = 0;
    
    // Generate trigger pulse
    gpio_put(TRIGGER_PIN, 1);
    busy_wait_us(10);
    gpio_put(TRIGGER_PIN, 0);
    
    // Wait for echo to go high
    uint32_t timeout_start = time_us_32();
    while (!gpio_get(ECHO_PIN)) {
        if (time_us_32() - timeout_start > 30000) {  // 30ms timeout
            printf("[DEBUG] Timeout waiting for echo start\n");
            return -1;
        }
    }
    start_time = time_us_64();
    
    // Wait for echo to go low
    timeout_start = time_us_32();
    while (gpio_get(ECHO_PIN)) {
        if (time_us_32() - timeout_start > 30000) {  // 30ms timeout
            printf("[DEBUG] Timeout waiting for echo end\n");
            return -1;
        }
    }
    end_time = time_us_64();
    
    return (float)(end_time - start_time);
}

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
    uint32_t measurement_count = 0;
    
    while (1) {
        measurement_count++;
        printf("[DEBUG] Starting ultrasonic measurement #%lu\n", measurement_count);
        
        // Get pulse duration
        float duration = get_pulse_duration();
        
        // Calculate distance
        if (duration > 0) {
            distance = (duration * 0.0343) / 2.0;
            if (distance > MAX_DISTANCE) {
                distance = -1.0f;
                printf("[DEBUG] Distance exceeds max range\n");
            } else {
                printf("[DEBUG] Valid distance measured: %.2f cm (duration: %.2f us)\n", distance, duration);
            }
        } else {
            distance = -1.0f;
            printf("[DEBUG] Invalid measurement\n");
        }
        
        // Update distance queue
        float prev_distance;
        while (xQueueReceive(distance_queue, &prev_distance, 0) == pdTRUE);
        xQueueSend(distance_queue, &distance, 0);

        // Send to UDP queue
        bool obstacle = (distance > 0 && distance <= 30.0f);
        snprintf(message, sizeof(message), "ULTRA:DIST=%.2f,OBS=%d", distance, obstacle);
        if (xQueueSend(xServerQueue, &message, 0) != pdTRUE) {
            printf("[DEBUG] Failed to send ultrasonic data to server queue\n");
        } else {
            printf("[DEBUG] Sent ultrasonic data: Distance=%.2f cm, Obstacle=%d\n", distance, obstacle);
        }
        
        // Run every 200ms
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
    
    // Initialize GPIO
    gpio_init(TRIGGER_PIN);
    gpio_init(ECHO_PIN);
    gpio_set_dir(TRIGGER_PIN, GPIO_OUT);
    gpio_set_dir(ECHO_PIN, GPIO_IN);
    
    // Start with trigger pin low
    gpio_put(TRIGGER_PIN, 0);
    printf("[DEBUG] GPIO initialized (Trigger: %d, Echo: %d)\n", TRIGGER_PIN, ECHO_PIN);
    
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
