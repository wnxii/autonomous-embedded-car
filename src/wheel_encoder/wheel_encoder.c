#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "wheel_encoder.h"
#include "../wifi/barcode_client_socket/barcode_client_socket.h"
#include <stdio.h> 

// Global variables protected by mutex
volatile EncoderData left_data = {0, 0};
volatile EncoderData right_data = {0, 0};
static EncoderData left_last_data = {0, 0};
static EncoderData right_last_data = {0, 0};

// Mutexes for each encoder
SemaphoreHandle_t left_data_mutex;
SemaphoreHandle_t right_data_mutex;

// Separate queues for each encoder
static QueueHandle_t left_encoder_queue;
static QueueHandle_t right_encoder_queue;

// External queue declaration
extern QueueHandle_t xServerQueue;

// Task to handle the left encoder
void left_encoder_task(void *params) {
    printf("[DEBUG] Starting left encoder task\n");
    
    if (!xServerQueue) {
        printf("[ERROR] Server queue not initialized\n");
        vTaskDelete(NULL);
        return;
    }

    EncoderData data;
    TickType_t last_wake_time = xTaskGetTickCount();
    char message[100];

    while (1) {
        if (xSemaphoreTake(left_data_mutex, portMAX_DELAY) == pdTRUE) {
            data = left_data;
            xSemaphoreGive(left_data_mutex);

            // Send data periodically
            xQueueReset(left_encoder_queue);
            if (xQueueSendToBack(left_encoder_queue, &data, pdMS_TO_TICKS(100)) == pdTRUE) {
                float speed = get_left_speed();
                float distance = get_left_distance();
                snprintf(message, sizeof(message), "WHEEL:L_SPD=%.2f,L_DIST=%.2f", speed, distance);
                if (xQueueSend(xServerQueue, &message, 0) != pdTRUE) {
                    printf("[DEBUG] Failed to send left encoder data to server queue\n");
                } else {
                    printf("[DEBUG] Left encoder data sent: Speed=%.2f cm/s, Distance=%.2f cm\n", speed, distance);
                }
            } else {
                printf("[DEBUG] Failed to send to left encoder queue\n");
            }
        }
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(100)); // Send every 100ms
    }
}

// Task to handle the right encoder
void right_encoder_task(void *params) {
    printf("[DEBUG] Starting right encoder task\n");
    
    if (!xServerQueue) {
        printf("[ERROR] Server queue not initialized\n");
        vTaskDelete(NULL);
        return;
    }

    EncoderData data;
    TickType_t last_wake_time = xTaskGetTickCount();
    char message[100];
    
    while (1) {
        if (xSemaphoreTake(right_data_mutex, portMAX_DELAY) == pdTRUE) {
            data = right_data;
            xSemaphoreGive(right_data_mutex);

            // Send data periodically
            xQueueReset(right_encoder_queue);
            if (xQueueSendToBack(right_encoder_queue, &data, pdMS_TO_TICKS(100)) == pdTRUE) {
                float speed = get_right_speed();
                float distance = get_right_distance();
                snprintf(message, sizeof(message), "WHEEL:R_SPD=%.2f,R_DIST=%.2f", speed, distance);
                if (xQueueSend(xServerQueue, &message, 0) != pdTRUE) {
                    printf("[DEBUG] Failed to send right encoder data to server queue\n");
                } else {
                    printf("[DEBUG] Right encoder data sent: Speed=%.2f cm/s, Distance=%.2f cm\n", speed, distance);
                }
            } else {
                printf("[DEBUG] Failed to send to right encoder queue\n");
            }
        }
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(100)); // Send every 100ms
    }
}

// Initialization function for both encoders
void init_wheel_encoders() {
    printf("[DEBUG] Initializing wheel encoders\n");

    // Initialize left encoder
    left_data.pulse_count = 0;
    left_data.timestamp = 0;
    gpio_init(LEFT_ENCODER_PIN);
    gpio_set_dir(LEFT_ENCODER_PIN, GPIO_IN);
    gpio_pull_up(LEFT_ENCODER_PIN);

    // Initialize right encoder
    right_data.pulse_count = 0;
    right_data.timestamp = 0;
    gpio_init(RIGHT_ENCODER_PIN);
    gpio_set_dir(RIGHT_ENCODER_PIN, GPIO_IN);
    gpio_pull_up(RIGHT_ENCODER_PIN);
    
    // Create FreeRTOS objects
    left_encoder_queue = xQueueCreate(1, sizeof(EncoderData));
    if (left_encoder_queue == NULL) {
        printf("[ERROR] Failed to create left encoder queue\n");
        return;
    }

    right_encoder_queue = xQueueCreate(1, sizeof(EncoderData));
    if (right_encoder_queue == NULL) {
        printf("[ERROR] Failed to create right encoder queue\n");
        return;
    }

    left_data_mutex = xSemaphoreCreateMutex();
    if (left_data_mutex == NULL) {
        printf("[ERROR] Failed to create left data mutex\n");
        return;
    }

    right_data_mutex = xSemaphoreCreateMutex();
    if (right_data_mutex == NULL) {
        printf("[ERROR] Failed to create right data mutex\n");
        return;
    }

    // Create separate tasks for each encoder
    TaskHandle_t left_task_handle = NULL;
    TaskHandle_t right_task_handle = NULL;

    xTaskCreate(left_encoder_task, "Left Encoder Task", configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 1, &left_task_handle);
    if (left_task_handle == NULL) {
        printf("[ERROR] Failed to create left encoder task\n");
        return;
    }

    xTaskCreate(right_encoder_task, "Right Encoder Task", configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 1, &right_task_handle);
    if (right_task_handle == NULL) {
        printf("[ERROR] Failed to create right encoder task\n");
        return;
    }

    printf("[DEBUG] Wheel encoders initialized successfully\n");
}

// Function to get distance for left encoder
float get_left_distance() {
    if (!left_encoder_queue) {
        printf("[ERROR] Left encoder queue not initialized\n");
        return 0.0f;
    }

    EncoderData data;
    float distance = 0.0f;

    if (xQueuePeek(left_encoder_queue, &data, pdMS_TO_TICKS(100)) == pdTRUE) {
        float distance_per_pulse = WHEEL_CIRCUMFERENCE / PULSES_PER_REVOLUTION;
        distance = distance_per_pulse * (float)data.pulse_count;
        printf("[DEBUG] Left Distance - Pulses: %lu, Distance Per Hole: %.2f, Distance: %.2f cm\n",
               data.pulse_count, distance_per_pulse, distance);
    } else {
        printf("[DEBUG] Failed to read from left encoder queue\n");
    }

    return distance;
}

// Function to get distance for right encoder
float get_right_distance() {
    if (!right_encoder_queue) {
        printf("[ERROR] Right encoder queue not initialized\n");
        return 0.0f;
    }

    EncoderData data;
    float distance = 0.0f;

    if (xQueuePeek(right_encoder_queue, &data, pdMS_TO_TICKS(100)) == pdTRUE) {
        float distance_per_pulse = WHEEL_CIRCUMFERENCE / PULSES_PER_REVOLUTION;
        distance = distance_per_pulse * (float)data.pulse_count;
        printf("[DEBUG] Right Distance - Pulses: %lu, Distance Per Hole: %.2f, Distance: %.2f cm\n",
               data.pulse_count, distance_per_pulse, distance);
    } else {
        printf("[DEBUG] Failed to read from right encoder queue\n");
    }
    return distance;
}

// Function to get speed for a left encoder
float get_left_speed() {
    if (!left_encoder_queue) {
        printf("[ERROR] Left encoder queue not initialized\n");
        return 0.0f;
    }

    EncoderData current;
    float speed = 0.0f;

    if (xQueuePeek(left_encoder_queue, &current, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (current.pulse_count != left_last_data.pulse_count) {  // Only calculate if count changed
            float time_diff = (current.timestamp - left_last_data.timestamp) / 1000000.0f; // Convert to seconds
            float count_diff = current.pulse_count - left_last_data.pulse_count;

            if (time_diff > 0) {
                float distance_per_pulse = WHEEL_CIRCUMFERENCE / PULSES_PER_REVOLUTION;
                speed = (distance_per_pulse * count_diff) / time_diff;  // Speed in cm/s
                printf("[DEBUG] Left Speed: %.2f cm/s (count diff: %.1f, time diff: %.6f s)\n", 
                        speed, count_diff, time_diff);
            }

            left_last_data = current;
        }
    } else {
        printf("[DEBUG] Failed to read from left encoder queue for speed calculation\n");
    }

    return speed;
}

// Function to get speed for right encoder
float get_right_speed() {
    if (!right_encoder_queue) {
        printf("[ERROR] Right encoder queue not initialized\n");
        return 0.0f;
    }

    EncoderData current;
    float speed = 0.0f;

    if (xQueuePeek(right_encoder_queue, &current, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (current.pulse_count != right_last_data.pulse_count) {  // Only calculate if count changed
            float time_diff = (current.timestamp - right_last_data.timestamp) / 1000000.0f; // Convert to seconds
            float count_diff = current.pulse_count - right_last_data.pulse_count;

            if (time_diff > 0) {
                float distance_per_pulse = WHEEL_CIRCUMFERENCE / PULSES_PER_REVOLUTION;
                speed = (distance_per_pulse * count_diff) / time_diff;  // Speed in cm/s
                printf("[DEBUG] Right Speed: %.2f cm/s (count diff: %.1f, time diff: %.6f s)\n", 
                        speed, count_diff, time_diff);
            } 

            right_last_data = current;
        }
    } else {
        printf("[DEBUG] Failed to read from right encoder queue for speed calculation\n");
    }

    return speed;
}

// Reset Left Encoder
void reset_left_encoder() {
    if (xSemaphoreTake(left_data_mutex, portMAX_DELAY) == pdTRUE) {
        left_data.pulse_count = 0;
        left_data.timestamp = 0;
        left_last_data.pulse_count = 0;
        left_last_data.timestamp = 0;
        printf("[DEBUG] Left Encoder reset - count and timestamp zeroed\n");
        xSemaphoreGive(left_data_mutex);

        if (left_encoder_queue) {
            xQueueReset(left_encoder_queue);
        }
    }
}

// Reset Right Encoder
void reset_right_encoder() {
    if (xSemaphoreTake(right_data_mutex, portMAX_DELAY) == pdTRUE) {
        right_data.pulse_count = 0;
        right_data.timestamp = 0;
        right_last_data.pulse_count = 0;
        right_last_data.timestamp = 0;
        printf("[DEBUG] Right Encoder reset - count and timestamp zeroed\n");
        xSemaphoreGive(right_data_mutex);

        if (right_encoder_queue) {
            xQueueReset(right_encoder_queue);
        }
    }
}
