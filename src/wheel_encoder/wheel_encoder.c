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

// Task to handle the left encoder
void left_encoder_task(void *params) {
    EncoderData data, last_sent = {0, 0};
    TickType_t last_wake_time = xTaskGetTickCount();
    char message[100];

    while (1) {
        if (xSemaphoreTake(left_data_mutex, portMAX_DELAY) == pdTRUE) {
            data = left_data;
            xSemaphoreGive(left_data_mutex);

            // Only send if data has changed
            if (data.pulse_count != last_sent.pulse_count) {
                xQueueReset(left_encoder_queue);
                if (xQueueSendToBack(left_encoder_queue, &data, 0) == pdTRUE) {
                    // Send to UDP client queue
                    float speed = get_left_speed();
                    float distance = get_left_distance();
                    snprintf(message, sizeof(message), "L_SPD=%.2f,L_DIST=%.2f", speed, distance);
                    xQueueSend(xWheelEncoderQueue, &message, 0);
                    last_sent = data;
                }
            }
        }
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10));
    }
}

// Task to handle the right encoder
void right_encoder_task(void *params) {
    EncoderData data, last_sent = {0, 0};
    TickType_t last_wake_time = xTaskGetTickCount();
    char message[100];
    
    while (1) {
        if (xSemaphoreTake(right_data_mutex, portMAX_DELAY) == pdTRUE) {
            data = right_data;
            xSemaphoreGive(right_data_mutex);

            // Only send if data has changed
            if (data.pulse_count != last_sent.pulse_count) {
                xQueueReset(right_encoder_queue);
                if (xQueueSendToBack(right_encoder_queue, &data, 0) == pdTRUE) {
                    // Send to UDP client queue
                    float speed = get_right_speed();
                    float distance = get_right_distance();
                    snprintf(message, sizeof(message), "R_SPD=%.2f,R_DIST=%.2f", speed, distance);
                    xQueueSend(xWheelEncoderQueue, &message, 0);
                    last_sent = data;
                }
            }
        }
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(100)); // Update every 100ms
    }
}

// Initialization function for both encoders
void init_wheel_encoders() {
    // Initialize left encoder
    left_data.pulse_count = 0;
    left_data.timestamp = 0;
    gpio_init(LEFT_ENCODER_PIN);
    gpio_set_dir(LEFT_ENCODER_PIN, GPIO_IN);
    gpio_pull_up(LEFT_ENCODER_PIN);
    gpio_set_irq_enabled(LEFT_ENCODER_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

    // Initialize right encoder
    right_data.pulse_count = 0;
    right_data.timestamp = 0;
    gpio_init(RIGHT_ENCODER_PIN);
    gpio_set_dir(RIGHT_ENCODER_PIN, GPIO_IN);
    gpio_pull_up(RIGHT_ENCODER_PIN);
    gpio_set_irq_enabled(RIGHT_ENCODER_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    
    // Create FreeRTOS objects
    left_encoder_queue = xQueueCreate(1, sizeof(EncoderData));
    right_encoder_queue = xQueueCreate(1, sizeof(EncoderData));

    left_data_mutex = xSemaphoreCreateMutex();
    right_data_mutex = xSemaphoreCreateMutex();

    // Create separate tasks for each encoder
    xTaskCreate(left_encoder_task, "Left Encoder Task", configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(right_encoder_task, "Right Encoder Task", configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 1, NULL);

}

// Function to get distance for left encoder
float get_left_distance() {
    EncoderData data;
    float distance = 0.0f;

    if (xQueuePeek(left_encoder_queue, &data, 0) == pdTRUE) {
        float distance_per_pulse = WHEEL_CIRCUMFERENCE / PULSES_PER_REVOLUTION;
        distance = distance_per_pulse * (float)data.pulse_count;
        // printf("Left Distance - Pulses: %lu, Distance Per Hole: %.2f, Distance: %.2f cm\n",
               // data.pulse_count, distance_per_pulse, distance);
    }

    return distance;
}

// Function to get distance for right encoder
float get_right_distance() {
    EncoderData data;
    float distance = 0.0f;

    if (xQueuePeek(right_encoder_queue, &data, 0) == pdTRUE) {
        float distance_per_pulse = WHEEL_CIRCUMFERENCE / PULSES_PER_REVOLUTION;
        distance = distance_per_pulse * (float)data.pulse_count;
        // printf("Right Distance - Pulses: %lu, Distance Per Hole: %.2f, Distance: %.2f cm\n",
               // data.pulse_count, distance_per_pulse, distance );
    } 
    return distance;
}

// Function to get speed for a left encoder
float get_left_speed() {
    EncoderData current;
    float speed = 0.0f;

    if (xQueuePeek(left_encoder_queue, &current, 0) == pdTRUE) {
        if (current.pulse_count != left_last_data.pulse_count) {  // Only calculate if count changed
            float time_diff = (current.timestamp - left_last_data.timestamp) / 1000000.0f; // Convert to seconds
            float count_diff = current.pulse_count - left_last_data.pulse_count;

            if (time_diff > 0) {
                float distance_per_pulse = WHEEL_CIRCUMFERENCE / PULSES_PER_REVOLUTION;
                speed = (distance_per_pulse * count_diff) / time_diff;  // Speed in cm/s
                // printf("Left Speed: %.2f cm/s (count diff: %.1f, time diff: %.6f s)\n", 
                        // speed, count_diff, time_diff);
            }

            left_last_data = current;
        } else {
            // printf("No pulse count change detected for left encoder\n");
        }
    } else {
        // printf("No data available in left encoder queue\n");
    }


    return speed;
}

// Function to get speed for right encoder
float get_right_speed() {
    EncoderData current;
    float speed = 0.0f;

    if (xQueuePeek(right_encoder_queue, &current, 0) == pdTRUE) {
        if (current.pulse_count != right_last_data.pulse_count) {  // Only calculate if count changed
            float time_diff = (current.timestamp - right_last_data.timestamp) / 1000000.0f; // Convert to seconds
            float count_diff = current.pulse_count - right_last_data.pulse_count;

            if (time_diff > 0) {
                float distance_per_pulse = WHEEL_CIRCUMFERENCE / PULSES_PER_REVOLUTION;
                speed = (distance_per_pulse * count_diff) / time_diff;  // Speed in cm/s
                // printf("Right Speed: %.2f cm/s (count diff: %.1f, time diff: %.6f s)\n", 
                        //speed, count_diff, time_diff);
            } 

            right_last_data = current;
        } else {
            // printf("No pulse count change detected for right encoder\n");
        }
    } else {
        // printf("No data available in right encoder queue\n");
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
        printf("Left Encoder reset - count and timestamp zeroed\n");
        xSemaphoreGive(left_data_mutex);

        // Clear the left encoder queue
        xQueueReset(left_encoder_queue);
    }
}

// Reset Right Encoder
void reset_right_encoder() {
    if (xSemaphoreTake(right_data_mutex, portMAX_DELAY) == pdTRUE) {
        right_data.pulse_count = 0;
        right_data.timestamp = 0;
        right_last_data.pulse_count = 0;
        right_last_data.timestamp = 0;
        printf("Right Encoder reset - count and timestamp zeroed\n");
        xSemaphoreGive(right_data_mutex);

        // Clear the right encoder queue
        xQueueReset(right_encoder_queue);
    }
}
