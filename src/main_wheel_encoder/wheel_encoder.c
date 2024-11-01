#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#define ENCODER_PIN 4 // GPIO pin for the encoder
#define WHEEL_CIRCUMFERENCE 20.0f  // Wheel circumference in cm
#define PULSES_PER_REVOLUTION 20  // Number of pulses per wheel revolution

// FreeRTOS handles
static QueueHandle_t encoder_queue;
static SemaphoreHandle_t data_mutex;

// Encoder data structure
typedef struct {
    uint32_t pulse_count;
    uint64_t timestamp;
} EncoderData;

// Global variables protected by mutex
static volatile EncoderData current_data = {0, 0};

void encoder_callback(uint gpio, uint32_t events) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    if (xSemaphoreTakeFromISR(data_mutex, &xHigherPriorityTaskWoken) == pdTRUE) {
        current_data.pulse_count++;
        current_data.timestamp = time_us_64();
        printf("Pulse count incremented to: %lu\n", current_data.pulse_count);
        xSemaphoreGiveFromISR(data_mutex, &xHigherPriorityTaskWoken);
    }
    
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void encoder_task(void *params) {
    EncoderData data;
    EncoderData last_sent = {0, 0};
    TickType_t last_wake_time = xTaskGetTickCount();
    
    while (1) {
        if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
            // Only send if data has changed
            if (current_data.pulse_count != last_sent.pulse_count) {
                data = current_data;
                xSemaphoreGive(data_mutex);
                
                // Clear queue before sending new data
                xQueueReset(encoder_queue);
                
                if (xQueueSendToBack(encoder_queue, &data, 0) == pdTRUE) {
                    printf("Successfully sent to queue - Count: %lu, Timestamp: %llu\n", 
                           data.pulse_count, data.timestamp);
                    last_sent = data;
                } else {
                    printf("Failed to send to queue - will retry\n");
                }
            } else {
                xSemaphoreGive(data_mutex);
            }
        }
        
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(50));  // Increased frequency
    }
}

void init_wheel_encoder() {
    // Initialize the encoder data
    current_data.pulse_count = 0;
    current_data.timestamp = 0;
    
    // Create FreeRTOS objects with increased queue size
    encoder_queue = xQueueCreate(1, sizeof(EncoderData));  // Single item queue
    if (encoder_queue == NULL) {
        printf("Failed to create encoder queue!\n");
        return;
    }
    
    data_mutex = xSemaphoreCreateMutex();
    if (data_mutex == NULL) {
        printf("Failed to create mutex!\n");
        return;
    }
    
    // Initialize GPIO
    gpio_init(ENCODER_PIN);
    gpio_set_dir(ENCODER_PIN, GPIO_IN);
    gpio_pull_up(ENCODER_PIN);
    
    printf("GPIO %d initialized. Current state: %d\n", 
           ENCODER_PIN, gpio_get(ENCODER_PIN));
    
    gpio_set_irq_enabled_with_callback(ENCODER_PIN, 
                                     GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, 
                                     true, &encoder_callback);
    
    printf("IRQ enabled for GPIO %d\n", ENCODER_PIN);
    
    // Create encoder task with increased stack size
    BaseType_t task_created = xTaskCreate(encoder_task, "Encoder Task", 
                                        configMINIMAL_STACK_SIZE * 4, NULL, 
                                        tskIDLE_PRIORITY + 1, NULL);
    
    if (task_created != pdPASS) {
        printf("Failed to create encoder task!\n");
    } else {
        printf("Encoder task created successfully\n");
    }
}

float get_distance() {
    EncoderData data;
    float distance = 0.0f;
    
    if (xQueuePeek(encoder_queue, &data, 0) == pdTRUE) {
        float revolutions = (float)data.pulse_count / PULSES_PER_REVOLUTION;
        distance = revolutions * WHEEL_CIRCUMFERENCE;
        printf("Calculating distance - Pulses: %lu, Revolutions: %.2f, Distance: %.2f\n",
               data.pulse_count, revolutions, distance);
    }
    
    return distance;
}

float get_speed() {
    static EncoderData last_data = {0, 0};
    EncoderData current;
    float speed = 0.0f;
    
    if (xQueuePeek(encoder_queue, &current, 0) == pdTRUE) {
        if (current.pulse_count != last_data.pulse_count) {  // Only calculate if count changed
            float time_diff = (current.timestamp - last_data.timestamp) / 1000000.0f;
            float count_diff = current.pulse_count - last_data.pulse_count;
            
            if (time_diff > 0) {
                float revolutions = count_diff / PULSES_PER_REVOLUTION;
                speed = (revolutions * WHEEL_CIRCUMFERENCE) / time_diff;
                printf("Speed: %.2f cm/s (count diff: %.1f, time diff: %.6f)\n", 
                       speed, count_diff, time_diff);
            }
            
            last_data = current;
        }
    }
    
    return speed;
}

void reset_encoder() {
    if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
        current_data.pulse_count = 0;
        current_data.timestamp = 0;
        printf("Encoder reset - count and timestamp zeroed\n");
        xSemaphoreGive(data_mutex);
        
        // Clear the queue
        xQueueReset(encoder_queue);
    }
}