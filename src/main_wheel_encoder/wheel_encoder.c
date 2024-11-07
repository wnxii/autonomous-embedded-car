#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "wheel_encoder.h"
#include <stdio.h> // Include for printf function

// FreeRTOS handles
// static QueueHandle_t encoder_queue;
// static SemaphoreHandle_t data_mutex;

// Separate queues for each encoder
/* static QueueHandle_t left_encoder_queue;
static QueueHandle_t right_encoder_queue;

// Mutexes for each encoder
static SemaphoreHandle_t left_data_mutex;
static SemaphoreHandle_t right_data_mutex; */

// Global variables protected by mutex
// static volatile EncoderData left_data = {0, 0};
// static volatile EncoderData right_data = {0, 0};
// static volatile EncoderData current_data = {0, 0};

volatile EncoderData left_data = {0, 0};
volatile EncoderData right_data = {0, 0};
static EncoderData left_last_data = {0, 0};
static EncoderData right_last_data = {0, 0};

SemaphoreHandle_t left_data_mutex;
SemaphoreHandle_t right_data_mutex;

static QueueHandle_t left_encoder_queue;
static QueueHandle_t right_encoder_queue;

/* void encoder_callback(uint gpio, uint32_t events) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    if (xSemaphoreTakeFromISR(data_mutex, &xHigherPriorityTaskWoken) == pdTRUE) {
        current_data.pulse_count++;
        current_data.timestamp = time_us_64();
        printf("Pulse count incremented to: %lu\n", current_data.pulse_count);
        xSemaphoreGiveFromISR(data_mutex, &xHigherPriorityTaskWoken);
    }
    
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
} */

// Callback for left encoder
/* void left_encoder_callback(uint gpio, uint32_t events) {
     printf("Left encoder callback triggered - GPIO %d\n", gpio); // Add this line
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (xSemaphoreTakeFromISR(left_data_mutex, &xHigherPriorityTaskWoken) == pdTRUE) {
        left_data.pulse_count++;
        left_data.timestamp = time_us_64();
        xSemaphoreGiveFromISR(left_data_mutex, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// Callback for right encoder
void right_encoder_callback(uint gpio, uint32_t events) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (xSemaphoreTakeFromISR(right_data_mutex, &xHigherPriorityTaskWoken) == pdTRUE) {
        right_data.pulse_count++;
        right_data.timestamp = time_us_64();
        xSemaphoreGiveFromISR(right_data_mutex, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
} */

// Unified callback for both left and right encoder interrupts
/* void encoder_callback(uint gpio, uint32_t events) {
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
} */


/* void encoder_task(void *params) {
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
} */

// Task to handle the left encoder
void left_encoder_task(void *params) {
    EncoderData data, last_sent = {0, 0};
    TickType_t last_wake_time = xTaskGetTickCount();

    while (1) {
        if (xSemaphoreTake(left_data_mutex, portMAX_DELAY) == pdTRUE) {
            data = left_data;
            xSemaphoreGive(left_data_mutex);

            // Only send if data has changed
            if (data.pulse_count != last_sent.pulse_count) {
                xQueueReset(left_encoder_queue);
                if (xQueueSendToBack(left_encoder_queue, &data, 0) == pdTRUE) {
                    // printf("Left encoder - Count: %lu, Timestamp: %llu\n", data.pulse_count, data.timestamp);
                    last_sent = data;
                }  else {
                    printf("Left encoder queue send failed\n");
                }
            }
        }
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10));  // Adjust frequency as needed
    }
}

// Task to handle the right encoder
void right_encoder_task(void *params) {
    EncoderData data, last_sent = {0, 0};
    TickType_t last_wake_time = xTaskGetTickCount();
    
    while (1) {
        if (xSemaphoreTake(right_data_mutex, portMAX_DELAY) == pdTRUE) {
            data = right_data;
            xSemaphoreGive(right_data_mutex);

            // Only send if data has changed
            if (data.pulse_count != last_sent.pulse_count) {
                xQueueReset(right_encoder_queue);
                if (xQueueSendToBack(right_encoder_queue, &data, 0) == pdTRUE) {
                    // printf("Right encoder - Count: %lu, Timestamp: %llu\n", data.pulse_count, data.timestamp);
                    last_sent = data;
                } else {
                    printf("Right encoder queue send failed\n");
                }
            }
        }
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10)); // Adjust frequency as needed
    }
}

/* void init_wheel_encoder() {
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
} */

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
    
     // Register the global callback once
    // gpio_set_irq_enabled_with_callback(RIGHT_ENCODER_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &right_encoder_callback);
    // gpio_set_irq_enabled_with_callback(LEFT_ENCODER_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_callback);
    // Create FreeRTOS objects
    left_encoder_queue = xQueueCreate(1, sizeof(EncoderData));
    right_encoder_queue = xQueueCreate(1, sizeof(EncoderData));

    left_data_mutex = xSemaphoreCreateMutex();
    right_data_mutex = xSemaphoreCreateMutex();

    // printf("Initializing Left Encoder on GPIO %d\n", LEFT_ENCODER_PIN);
    // printf("Initializing Right Encoder on GPIO %d\n", RIGHT_ENCODER_PIN);

    // Create separate tasks for each encoder
    xTaskCreate(left_encoder_task, "Left Encoder Task", configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(right_encoder_task, "Right Encoder Task", configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 1, NULL);
    /* if (xTaskCreate(left_encoder_task, "Left Encoder Task", configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
        printf("Failed to create Left Encoder Task!\n");
    } */
    /* if (xTaskCreate(right_encoder_task, "Right Encoder Task", configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
        printf("Failed to create Right Encoder Task!\n");
    } */

}


/* float get_distance() {
    EncoderData data;
    float distance = 0.0f;
    
    if (xQueuePeek(encoder_queue, &data, 0) == pdTRUE) {
        float revolutions = (float)data.pulse_count / PULSES_PER_REVOLUTION;
        distance = revolutions * WHEEL_CIRCUMFERENCE;
        printf("Calculating distance - Pulses: %lu, Revolutions: %.2f, Distance: %.2f\n",
               data.pulse_count, revolutions, distance);
    }
    
    return distance;
} */

// Function to get distance for a specific encoder
float get_left_distance() {
    EncoderData data;
    float distance = 0.0f;

    if (xQueuePeek(left_encoder_queue, &data, 0) == pdTRUE) {
        // float revolutions = (float)data.pulse_count / PULSES_PER_REVOLUTION;
        // distance = revolutions * WHEEL_CIRCUMFERENCE;
        float distance_per_pulse = WHEEL_CIRCUMFERENCE / PULSES_PER_REVOLUTION;
        distance = distance_per_pulse * (float)data.pulse_count;
        // distance = encoder_distance * SCALING_FACTOR; // Adjusted for actual wheel distance
        /* printf("Left Distance - Pulses: %lu, Revolutions: %.2f, Distance: %.2f cm\n",
               data.pulse_count, revolutions, distance); */
        printf("Left Distance - Pulses: %lu, Distance Per Hole: %.2f, Distance: %.2f cm\n",
               data.pulse_count, distance_per_pulse, distance);
    }

    return distance;
}

float get_right_distance() {
    EncoderData data;
    float distance = 0.0f;

    if (xQueuePeek(right_encoder_queue, &data, 0) == pdTRUE) {
        // float revolutions = (float)data.pulse_count / PULSES_PER_REVOLUTION;
        // distance = revolutions * WHEEL_CIRCUMFERENCE;
        float distance_per_pulse = WHEEL_CIRCUMFERENCE / PULSES_PER_REVOLUTION;
        distance = distance_per_pulse * (float)data.pulse_count;
        // distance = encoder_distance * SCALING_FACTOR; // Adjusted for actual wheel distance
        /* printf("Right Distance - Pulses: %lu, Revolutions: %.2f, Distance: %.2f cm\n",
               data.pulse_count, revolutions, distance); */
        printf("Right Distance - Pulses: %lu, Distance Per Hole: %.2f, Distance: %.2f cm\n",
               data.pulse_count, distance_per_pulse, distance );
    } 
    return distance;
}


/* float get_speed() {
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
 */


// Function to get speed for a specific encoder
float get_left_speed() {
    EncoderData current;
    float speed = 0.0f;

    if (xQueuePeek(left_encoder_queue, &current, 0) == pdTRUE) {
        if (current.pulse_count != left_last_data.pulse_count) {  // Only calculate if count changed
            float time_diff = (current.timestamp - left_last_data.timestamp) / 1000000.0f; // Convert to seconds
            float count_diff = current.pulse_count - left_last_data.pulse_count;

            if (time_diff > 0) {
                // float revolutions = count_diff / PULSES_PER_REVOLUTION;
                // speed = (revolutions * WHEEL_CIRCUMFERENCE) / time_diff;
                float distance_per_pulse = WHEEL_CIRCUMFERENCE / PULSES_PER_REVOLUTION;
                speed = (distance_per_pulse * count_diff) / time_diff;  // Speed in cm/s
                printf("Left Speed: %.2f cm/s (count diff: %.1f, time diff: %.6f s)\n", 
                       speed, count_diff, time_diff);
            }

            left_last_data = current;
        } else {
            printf("No pulse count change detected for left encoder\n");
        }
    } else {
        printf("No data available in left encoder queue\n");
    }


    return speed;
}

float get_right_speed() {
    EncoderData current;
    float speed = 0.0f;

    if (xQueuePeek(right_encoder_queue, &current, 0) == pdTRUE) {
        if (current.pulse_count != right_last_data.pulse_count) {  // Only calculate if count changed
            float time_diff = (current.timestamp - right_last_data.timestamp) / 1000000.0f; // Convert to seconds
            float count_diff = current.pulse_count - right_last_data.pulse_count;

            if (time_diff > 0) {
                float revolutions = count_diff / PULSES_PER_REVOLUTION;
                speed = (revolutions * WHEEL_CIRCUMFERENCE) / time_diff;
                printf("Right Speed: %.2f cm/s (count diff: %.1f, time diff: %.6f s)\n", 
                       speed, count_diff, time_diff);
            } 

            right_last_data = current;
        } else {
            printf("No pulse count change detected for right encoder\n");
        }
    } else {
        printf("No data available in right encoder queue\n");
    }

    return speed;
}



/* void reset_encoder() {
    if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
        current_data.pulse_count = 0;
        current_data.timestamp = 0;
        printf("Encoder reset - count and timestamp zeroed\n");
        xSemaphoreGive(data_mutex);
        
        // Clear the queue
        xQueueReset(encoder_queue);
    }
} */

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
