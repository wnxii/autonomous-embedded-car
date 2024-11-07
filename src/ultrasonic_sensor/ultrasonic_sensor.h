#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include "task.h"
#include "queue.h"
#include "semphr.h"

#define TRIGGER_PIN 6
#define ECHO_PIN 7
#define MAX_DISTANCE 400 // Maximum distance in cm
#define TIMEOUT_US 23200 // Timeout in microseconds (based on MAX_DISTANCE)

// Measurement data structure
typedef struct {
    uint64_t start_time;
    uint64_t end_time;
    bool measurement_done;
    bool waiting_for_echo;
} MeasurementData;

// Function declarations
void echo_isr(uint gpio, uint32_t events);
void ultrasonic_task(void *params);
void init_ultrasonic_sensor();
float measure_distance();
bool is_obstacle_detected(float safety_threshold);

extern volatile MeasurementData current_measurement;
extern SemaphoreHandle_t measurement_mutex;

#endif // ULTRASONIC_SENSOR_H
