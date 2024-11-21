#ifndef BARCODE_CLIENT_SOCKET_H
#define BARCODE_CLIENT_SOCKET_H

#include "FreeRTOS.h"
#include "queue.h"

extern QueueHandle_t xWheelEncoderQueue;
extern QueueHandle_t xUltrasonicQueue;
extern volatile int remote_target_speed;
extern volatile int remote_steering;

void init_wifi(void);
void init_sensor_queues(void);  // New function to initialize queues early

#endif
