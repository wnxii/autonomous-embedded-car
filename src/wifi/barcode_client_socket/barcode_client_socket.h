#ifndef BARCODE_CLIENT_SOCKET_H
#define BARCODE_CLIENT_SOCKET_H

#include "FreeRTOS.h"
#include "queue.h"

// Queues for sensor data
extern QueueHandle_t xWheelEncoderQueue;
extern QueueHandle_t xUltrasonicQueue;
extern int remote_target_speed;
extern int remote_steering;

// Connection status
extern volatile bool connected;

void init_wifi();

#endif
