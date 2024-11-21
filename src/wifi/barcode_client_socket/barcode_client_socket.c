#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include <lwip/sockets.h>
#include <stdio.h>
#include <string.h>
#include "barcode_client_socket.h"
#include "../../ir_sensor/barcode_scanner/barcode_scanner.h"

// Wi-Fi and server configurations
#define WIFI_SSID "liangfannn"
#define WIFI_PASSWORD "saypleasethankyou"
#define SERVER_IP "172.20.10.5"
#define SERVER_PORT 12345
#define TEST_TASK_PRIORITY (tskIDLE_PRIORITY + 2UL)

// Create queues for sensor data
QueueHandle_t xWheelEncoderQueue;
QueueHandle_t xUltrasonicQueue;

// Connection status
volatile bool connected = false;

// Send message over UDP socket
static void send_message_udp(int socket, const char *msg, struct sockaddr_in *server_addr) {
    int len = strlen(msg);
    int sent = sendto(socket, msg, len, 0, (struct sockaddr *)server_addr, sizeof(*server_addr));
    if (sent < 0) {
        printf("Failed to send message. Error: %d\n", errno);
    } else {
        printf("Sent %d bytes via UDP\n", sent);
    }
}

// Function to create UDP socket
static int create_udp_socket() {
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        printf("Unable to create UDP socket: error %d\n", errno);
        return -1;
    }
    printf("UDP socket created\n");
    return sock;
}

// Client task to receive and send all sensor data to the server
void client_task(__unused void *params) {
    if (cyw43_arch_init()) {
        printf("WiFi init failed\n");
        connected = false;
        return;
    }

    cyw43_arch_enable_sta_mode();
    printf("Connecting to WiFi...\n");

    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("Failed to connect to WiFi\n");
        connected = false;
        return;
    }
    printf("Connected to WiFi\n");
    connected = true;

    int sock = create_udp_socket();
    if (sock < 0) {
        printf("Failed to create UDP socket\n");
        connected = false;
        return;
    }

    // Set up server address
    struct sockaddr_in server_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(SERVER_PORT),
    };
    inet_aton(SERVER_IP, &server_addr.sin_addr);

    char message[200];
    char wheel_message[100];
    char ultrasonic_message[100];

    while (true) {
        // Handle barcode data
        if (xQueueReceive(xServerQueue, &message, 0)) {
            snprintf(message, sizeof(message), "BARCODE:%s", message);
            send_message_udp(sock, message, &server_addr);
            printf("Sent barcode data: %s\n", message);
        }

        // Handle wheel encoder data
        if (xQueueReceive(xWheelEncoderQueue, &wheel_message, 0)) {
            snprintf(message, sizeof(message), "WHEEL:%s", wheel_message);
            send_message_udp(sock, message, &server_addr);
            printf("Sent wheel encoder data: %s\n", message);
        }

        // Handle ultrasonic data
        if (xQueueReceive(xUltrasonicQueue, &ultrasonic_message, 0)) {
            snprintf(message, sizeof(message), "ULTRA:%s", ultrasonic_message);
            send_message_udp(sock, message, &server_addr);
            printf("Sent ultrasonic data: %s\n", message);
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // Small delay to prevent tight loop
    }

    connected = false;
    closesocket(sock);
    cyw43_arch_deinit();
}

void init_barcode_wifi() {
    // Create queues
    xWheelEncoderQueue = xQueueCreate(5, sizeof(char[100]));
    xUltrasonicQueue = xQueueCreate(5, sizeof(char[100]));

    // Create client task
    TaskHandle_t client_handle;
    xTaskCreate(client_task, "client_task", 4096, NULL, TEST_TASK_PRIORITY, &client_handle);
}
