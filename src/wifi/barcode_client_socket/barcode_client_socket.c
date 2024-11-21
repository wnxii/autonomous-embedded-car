#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include <lwip/sockets.h>
#include "lwip/apps/lwiperf.h"
#include "lwip/ip4_addr.h"
#include "lwip/netif.h"
#include <stdio.h>
#include <string.h>
#include "barcode_client_socket.h"
#include "../../ir_sensor/barcode_scanner/barcode_scanner.h"

// Wi-Fi and server configurations
#define WIFI_SSID "yongjun"
#define WIFI_PASSWORD "pewpew1234"
#define SERVER_IP "172.20.10.3"
#define SERVER_PORT 12346
#define TEST_TASK_PRIORITY (tskIDLE_PRIORITY + 2UL)

// Global queue for all sensor data
QueueHandle_t xServerQueue = NULL;

// Define the global variables
volatile int remote_target_speed;
volatile int remote_steering;
volatile bool connected;

void handle_received_controls(const char *data) {
    int ascii_speed = (unsigned char)data[0];
    int ascii_steering = (unsigned char)data[1];
    remote_target_speed = ascii_speed - 21;
    remote_steering = ascii_steering - 21;
}

static void send_message_udp(int socket, const char *msg, struct sockaddr_in *server_addr) {
    if (!msg || strlen(msg) == 0) {
        printf("[DEBUG] Attempted to send empty message\n");
        return;
    }

    int len = strlen(msg);
    int result = sendto(socket, msg, len, 0, (struct sockaddr *)server_addr, sizeof(*server_addr));
    if (result < 0) {
        printf("[DEBUG] Failed to send UDP message: %d\n", result);
    } else {
        printf("[DEBUG] Sent UDP message (%d bytes): %s\n", result, msg);
    }
}

void wifi_task(__unused void *params) {
    remote_target_speed = 0;
    remote_steering = 0;
    connected = false;

    // Initialize WiFi
    printf("[DEBUG] Initializing WiFi...\n");
    if (cyw43_arch_init()) {
        printf("[ERROR] Failed to initialize cyw43_arch\n");
        return;
    }

    cyw43_arch_enable_sta_mode();
    printf("[DEBUG] Connecting to WiFi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("[ERROR] Failed to connect to WiFi\n");
        return;
    }
    printf("[DEBUG] Connected to WiFi successfully\n");

    // Create sockets
    int dashboard_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (dashboard_sock < 0) {
        printf("[ERROR] Failed to create dashboard socket\n");
        return;
    }
    printf("[DEBUG] Created dashboard socket\n");

    int remote_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (remote_sock < 0) {
        printf("[ERROR] Failed to create remote socket\n");
        closesocket(dashboard_sock);
        return;
    }
    printf("[DEBUG] Created remote socket\n");

    // Bind remote socket
    struct sockaddr_in listen_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(12345),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    if (bind(remote_sock, (struct sockaddr *)&listen_addr, sizeof(listen_addr)) < 0) {
        printf("[ERROR] Failed to bind remote socket\n");
        closesocket(remote_sock);
        closesocket(dashboard_sock);
        return;
    }
    printf("[DEBUG] Bound remote socket successfully\n");

    // Set up server address
    struct sockaddr_in dashboard_server_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(SERVER_PORT),
    };
    inet_aton(SERVER_IP, &dashboard_server_addr.sin_addr);

    // Send initial connection message
    send_message_udp(dashboard_sock, "Connection From Pico Car", &dashboard_server_addr);
    connected = true;
    printf("[DEBUG] Sent initial connection message\n");

    // Message buffers
    struct sockaddr_in remote_client_addr;
    socklen_t remote_client_addr_len = sizeof(remote_client_addr);
    char buffer[512];
    char message[200];

    // Set socket timeout
    struct timeval tv = {.tv_sec = 0, .tv_usec = 10000}; // 10ms timeout
    setsockopt(remote_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t sensor_check_period = pdMS_TO_TICKS(50); // Check sensors every 50ms

    while (true) {
        // Handle remote control messages
        int recv_len = recvfrom(remote_sock, buffer, sizeof(buffer) - 1, 0, 
                               (struct sockaddr *)&remote_client_addr, &remote_client_addr_len);
        if (recv_len >= 2) {
            buffer[recv_len] = '\0';
            handle_received_controls(buffer);
            printf("[DEBUG] Received control message\n");
        }

        // Handle sensor data from xServerQueue
        if (xQueueReceive(xServerQueue, &message, 0) == pdTRUE) {
            send_message_udp(dashboard_sock, message, &dashboard_server_addr);
        }

        // Wait for next period
        vTaskDelayUntil(&last_wake_time, sensor_check_period);
    }
}

void init_sensor_queues() {
    printf("[DEBUG] Creating server queue...\n");
    xServerQueue = xQueueCreate(10, sizeof(char[200]));
    if (xServerQueue == NULL) {
        printf("[ERROR] Failed to create server queue\n");
        return;
    }
    printf("[DEBUG] Created server queue successfully\n");
}

void init_wifi() {
    printf("[DEBUG] Creating WiFi task...\n");
    xTaskCreate(wifi_task, "client_task", 4096, NULL, TEST_TASK_PRIORITY, NULL);
    printf("[DEBUG] WiFi task created successfully\n");
}
