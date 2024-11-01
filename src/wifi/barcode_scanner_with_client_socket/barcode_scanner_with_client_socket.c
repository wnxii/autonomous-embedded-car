#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include <lwip/sockets.h>
#include <stdio.h>
#include <string.h>
#include "../../ir_sensor/barcode_scanner/barcode_scanner.h"  // Include the barcode scanner header

// Wi-Fi and server configurations
#define WIFI_SSID "liangfannn"
#define WIFI_PASSWORD "saypleasethankyou"
#define SERVER_IP "172.20.10.2"
#define SERVER_PORT 12345
#define TEST_TASK_PRIORITY (tskIDLE_PRIORITY + 2UL)

// Send message over socket
static void send_message(int socket, const char *msg) {
    int len = strlen(msg);
    int sent = 0;
    while (sent < len) {
        int bytes = send(socket, msg + sent, len - sent, 0);
        if (bytes <= 0) {
            printf("Connection closed\n");
            return;
        }
        sent += bytes;
    }
}

// Function to connect to the server
static int connect_to_server() {
    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    struct sockaddr_in server_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(SERVER_PORT),
    };
    inet_aton(SERVER_IP, &server_addr.sin_addr);

    if (sock < 0) {
        printf("Unable to create socket: error %d\n", errno);
        return -1;
    }

    printf("Connecting to server at %s:%d\n", SERVER_IP, SERVER_PORT);
    if (connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        printf("Connection to server failed. Error: %d\n", errno);
        closesocket(sock);
        return -1;
    }

    printf("Connected to server!\n");
    return sock;
}

// Client task to send barcode data to the server
void client_task(__unused void *params) {
    if (cyw43_arch_init()) {
        printf("WiFi init failed\n");
        return;
    }

    cyw43_arch_enable_sta_mode();
    printf("Connecting to WiFi...\n");

    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("Failed to connect to WiFi\n");
        return;
    }
    printf("Connected to WiFi\n");

    int sock = connect_to_server();
    if (sock < 0) {
        printf("Failed to connect to the server\n");
        return;
    }

    // Initialize barcode scanning tasks
    vInitializeTasks();

    char message[200];
    while (true) {
        if (xQueueReceive(xServerQueue, &message, portMAX_DELAY)) {
            send_message(sock, message);
            printf("Sent: %s", message);
        }
    }

    closesocket(sock);
    cyw43_arch_deinit();
}

int main() {
    stdio_init_all();
    TaskHandle_t task_handle;
    xTaskCreate(client_task, "client_task", 4096, NULL, TEST_TASK_PRIORITY, &task_handle);
    vTaskStartScheduler();
    return 0;
}
