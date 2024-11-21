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
#define WIFI_SSID "liangfannn"
#define WIFI_PASSWORD "saypleasethankyou"
#define SERVER_IP "172.20.10.4"
#define SERVER_PORT 12346
#define TEST_TASK_PRIORITY (tskIDLE_PRIORITY + 2UL)

// Create queues for sensor data
QueueHandle_t xWheelEncoderQueue;
QueueHandle_t xUltrasonicQueue;

// Connection status
volatile bool connected = false;
int remote_target_speed = 0;
int remote_steering = 0;

// Helper function to unmap from one range to another
int unmap(int value, int in_min, int in_max, int out_min, int out_max) {
    long unmapped = (long)(value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    if (unmapped < out_min)
        return out_min;
    if (unmapped > out_max)
        return out_max;
    return (int)unmapped;
}

// Unmaps an ASCII value back to the original range
int unmap_from_ascii_range(int ascii_value, int ascii_min, int ascii_max, int out_min, int out_max, int ascii_neutral) {
    // If ASCII value is neutral, return 0
    if (ascii_value == ascii_neutral)
    {
        return 0;
    }

    // Determine if we're unmapping from the positive or negative range
    if (ascii_value > ascii_neutral)
    {
        // Handle positive range
        return unmap(ascii_value, ascii_neutral + 1, ascii_max, 1, out_max);
    }
    else
    {
        // Handle negative range
        return unmap(ascii_value, ascii_min, ascii_neutral - 1, out_min, -1);
    }
}

// Example usage in UDP receive callback:
void handle_received_controls(const char *data) {
    // Extract ASCII values
    int ascii_speed = (unsigned char)data[0];    // First byte for speed
    int ascii_steering = (unsigned char)data[1]; // Second byte for steering

    // Unmap speed (ASCII 0-40, neutral 20) back to -212 to 212
    remote_target_speed = unmap_from_ascii_range(ascii_speed, 1, 41, -20, 20, 21);

    // Unmap steering (ASCII 0-40, neutral 20) back to -80 to 80
    remote_steering = unmap_from_ascii_range(ascii_steering, 1, 41, -20, 20, 21);

    printf("Decoded values - Speed: %d, Steering: %d\n", remote_target_speed, remote_steering);

    // Now you can use speed and steering values to control your device
    // For example:
    // set_motor_speed(speed);
    // set_steering_angle(steering);
}

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
void wifi_task(__unused void *params) {
    if (cyw43_arch_init()) {
        printf("Failed to initialize WiFi\n");
        return;
    }

    cyw43_arch_enable_sta_mode();

    printf("Connecting to WiFi...\n");

    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("Failed to connect to WiFi.\n");
    }

    int dashboard_sock = create_udp_socket();
    if (dashboard_sock < 0) {
        printf("Failed to create Dashboard Client Socket\n");
    }

    int remote_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (remote_sock < 0) {
        printf("Failed to create Remote Control Socket: Error %d\n", errno);
    }

    struct sockaddr_in listen_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(12345),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    if (bind(remote_sock, (struct sockaddr *)&listen_addr, sizeof(listen_addr)) < 0) {
        printf("Failed to bind Remote Control Socket: Error %d\n", errno);
        closesocket(remote_sock);
    }

    printf("Starting UDP Remote Control Server at %s on port %u\n", ip4addr_ntoa(netif_ip4_addr(netif_list)), ntohs(listen_addr.sin_port));

    struct sockaddr_in remote_client_addr;
    socklen_t remote_client_addr_len = sizeof(remote_client_addr);
    char buffer[512];

    printf("Connecting to Dashboard Server...\n");
    // Set up server address
    struct sockaddr_in dashboard_server_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(SERVER_PORT),
    };
    inet_aton(SERVER_IP, &dashboard_server_addr.sin_addr);

    printf("Connected to Dashboard Server\n");
    send_message_udp(dashboard_sock, "Connection From Pico Car", &dashboard_server_addr);

    char message[200];
    char barcode_message[100];
    char wheel_message[100];
    char ultrasonic_message[100];

    while (true) {
        printf("Waiting for data from Remote Control Client\n");
        // Handle Receive Controls from Remote Control Pico
        int recv_len = recvfrom(remote_sock, buffer, sizeof(buffer) - 1, 0, (struct sockaddr *)&remote_client_addr, &remote_client_addr_len);
        if (recv_len < 2) {
            printf("Failed to receive message: error %d\n", errno);
            continue;
        }
        buffer[recv_len] = '\0'; // Null-terminate the received string
        handle_received_controls(buffer);

        printf("Sending Pico Car Data to Dashboard Server\n");

        // Handle barcode data
        if (xQueueReceive(xServerQueue, &barcode_message, 0)) {
            snprintf(message, sizeof(message), "BARCODE:%s", barcode_message);
            send_message_udp(dashboard_sock, message, &dashboard_server_addr);
            printf("Sent barcode data: %s\n", message);
        }

        // Handle wheel encoder data
        if (xQueueReceive(xWheelEncoderQueue, &wheel_message, 0)) {
            snprintf(message, sizeof(message), "WHEEL:%s", wheel_message);
            send_message_udp(dashboard_sock, message, &dashboard_server_addr);
            printf("Sent wheel encoder data: %s\n", message);
        }

        // Handle ultrasonic data
        if (xQueueReceive(xUltrasonicQueue, &ultrasonic_message, 0)) {
            snprintf(message, sizeof(message), "ULTRA:%s", ultrasonic_message);
            send_message_udp(dashboard_sock, message, &dashboard_server_addr);
            printf("Sent ultrasonic data: %s\n", message);
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // Small delay to prevent tight loop
    }

    closesocket(remote_sock);
    closesocket(dashboard_sock);
    cyw43_arch_deinit();
}

void init_wifi() {
    // Create queues
    xWheelEncoderQueue = xQueueCreate(5, sizeof(char[100]));
    xUltrasonicQueue = xQueueCreate(5, sizeof(char[100]));

    // Create client task
    TaskHandle_t client_handle;
    xTaskCreate(wifi_task, "client_task", 4096, NULL, TEST_TASK_PRIORITY, &client_handle);
}
