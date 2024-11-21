#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/sockets.h"
#include "lwip/netif.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <stdio.h>

// WIFI Credentials
#define WIFI_SSID "liangfannn"
#define WIFI_PASSWORD "saypleasethankyou"
#define UDP_PORT 12345
#define MAX_BUFFER_SIZE 1024

// Function to print IP address
static void print_ip_address() {
    struct netif *netif = netif_default;
    if (netif != NULL) {
        printf("\nServer IP Address: %s\n", ip4addr_ntoa(netif_ip4_addr(netif)));
        printf("UDP Port: %d\n\n", UDP_PORT);
    }
}

// Function to create and bind UDP socket
static int create_udp_server() {
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        printf("Failed to create socket: error %d\n", errno);
        return -1;
    }

    struct sockaddr_in server_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(UDP_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY)
    };

    if (bind(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        printf("Failed to bind socket: error %d\n", errno);
        closesocket(sock);
        return -1;
    }

    printf("UDP server listening on port %d\n", UDP_PORT);
    return sock;
}

// Function to handle received data
static void handle_received_data(const char *data) {
    if (strncmp(data, "BARCODE:", 8) == 0) {
        printf("\nReceived Barcode Data: %s\n", data + 8);
    }
    else if (strncmp(data, "WHEEL:", 6) == 0) {
        printf("\nReceived Wheel Encoder Data: %s\n", data + 6);
    }
    else if (strncmp(data, "ULTRA:", 6) == 0) {
        printf("\nReceived Ultrasonic Data: %s\n", data + 6);
    }
    else {
        printf("\nReceived Unknown Data: %s\n", data);
    }
}

// UDP Server Task
void udp_server_task(void *params) {
    printf("\nUDP Server Task Starting...\n");

    // Initialize WiFi
    if (cyw43_arch_init()) {
        printf("Failed to initialize WiFi\n");
        vTaskDelete(NULL);
        return;
    }

    cyw43_arch_enable_sta_mode();

    // Connect to WiFi
    printf("Connecting to WiFi...\n");
    while (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000) != 0) {
        printf("Failed to connect. Retrying...\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    printf("Connected to WiFi\n");

    // Print server IP address
    print_ip_address();

    // Create UDP server
    int sock = create_udp_server();
    if (sock < 0) {
        cyw43_arch_deinit();
        vTaskDelete(NULL);
        return;
    }

    char rx_buffer[MAX_BUFFER_SIZE];
    struct sockaddr_in client_addr;
    socklen_t addr_len = sizeof(client_addr);

    // Main receive loop
    while (1) {
        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, 
                          (struct sockaddr *)&client_addr, &addr_len);
        
        if (len < 0) {
            printf("recvfrom failed: error %d\n", errno);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // Null terminate received data
        rx_buffer[len] = 0;
        
        // Get client's IP address
        char addr_str[16];
        inet_ntoa_r(client_addr.sin_addr, addr_str, sizeof(addr_str) - 1);
        
        printf("\nReceived %d bytes from %s:%d\n", len, addr_str, ntohs(client_addr.sin_port));
        
        // Handle the received data
        handle_received_data(rx_buffer);
        
        // Small delay to prevent task from hogging CPU
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Should never reach here
    closesocket(sock);
    cyw43_arch_deinit();
    vTaskDelete(NULL);
}

int main() {
    stdio_init_all();
    sleep_ms(5000);  // Give time for USB serial to initialize
    printf("Starting UDP Server Application\n");

    // Create the UDP server task
    xTaskCreate(udp_server_task, "UDP_SERVER", configMINIMAL_STACK_SIZE * 4, NULL, 1, NULL);

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();

    // Should never reach here
    while(1) {
        tight_loop_contents();
    }

    return 0;
}
