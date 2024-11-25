/*******************************************************************************
 * File: receiver.c
 *
 * Description:
 * UDP server implementation for Raspberry Pi Pico W that receives and processes
 * sensor data from various sources (barcode scanner, wheel encoder, ultrasonic).
 * Uses FreeRTOS for task management and lwIP for network communication.
 *
 * Hardware: Raspberry Pi Pico W
 *
 * Network Configuration:
 * - WiFi SSID: liangfannn
 * - UDP Port: 12340
 * - Protocol: UDP (SOCK_DGRAM)
 *
 * Message Format:
 * - Barcode data: "BARCODE:<data>"
 * - Wheel encoder: "WHEEL:<data>"
 * - Ultrasonic: "ULTRA:<data>"
 *
 * Dependencies:
 * - Pico SDK (stdlib, cyw43_arch)
 * - lwIP stack (sockets, netif)
 * - FreeRTOS
 ******************************************************************************/

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/sockets.h"
#include "lwip/netif.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <stdio.h>

// Network configuration constants
#define WIFI_SSID "liangfannn"              // WiFi Network Name
#define WIFI_PASSWORD "saypleasethankyou"   // WiFi Network Password
#define UDP_PORT 12340                      // UDP Port for server
#define MAX_BUFFER_SIZE 1024                // Maximum buffer size for incoming messages

/**
 * @brief Prints the server's IP address and UDP port
 *
 * Retrieves and displays the IP address from the default network interface
 * and the configured UDP port. Used for connection verification.
 */
static void print_ip_address() {
    struct netif *netif = netif_default;
    if (netif != NULL) {
        printf("\nServer IP Address: %s\n", ip4addr_ntoa(netif_ip4_addr(netif)));
        printf("UDP Port: %d\n\n", UDP_PORT);
    }
}

/**
 * @brief Creates and initializes a UDP server socket
 *
 * Creates a UDP socket and binds it to all available interfaces (INADDR_ANY)
 * on the specified UDP_PORT.
 *
 * @return int Socket file descriptor if successful, -1 on error
 */
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

    return sock;
}

/**
 * @brief Processes received sensor data based on message prefix
 *
 * Parses and displays received messages based on their prefix:
 * - "BARCODE:" - Data from barcode scanner
 * - "WHEEL:" - Data from wheel encoder
 * - "ULTRA:" - Data from ultrasonic sensor
 *
 * @param data Null-terminated string containing the received message
 */
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
        printf("\n%s\n", data);
    }
}

/**
 * @brief FreeRTOS task that runs the UDP server
 *
 * Main server task that:
 * 1. Initializes WiFi and connects to the network
 * 2. Creates and configures the UDP server socket
 * 3. Enters receive loop to process incoming messages
 *
 * @param params Task parameters (unused)
 */
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

/**
 * @brief Program entry point
 *
 * Initializes the system and creates the UDP server task:
 * 1. Initializes stdio for debug output
 * 2. Creates UDP server task with required stack size
 * 3. Starts the FreeRTOS scheduler
 *
 * @return int Never returns under normal operation
 */
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
