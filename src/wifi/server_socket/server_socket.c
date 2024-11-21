#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include <lwip/sockets.h>
#include "lwip/apps/lwiperf.h"
#include "lwip/ip4_addr.h"
#include "lwip/netif.h"
#include "../../main/FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "server_socket.h"

#define TEST_TASK_PRIORITY (tskIDLE_PRIORITY + 2UL)

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

// Function to run the TCP server
void run_server() {
    int server_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (server_sock < 0) {
        printf("Unable to create socket: error %d\n", errno);
        return;
    }

    struct sockaddr_in listen_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(12345),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    if (bind(server_sock, (struct sockaddr *)&listen_addr, sizeof(listen_addr)) < 0) {
        printf("Unable to bind socket: error %d\n", errno);
        closesocket(server_sock);
        return;
    }

    printf("Starting UDP server at %s on port %u\n", ip4addr_ntoa(netif_ip4_addr(netif_list)), ntohs(listen_addr.sin_port));

    struct sockaddr_in client_addr;
    socklen_t client_addr_len = sizeof(client_addr);
    char buffer[512];

    while (true)
    {
        int recv_len = recvfrom(server_sock, buffer, sizeof(buffer) - 1, 0, (struct sockaddr *)&client_addr, &client_addr_len);
        if (recv_len < 2) {
            printf("Failed to receive message: error %d\n", errno);
            continue;
        }

        printf("Message Length: %d\n", recv_len);
        buffer[recv_len] = '\0'; // Null-terminate the received string
        // printf("%s", buffer);
        handle_received_controls(buffer);
    }

    closesocket(server_sock);
}

static void server_task(void *params) {
    run_server();
}

void init_server_socket(void) {
    printf("Initializing server socket...\n");
    xTaskCreate(server_task, "ServerTask", 4096, NULL, TEST_TASK_PRIORITY, NULL);
}

/* int main() {
    stdio_init_all();
    printf("Starting server socket\n");

    init_server_socket();

    vTaskStartScheduler();
    return 0;
} */