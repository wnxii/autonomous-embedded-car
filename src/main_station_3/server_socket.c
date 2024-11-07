#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include <lwip/sockets.h>
#include "lwip/apps/lwiperf.h"
#include "lwip/ip4_addr.h"
#include "lwip/netif.h"
#include "FreeRTOS.h"
#include "task.h"

#define TEST_TASK_PRIORITY (tskIDLE_PRIORITY + 2UL)
#define WIFI_SSID ""
#define WIFI_PASSWORD ""

// Function to receive data from accelerometer
bool receive_data_with_timing(int client_socket) {
    char buffer[512];
    int bytes_received;

    while (true)
    {
        // Receive data from the client
        bytes_received = recv(client_socket, buffer, sizeof(buffer) - 1, 0);
        if (bytes_received > 0)
        {
            buffer[bytes_received] = '\0'; // Null-terminate the received string
            printf("%s", buffer);
        }
        else if (bytes_received == 0)
        {
            // Client has disconnected
            printf("Client disconnected.\n");
            return false;
        }
        else
        {
            // An error occurred
            printf("Error receiving data: %d\n", errno);
            return false;
        }
    }
}

// Function to run the TCP server
void run_server()
{
    int server_sock = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in listen_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(12345),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    if (server_sock < 0)
    {
        printf("Unable to create socket: error %d\n", errno);
        return;
    }

    if (bind(server_sock, (struct sockaddr *)&listen_addr, sizeof(listen_addr)) < 0)
    {
        printf("Unable to bind socket: error %d\n", errno);
        return;
    }

    if (listen(server_sock, 1) < 0)
    {
        printf("Unable to listen on socket: error %d\n", errno);
        return;
    }

    printf("Starting server at %s on port %u\n", ip4addr_ntoa(netif_ip4_addr(netif_list)), ntohs(listen_addr.sin_port));

    while (true)
    {
        struct sockaddr_storage remote_addr;
        socklen_t len = sizeof(remote_addr);
        int conn_sock = accept(server_sock, (struct sockaddr *)&remote_addr, &len);
        if (conn_sock < 0)
        {
            printf("Unable to accept incoming connection: error %d\n", errno);
            continue;
        }

        printf("Client connected.\n");
        printf("Connection from %s\n", ip4addr_ntoa((ip4_addr_t *)&((struct sockaddr_in *)&remote_addr)->sin_addr));

        if (!receive_data_with_timing(conn_sock))
        {
            // Client disconnected or an error occurred
            closesocket(conn_sock);
            continue;
        }

        closesocket(conn_sock);
    }
}

static void main_task()
{
  if (cyw43_arch_init()) {
    printf("failed to initialise\n");
    return;
  }

  cyw43_arch_enable_sta_mode();

  printf("Connecting to WiFi...\n");

  if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
    printf("failed to connect.\n");
    exit(1);
  }
  else {
    printf("Connected.\n");
  }

  run_server();
  cyw43_arch_deinit();
}

int main(void) {
  stdio_init_all();
  TaskHandle_t task_handle;
  xTaskCreate(main_task, "main_task", 4096, NULL, TEST_TASK_PRIORITY, &task_handle);
  vTaskStartScheduler();

  return 0;
}