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

static void send_message(int socket, char *msg) {
  int len = strlen(msg);
  int done = 0;
  while (done < len) {
    // If sent is 0, the connection is closed
    // msg + done = dataptr --> pointer to the address of the data to be sent
    // len - done = len --> length of the data left to be sent
    int sent = send(socket, msg + done, len - done, 0);
    if (sent <= 0) {
      printf("Connection closed\n");
      return;
    }
    done += sent;
  }
}

/* static int handle_single_command(int conn_sock) {
  // 128 bytes buffer --> change this to a larger value if you want to send more data
  char buffer[128];
  int done = 0;

  // Keep receiving data until 128 bytes are received
  while (done < sizeof(buffer)) {
    // buffer + done = dataptr --> pointer to the address of the data to be received
    // sizeof(buffer) - done = len --> length of the data left to be received
    int sent = recv(conn_sock, buffer + done, sizeof(buffer) - done, 0);
    if (sent > 0) {
      printf("%s\n", buffer);
    }
    // If sent is 0, the connection is closed
    if (sent <= 0)
      return -1;
    
    // sent = number of bytes received, done = number of bytes received so far
    done += sent;
    break;
  }

  return 0;
} */

static int handle_single_command(int conn_sock) {
    // 512 bytes buffer for message reception
    char buffer[512];
    int done = 0;

    // Loop to receive data in chunks
    while (done < sizeof(buffer)) {
        // Receive data into the buffer
        int sent = recv(conn_sock, buffer + done, sizeof(buffer) - done - 1, 0);
        
        // Check if data was received
        if (sent > 0) {
            done += sent;
            buffer[done] = '\0'; // Null-terminate the buffer

            // Look for newline delimiter to identify end of message
            char *newline_pos = strchr(buffer, '\n');
            if (newline_pos) {
                // Print the message up to the newline character
                printf("%s\n", buffer);

                // Reset buffer and done counter for the next message
                memset(buffer, 0, sizeof(buffer));
                done = 0;
            }
        } 
        else if (sent == 0) {
            // Connection closed by client
            printf("Client disconnected.\n");
            return -1;
        } 
        else {
            // Error in receiving data
            printf("Error receiving data: %d\n", errno);
            return -1;
        }
    }

    return 0;
}


static void handle_connection(int conn_sock) {
  while (!handle_single_command(conn_sock))
    ;

  closesocket(conn_sock);
}

static void run_server() {
  int server_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
  struct sockaddr_in listen_addr = {
      .sin_len = sizeof(struct sockaddr_in),
      .sin_family = AF_INET,
      .sin_port = htons(12345),
      .sin_addr = {0},
  };

  if (server_sock < 0) {
    printf("Unable to create socket: error %d", errno);
    return;
  }

  if (bind(server_sock, (struct sockaddr *)&listen_addr, sizeof(listen_addr)) < 0) {
    printf("Unable to bind socket: error %d\n", errno);
    return;
  }

  if (listen(server_sock, 1) < 0) {
    printf("Unable to listen on socket: error %d\n", errno);
    return;
  }

  printf("Starting server at %s on port %u\n", ip4addr_ntoa(netif_ip4_addr(netif_list)), ntohs(listen_addr.sin_port));

  while (true) {
    struct sockaddr_storage remote_addr;
    socklen_t len = sizeof(remote_addr);
    int conn_sock = accept(server_sock, (struct sockaddr *)&remote_addr, &len);
    if (conn_sock < 0)
    {
      printf("Unable to accept incoming connection: error %d\n", errno);
      return;
    }
    printf("Connection from %s\n", ip4addr_ntoa((ip4_addr_t *)&((struct sockaddr_in *)&remote_addr)->sin_addr));
    handle_connection(conn_sock);
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
}