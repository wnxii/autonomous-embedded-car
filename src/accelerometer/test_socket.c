// Libraries needed for magnetometer and gyroscope components
#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <math.h>

// Libraries needed for WiFi component
#include "pico/cyw43_arch.h"
#include <lwip/sockets.h>
#include "lwip/apps/lwiperf.h"
#include "lwip/ip4_addr.h"
#include "lwip/netif.h"
#include "FreeRTOS.h"
#include "task.h"

// I2C configuration
#define I2C_PORT i2c1
#define SDA_PIN 26
#define SCL_PIN 27

// LSM303 I2C addresses
#define LSM303_ACC_ADDR 0x19 // Accelerometer I2C address
#define LSM303_MAG_ADDR 0x1E // Magnetometer I2C address

// Thresholds for pitch and roll angles
#define PITCH_THRESHOLD 10.0f // Threshold for pitch to detect forward/backward
#define ROLL_THRESHOLD 10.0f  // Threshold for roll to detect left/right

// WiFi configuration
#define TEST_TASK_PRIORITY (tskIDLE_PRIORITY + 2UL)
#define WIFI_SSID "yongjun"
#define WIFI_PASSWORD "pewpew1234"
#define SERVER_IP "172.20.10.2"
#define SERVER_PORT 12345

void i2c_init_setup() {
    i2c_init(I2C_PORT, 400 * 1000); // Initialize I2C at 400kHz
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
}

void send_message(int socket, const char *msg) {
    int msg_len = strmsg_len(msg);
    int done = 0;
    while (done < msg_len) {
        int msg_sent = send(socket, msg + done, msg_len - done, 0);
        if (msg_sent <= 0) {
            printf("Connection closed\n");
            return;
        }
        done += msg_sent;
    }
}

int run_client() {
    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    struct sockaddr_in server_addr;
    server_addr.sin_len = sizeof(struct sockaddr_in);
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SERVER_PORT);
    inet_aton(SERVER_IP, &server_addr.sin_addr);

    if (sock < 0) {
        printf("Unable to create socket: error %d\n", errno);
        return;
    }

    printf("Connecting to server at %s:%d\n", SERVER_IP, SERVER_PORT);
    if (connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        printf("Connection to server failed. Error: %d\n", errno);
        closesocket(sock);
        cyw43_arch_deinit();
        return;
    }

    printf("Connected to server!\n");

    return sock;
}

void client_task(__unused void *params) {
    int sock = 0;
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

    sock = run_client();
    i2c_init_setup();
    lsm303_init();

    int16_t acc_x_raw, acc_y_raw, acc_z_raw;
    float acc_x_g, acc_y_g, acc_z_g;
    float acc_x_filtered = 0, acc_y_filtered = 0, acc_z_filtered = 0;
    float alpha = 0.5; // Smoothing factor
    float pitch, roll;
    int speed, steering;

    while (true)
    {
        printf("\033[2J\033[H");
        printf("------------------------------\n");

        // Read accelerometer data
        read_accel_data(&acc_x_raw, &acc_y_raw, &acc_z_raw);

        printf("Accel X: %d, Accel Y: %d, Accel Z: %d\n", acc_x_raw, acc_y_raw, acc_z_raw);

        // Convert raw data to 'g' units
        acc_x_g = acc_x_raw * 0.001;
        acc_y_g = acc_y_raw * 0.001;
        acc_z_g = acc_z_raw * 0.001;

        printf("Accel X: %.2f, Accel Y: %.2f, Accel Z: %.2f\n", acc_x_g, acc_y_g, acc_z_g);

        // Apply filtering
        filter_data(&acc_x_filtered, acc_x_g, alpha);
        filter_data(&acc_y_filtered, acc_y_g, alpha);
        filter_data(&acc_z_filtered, acc_z_g, alpha);

        printf("Filtered Accel X: %.2f, Filtered Accel Y: %.2f, Filtered Accel Z: %.2f\n", acc_x_filtered, acc_y_filtered, acc_z_filtered);

        // Compute tilt angles
        compute_tilt_angles(acc_x_filtered, acc_y_filtered, acc_z_filtered, &pitch, &roll);

        printf("Pitch: %.2f, Roll: %.2f\n", pitch, roll);

        // Map pitch to forward/backward speed
        speed = map_angle_to_control(pitch, PITCH_THRESHOLD);
        // Map roll to left/right steering
        steering = map_angle_to_control(roll, ROLL_THRESHOLD);

        // Display control values
        printf("Speed Control (based on pitch): %d\n", speed);
        printf("Steering Control (based on roll): %d\n", steering);

        // Send direction and intensity
        send_direction_and_intensity(sock, speed, steering);

        printf("------------------------------\n");

        // Transmit control signals via WiFi (handled by WiFi buddy)
        // ...

        sleep_ms(50); // Adjust delay as needed
    }

    cyw43_arch_deinit();
}

void lsm303_init() {
    uint8_t buf[2];

    // Initialize Accelerometer
    buf[0] = 0x20; // CTRL_REG1_A
    buf[1] = 0x57; // 100Hz data rate, all axes enabled
    i2c_write_blocking(I2C_PORT, LSM303_ACC_ADDR, buf, 2, false);

    buf[0] = 0x23; // CTRL_REG4_A
    buf[1] = 0x00; // ±2g full scale
    i2c_write_blocking(I2C_PORT, LSM303_ACC_ADDR, buf, 2, false);

    // Initialize Magnetometer
    buf[0] = 0x00; // CRA_REG_M
    buf[1] = 0x14; // 30Hz data rate
    i2c_write_blocking(I2C_PORT, LSM303_MAG_ADDR, buf, 2, false);

    buf[0] = 0x02; // MR_REG_M
    buf[1] = 0x00; // Continuous conversion mode
    i2c_write_blocking(I2C_PORT, LSM303_MAG_ADDR, buf, 2, false);
}

void read_accel_data(int16_t *acc_x, int16_t *acc_y, int16_t *acc_z) {
    uint8_t reg = 0x28 | 0x80; // Auto-increment address
    uint8_t data[6];

    i2c_write_blocking(I2C_PORT, LSM303_ACC_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, LSM303_ACC_ADDR, data, 6, false);

    *acc_x = (int16_t)(data[1] << 8 | data[0]);
    *acc_y = (int16_t)(data[3] << 8 | data[2]);
    *acc_z = (int16_t)(data[5] << 8 | data[4]);
}

void filter_data(float *filtered_value, float new_value, float alpha) {
    *filtered_value = alpha * new_value + (1 - alpha) * (*filtered_value);
}

void compute_tilt_angles(float acc_x, float acc_y, float acc_z, float *pitch, float *roll) {
    *roll = atan2(acc_y, sqrt(acc_x * acc_x + acc_z * acc_z)) * 180 / M_PI; // Roll from Y and XZ components
    *pitch = atan2(-acc_x, acc_z) * 180 / M_PI; // Pitch from X and Z components
}

int map_angle_to_control(float angle, float threshold) {
    if (angle > threshold) {
        return (int)(100 * (angle - threshold) / (90.0f - threshold));
    } else if (angle < -threshold) {
        return (int)(100 * (angle + threshold) / (90.0f - threshold));
    } else {
        return 0; // Neutral position
    }
}

void send_direction_and_intensity(int sock, int speed, int steering) {
    // Determine forward/backward direction and speed intensity
    char msg[150];
    if (speed > 0) {
        snprintf(msg, 150, "[Forward] at Speed %d\n", speed);
        printf("%s", msg);
        send_message(sock, msg);
    } else if (speed < 0) {
        snprintf(msg, 150, "[Backward] at Speed %d\n", -speed); // Print positive value for intensity
        printf("%s", msg);
        send_message(sock, msg);
    } else {
        printf("Speed: Neutral\n");
    }

    // Determine left/right direction and steering intensity
    if (steering > 0) {
        snprintf(msg, 150, "[Right] at Intensity %d\n", steering);
        printf("%s", msg);
        send_message(sock, msg);
    } else if (steering < 0) {
        snprintf(msg, 150, "[Left] at Intensity %d\n", -steering); // Print positive value for intensity
        printf("%s", msg);
        send_message(sock, msg);
    } else {
        printf("Steering: Neutral\n");
    }
}

int main() {
    stdio_init_all();
    TaskHandle_t task;
    xTaskCreate(client_task, "MainThread", configMINIMAL_STACK_SIZE, NULL, 2, &task);
    vTaskStartScheduler();
    
    return 0;
}