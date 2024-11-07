/**
 * Tilt-Based Control System for Raspberry Pi Pico
 * 
 * This program implements a tilt-based control system using a Raspberry Pi Pico
 * with an LSM303 accelerometer/magnetometer. It measures device orientation
 * and sends control commands over WiFi to a remote server.
 * 
 * The system:
 * - Reads acceleration data from LSM303 sensor
 * - Calculates pitch and roll angles from acceleration data
 * - Maps these angles to control values for speed and steering
 * - Sends control commands to a server over WiFi
 * 
 * Hardware Requirements:
 * - Raspberry Pi Pico W
 * - LSM303 accelerometer/magnetometer module
 * - I2C connection on pins 26 (SDA) and 27 (SCL)
 */

// Libraries needed for magnetometer and gyroscope components
#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

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
#define SDA_PIN 26  // Data pin for I2C
#define SCL_PIN 27  // Clock pin for I2C

// LSM303 I2C addresses for different components
#define LSM303_ACC_ADDR 0x19  // Accelerometer I2C address
#define LSM303_MAG_ADDR 0x1E  // Magnetometer I2C address

// Angle thresholds for control activation (in degrees)
#define PITCH_THRESHOLD 10.0f  // Minimum pitch angle for forward/backward control
#define ROLL_THRESHOLD 10.0f   // Minimum roll angle for left/right control

// WiFi network configuration
#define TEST_TASK_PRIORITY (tskIDLE_PRIORITY + 2UL)
#define WIFI_SSID "yongjun"          // WiFi network name
#define WIFI_PASSWORD "pewpew1234"      // WiFi password
#define SERVER_IP "172.20.10.3"          // Remote server IP address
#define SERVER_PORT 12345     // Remote server port number

/**
 * Initializes I2C communication for the LSM303 sensor.
 * Sets up I2C on specified pins with 400kHz clock speed and enables pull-up resistors.
 */
void i2c_init_setup() {
    i2c_init(I2C_PORT, 400 * 1000);  // Initialize I2C at 400kHz
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);  // Enable pull-up resistor on SDA
    gpio_pull_up(SCL_PIN);  // Enable pull-up resistor on SCL
}

/**
 * Sends a message to the server over TCP socket.
 * 
 * @param socket Socket file descriptor
 * @param msg Message to send
 */
void send_message(int socket, const char *msg) {
    int msg_len = strlen(msg);
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

/**
 * Establishes TCP connection with the remote server.
 * 
 * @return Socket file descriptor if successful, 0 if connection fails
 */
int run_client() {
    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    struct sockaddr_in server_addr;
    server_addr.sin_len = sizeof(struct sockaddr_in);
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SERVER_PORT);
    inet_aton(SERVER_IP, &server_addr.sin_addr);

    if (sock < 0) {
        printf("Unable to create socket: error %d\n", errno);
        return 0;
    }

    printf("Connecting to server at %s:%d\n", SERVER_IP, SERVER_PORT);
    if (connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        printf("Connection to server failed. Error: %d\n", errno);
        closesocket(sock);
        cyw43_arch_deinit();
        return 0;
    }

    printf("Connected to server!\n");
    return sock;
}

/**
 * Initializes the LSM303 sensor with appropriate settings.
 * Configures both accelerometer and magnetometer components.
 */
void lsm303_init() {
    uint8_t buf[2];

    // Initialize Accelerometer
    buf[0] = 0x20;  // CTRL_REG1_A register
    buf[1] = 0x57;  // 100Hz data rate, all axes enabled
    i2c_write_blocking(I2C_PORT, LSM303_ACC_ADDR, buf, 2, false);

    buf[0] = 0x23;  // CTRL_REG4_A register
    buf[1] = 0x00;  // ±2g full scale range
    i2c_write_blocking(I2C_PORT, LSM303_ACC_ADDR, buf, 2, false);

    // Initialize Magnetometer
    buf[0] = 0x00;  // CRA_REG_M register
    buf[1] = 0x14;  // 30Hz data rate
    i2c_write_blocking(I2C_PORT, LSM303_MAG_ADDR, buf, 2, false);

    buf[0] = 0x02;  // MR_REG_M register
    buf[1] = 0x00;  // Continuous conversion mode
    i2c_write_blocking(I2C_PORT, LSM303_MAG_ADDR, buf, 2, false);
}

/**
 * Reads raw acceleration data from the LSM303 sensor.
 * 
 * @param acc_x Pointer to store X-axis acceleration
 * @param acc_y Pointer to store Y-axis acceleration
 * @param acc_z Pointer to store Z-axis acceleration
 */
void read_accel_data(int16_t *acc_x, int16_t *acc_y, int16_t *acc_z) {
    uint8_t reg = 0x28 | 0x80;  // Starting register address with auto-increment
    uint8_t data[6];

    i2c_write_blocking(I2C_PORT, LSM303_ACC_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, LSM303_ACC_ADDR, data, 6, false);

    // Combine high and low bytes into 16-bit values
    *acc_x = (int16_t)(data[1] << 8 | data[0]);
    *acc_y = (int16_t)(data[3] << 8 | data[2]);
    *acc_z = (int16_t)(data[5] << 8 | data[4]);
}

/**
 * Applies a low-pass filter to smooth sensor data.
 * 
 * @param filtered_value Pointer to the current filtered value
 * @param new_value New sensor reading
 * @param alpha Filter coefficient (0-1), higher values mean less filtering
 */
void filter_data(float *filtered_value, float new_value, float alpha) {
    *filtered_value = alpha * new_value + (1 - alpha) * (*filtered_value);
}

/**
 * Calculates pitch and roll angles from acceleration data.
 * 
 * @param acc_x X-axis acceleration
 * @param acc_y Y-axis acceleration
 * @param acc_z Z-axis acceleration
 * @param pitch Pointer to store calculated pitch angle
 * @param roll Pointer to store calculated roll angle
 */
void compute_tilt_angles(float acc_x, float acc_y, float acc_z, float *pitch, float *roll) {
    // Calculate roll angle using Y and XZ components
    *roll = atan2(acc_y, sqrt(acc_x * acc_x + acc_z * acc_z)) * 180 / M_PI;
    
    // Calculate pitch angle using X and Z components
    *pitch = atan2(-acc_x, acc_z) * 180 / M_PI;
}

/**
 * Maps tilt angle to control value with deadzone.
 * 
 * @param angle Current tilt angle
 * @param threshold Minimum angle before control activates
 * @return Control value (-100 to 100)
 */
int map_angle_to_control(float angle, float threshold) {
    if (angle > threshold) {
        // Map positive angles to 0-100 range
        return (int)(100 * (angle - threshold) / (90.0f - threshold));
    } else if (angle < -threshold) {
        // Map negative angles to -100-0 range
        return (int)(100 * (angle + threshold) / (90.0f - threshold));
    } else {
        return 0;  // Within deadzone - no control input
    }
}

/**
 * Sends direction and intensity commands to the server.
 * 
 * @param sock Socket connection to server
 * @param speed Forward/backward speed (-100 to 100)
 * @param steering Left/right steering intensity (-100 to 100)
 */
void send_direction_and_intensity(int sock, int speed, int steering) {
    char msg[150];

    // Handle forward/backward movement
    if (speed > 0) {
        snprintf(msg, 150, "[Forward] at Speed %d\n", speed);
        printf("%s", msg);
        send_message(sock, msg);
    } else if (speed < 0) {
        snprintf(msg, 150, "[Backward] at Speed %d\n", -speed);
        printf("%s", msg);
        send_message(sock, msg);
    } else {
        printf("Speed: Neutral\n");
    }

    // Handle left/right steering
    if (steering > 0) {
        snprintf(msg, 150, "[Right] at Intensity %d\n", steering);
        printf("%s", msg);
        send_message(sock, msg);
    } else if (steering < 0) {
        snprintf(msg, 150, "[Left] at Intensity %d\n", -steering);
        printf("%s", msg);
        send_message(sock, msg);
    } else {
        printf("Steering: Neutral\n");
    }
}

/**
 * Main Control Loop Implementation for Tilt-Based Control System
 * 
 * This file contains the main control task and program entry point.
 * It handles WiFi connection, sensor initialization, continuous data reading,
 * and control signal transmission.
 */

/**
 * Main client task that handles sensor reading and control signal generation.
 * This function runs as a FreeRTOS task and contains the main control loop.
 * 
 * The function performs the following steps:
 * 1. Initializes WiFi connection
 * 2. Sets up I2C and sensor communication
 * 3. Continuously reads and processes sensor data
 * 4. Transmits control signals to the server
 * 
 * @param params Task parameters (unused in this implementation)
 */
void client_task(__unused void *params)
{
    // Socket file descriptor for network communication
    int sock = 0;

    // Initialize WiFi hardware
    if (cyw43_arch_init())
    {
        printf("failed to initialise\n");
        return;
    }

    // Configure WiFi in station (client) mode
    cyw43_arch_enable_sta_mode();
    printf("Connecting to WiFi...\n");

    // Attempt to connect to WiFi network with 30-second timeout
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, 
        CYW43_AUTH_WPA2_AES_PSK, 30000))
    {
        printf("failed to connect.\n");
    }
    else
    {
        printf("Connected.\n");
    }

    // Initialize network socket and hardware interfaces
    sock = run_client();
    i2c_init_setup();
    lsm303_init();

    // Variables for storing accelerometer readings
    int16_t acc_x_raw, acc_y_raw, acc_z_raw;    // Raw sensor values
    float acc_x_g, acc_y_g, acc_z_g;            // Converted to g-force units
    float acc_x_filtered = 0, acc_y_filtered = 0, acc_z_filtered = 0;  // Filtered values
    float alpha = 0.5;  // Low-pass filter coefficient (0.5 = equal weight to new and old values)

    // Variables for storing calculated angles and control values
    float pitch, roll;  // Device orientation angles
    int speed, steering;  // Calculated control values

    // Main control loop
    while (true)
    {
        // Clear screen and print separator for clean output
        printf("\033[2J\033[H");  // ANSI escape codes to clear screen and move cursor to top
        printf("------------------------------\n");

        // Step 1: Read raw accelerometer data
        read_accel_data(&acc_x_raw, &acc_y_raw, &acc_z_raw);
        printf("Accel X: %d, Accel Y: %d, Accel Z: %d\n", 
               acc_x_raw, acc_y_raw, acc_z_raw);

        // Step 2: Convert raw accelerometer data to g-force units
        // Scaling factor of 0.001 converts raw values to approximate g-force
        acc_x_g = acc_x_raw * 0.001;
        acc_y_g = acc_y_raw * 0.001;
        acc_z_g = acc_z_raw * 0.001;
        printf("Accel X: %.2f, Accel Y: %.2f, Accel Z: %.2f\n", 
               acc_x_g, acc_y_g, acc_z_g);

        // Step 3: Apply low-pass filtering to smooth sensor data
        filter_data(&acc_x_filtered, acc_x_g, alpha);
        filter_data(&acc_y_filtered, acc_y_g, alpha);
        filter_data(&acc_z_filtered, acc_z_g, alpha);
        printf("Filtered Accel X: %.2f, Filtered Accel Y: %.2f, Filtered Accel Z: %.2f\n",
               acc_x_filtered, acc_y_filtered, acc_z_filtered);

        // Step 4: Calculate device orientation angles
        compute_tilt_angles(acc_x_filtered, acc_y_filtered, acc_z_filtered, 
                          &pitch, &roll);
        printf("Pitch: %.2f, Roll: %.2f\n", pitch, roll);

        // Step 5: Map orientation angles to control values
        // Pitch controls forward/backward speed
        speed = map_angle_to_control(pitch, PITCH_THRESHOLD);
        // Roll controls left/right steering
        steering = map_angle_to_control(roll, ROLL_THRESHOLD);

        // Display calculated control values
        printf("Speed Control (based on pitch): %d\n", speed);
        printf("Steering Control (based on roll): %d\n", steering);

        // Step 6: Transmit control signals to server
        send_direction_and_intensity(sock, speed, steering);
        printf("------------------------------\n");

        // Delay to control sampling rate
        sleep_ms(50);  // 20Hz update rate
    }

    // Cleanup WiFi on task exit (normally never reached)
    cyw43_arch_deinit();
}

/**
 * Program entry point
 * 
 * Initializes system and creates the main control task using FreeRTOS.
 * 
 * @return 0 on successful completion (never actually returns in this implementation)
 */
int main() {
    // Initialize standard I/O
    stdio_init_all();

    // Create FreeRTOS task for main control loop
    TaskHandle_t task;
    xTaskCreate(
        client_task,                  // Task function
        "MainThread",                 // Task name
        configMINIMAL_STACK_SIZE,     // Stack size
        NULL,                         // Task parameters (none)
        2,                           // Task priority
        &task                        // Task handle
    );

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();

    // Should never reach here as scheduler takes over
    return 0;
}