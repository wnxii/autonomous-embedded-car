// Libraries needed for magnetometer and gyroscope components
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// Libraries needed for WiFi component
#include "pico/cyw43_arch.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"

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

// WiFi and UDP configuration
#define WIFI_SSID "liangfannn"
#define WIFI_PASSWORD "saypleasethankyou"
#define SERVER_IP "172.20.10.3"
#define UDP_PORT 12345
#define MSG_MAX_LEN 10

void i2c_init_setup()
{
    i2c_init(I2C_PORT, 400 * 1000); // Initialize I2C at 400kHz
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
}

void send_udp_message(struct udp_pcb* pcb, const ip_addr_t* addr, const char* msg)
{
    struct pbuf* p = pbuf_alloc(PBUF_TRANSPORT, strlen(msg) + 1, PBUF_RAM);
    if (!p) {
        printf("Failed to allocate pbuf\n");
        return;
    }

    memcpy(p->payload, msg, strlen(msg) + 1);
    err_t err = udp_sendto(pcb, p, addr, UDP_PORT);
    pbuf_free(p);

    if (err != ERR_OK) {
        printf("Failed to send UDP message: %d\n", err);
    }
}

void lsm303_init()
{
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

void read_accel_data(int16_t *acc_x, int16_t *acc_y, int16_t *acc_z)
{
    uint8_t reg = 0x28 | 0x80; // Auto-increment address
    uint8_t data[6];

    i2c_write_blocking(I2C_PORT, LSM303_ACC_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, LSM303_ACC_ADDR, data, 6, false);

    *acc_x = (int16_t)(data[1] << 8 | data[0]);
    *acc_y = (int16_t)(data[3] << 8 | data[2]);
    *acc_z = (int16_t)(data[5] << 8 | data[4]);
}

void filter_data(float *filtered_value, float new_value, float alpha)
{
    *filtered_value = alpha * new_value + (1 - alpha) * (*filtered_value);
}

void compute_tilt_angles(float acc_x, float acc_y, float acc_z, float *pitch, float *roll)
{
    *roll = atan2(acc_y, sqrt(acc_x * acc_x + acc_z * acc_z)) * 180 / M_PI;
    *pitch = atan2(-acc_x, acc_z) * 180 / M_PI;
}

int map_angle_to_control(float angle, float threshold)
{
    if (angle > threshold)
    {
        return (int)(100 * (angle - threshold) / (90.0f - threshold));
    }
    else if (angle < -threshold)
    {
        return (int)(100 * (angle + threshold) / (90.0f - threshold));
    }
    else
    {
        return 0; // Neutral position
    }
}

// void send_direction_and_intensity(struct udp_pcb* pcb, const ip_addr_t* addr, int speed, int steering)
// {
//     char msg[MSG_MAX_LEN];
//     const char *speed_dir = (speed > 0) ? "Forward" : (speed < 0) ? "Backward" : "Neutral";
//     const char *steer_dir = (steering > 0) ? "Right" : (steering < 0) ? "Left" : "Center";

//     snprintf(msg, MSG_MAX_LEN, "[%s-%s] Speed: %d, Steering: %d\n", 
//              speed_dir, steer_dir, 
//              (speed > 0) ? speed : -speed, 
//              (steering > 0) ? steering : -steering);
    
//     send_udp_message(pcb, addr, msg);
// }

// Helper function to map one range to another
int map(int value, int in_min, int in_max, int out_min, int out_max) {
    long mapped = (long)(value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    if (mapped < out_min) return out_min;
    if (mapped > out_max) return out_max;
    return (int)mapped;
}

// Maps a value from one range to an ASCII character range with a specified neutral point
int map_to_ascii_range(int value, int in_min, int in_max, int ascii_min, int ascii_max, int ascii_neutral) {
    // If input is close to 0 (neutral), return the neutral ASCII value
    int neutral_threshold = (in_max - in_min) / 20; // Adjust this threshold as needed
    if (abs(value) <= neutral_threshold) {
        return ascii_neutral;
    }

    // Determine if we're mapping the positive or negative range
    if (value > 0) {
        return map(value, neutral_threshold, in_max, ascii_neutral + 1, ascii_max);
    } else {
        return map(value, in_min, -neutral_threshold, ascii_min, ascii_neutral - 1);
    }
}

// Example of how to use it in your send_direction_and_intensity function:
void send_direction_and_intensity(struct udp_pcb* pcb, const ip_addr_t* addr, int speed, int steering) {
    // Map speed (-212 to 212) to ASCII range (0 to 41) with 20 as neutral
    int ascii_speed = map_to_ascii_range(speed, -20, 20, 1, 41, 21);
    
    // Map steering (-80 to 80) to ASCII range (42 to 83) with 62 as neutral
    int ascii_steering = map_to_ascii_range(steering, -20, 20, 1, 41, 21);
    
    char msg[MSG_MAX_LEN];
    snprintf(msg, MSG_MAX_LEN, "%c%c", (char)ascii_speed, (char)ascii_steering);
    printf("Sending: %s\n Ascii Speed: %d, Ascii Steering: %d\n", msg, ascii_speed, ascii_steering);

    send_udp_message(pcb, addr, msg);
}

int main()
{
    stdio_init_all();

    if (cyw43_arch_init())
    {
        printf("Failed to initialize\n");
        return 1;
    }

    cyw43_arch_enable_sta_mode();

    printf("Connecting to WiFi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000))
    {
        printf("Failed to connect.\n");
        cyw43_arch_deinit();
        return 1;
    }
    printf("Connected.\n");

    // Initialize UDP
    struct udp_pcb* pcb = udp_new();
    if (!pcb) {
        printf("Failed to create UDP PCB\n");
        cyw43_arch_deinit();
        return 1;
    }

    // Set up server address
    ip_addr_t server_addr;
    ipaddr_aton(SERVER_IP, &server_addr);

    // Initialize sensors
    i2c_init_setup();
    lsm303_init();

    // Variables for sensor data processing
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
        printf("Filtered Accel X: %.2f, Filtered Accel Y: %.2f, Filtered Accel Z: %.2f\n", 
               acc_x_filtered, acc_y_filtered, acc_z_filtered);

        // Compute tilt angles
        compute_tilt_angles(acc_x_filtered, acc_y_filtered, acc_z_filtered, &pitch, &roll);
        printf("Pitch: %.2f, Roll: %.2f\n", pitch, roll);

        // Map angles to control values
        speed = map_angle_to_control(pitch, PITCH_THRESHOLD);
        steering = map_angle_to_control(roll, ROLL_THRESHOLD);

        printf("Speed Control (based on pitch): %d\n", speed);
        printf("Steering Control (based on roll): %d\n", steering);

        // Send control values via UDP
        send_direction_and_intensity(pcb, &server_addr, speed, steering);

        printf("------------------------------\n");

#if PICO_CYW43_ARCH_POLL
        // If using poll mode, need to periodically service the WiFi driver
        cyw43_arch_poll();
        sleep_ms(50);
#else
        sleep_ms(50);
#endif
    }

    udp_remove(pcb);
    cyw43_arch_deinit();
    return 0;
}
