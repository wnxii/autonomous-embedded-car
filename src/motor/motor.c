#include "FreeRTOS.h"
#include "task.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <stdio.h>
#include <stdbool.h>
#include "semphr.h"
#include "../main_wheel_encoder/wheel_encoder.h" // Adjust this path if necessary

// Define GPIO pins
#define PWM_PIN1 2     // GP2 for PWM (Left Motor)
#define DIR_PIN1 0     // GP0 for direction
#define DIR_PIN2 1     // GP1 for direction

#define PWM_PIN2 3     // GP3 for PWM (Right Motor)
#define DIR_PIN3 26    // GP26 for direction
#define DIR_PIN4 27    // GP27 for direction

// PID Constants
#define KP 2.0f
#define KI 0.5f
#define KD 0.1f

// Movement Enum for clear direction control
typedef enum {
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT,
    STOP
} MovementDirection;

// Motor control structure
typedef struct {
    uint32_t pwm_pin;
    uint32_t dir_pin1;
    uint32_t dir_pin2;
    float current_speed;
    float target_speed;
} MotorConfig;

// Global motor configurations
static MotorConfig left_motor = {PWM_PIN1, DIR_PIN1, DIR_PIN2, 0.0f, 0.0f};
static MotorConfig right_motor = {PWM_PIN2, DIR_PIN3, DIR_PIN4, 0.0f, 0.0f};

// Mutex for PID calculations
static SemaphoreHandle_t pid_mutex;

// PID control variables
typedef struct {
    float integral;
    float prev_error;
} PIDState;

static PIDState left_pid = {0.0f, 0.0f};
static PIDState right_pid = {0.0f, 0.0f};

// Function to set PWM for a motor
void set_motor_pwm(uint gpio, float duty_cycle) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(gpio);

    pwm_set_wrap(slice_num, 65535);
    pwm_set_clkdiv(slice_num, 125.0f);  // Set frequency at approximately 1kHz

    pwm_set_gpio_level(gpio, (uint16_t)(duty_cycle * 65535));
    pwm_set_enabled(slice_num, true);
}

// PID calculation function
float calculate_pid(float target_speed, float current_speed, PIDState* pid_state) {
    float error = target_speed - current_speed;

    // Proportional term
    float p_term = KP * error;

    // Integral term
    pid_state->integral += error;
    float i_term = KI * pid_state->integral;

    // Derivative term
    float d_term = KD * (error - pid_state->prev_error);

    // Update previous error
    pid_state->prev_error = error;

    // Calculate output
    float output = p_term + i_term + d_term;

    // Clamp output between 0 and 1
    if (output > 1.0f) output = 1.0f;
    if (output < 0.0f) output = 0.0f;

    return output;
}

// Control motor direction and speed
void control_motor(MotorConfig* motor, bool forward, float target_speed) {
    gpio_put(motor->dir_pin1, forward);
    gpio_put(motor->dir_pin2, !forward);

    // Obtain current speed from encoder
    motor->current_speed = get_speed();  // Assumes get_speed() returns a value in cm/s
    motor->target_speed = target_speed;

    // Protect PID calculations with mutex
    if (xSemaphoreTake(pid_mutex, portMAX_DELAY)) {
        PIDState* pid = (motor == &left_motor) ? &left_pid : &right_pid;
        float pwm_output = calculate_pid(target_speed, motor->current_speed, pid);
        xSemaphoreGive(pid_mutex);

        set_motor_pwm(motor->pwm_pin, pwm_output);
    }
}

// Unified movement function
void move_robot(MovementDirection direction, float speed) {
    switch(direction) {
        case FORWARD:
            control_motor(&left_motor, true, speed);
            control_motor(&right_motor, true, speed);
            break;

        case BACKWARD:
            control_motor(&left_motor, false, speed);
            control_motor(&right_motor, false, speed);
            break;

        case LEFT:
            control_motor(&left_motor, true, speed * 0.9f);   // Slower left motor
            control_motor(&right_motor, true, speed);         // Full speed right motor
            break;

        case RIGHT:
            control_motor(&left_motor, true, speed);          // Full speed left motor
            control_motor(&right_motor, true, speed * 0.9f);  // Slower right motor
            break;

        case STOP:
            control_motor(&left_motor, true, 0.0f);
            control_motor(&right_motor, true, 0.0f);
            break;
    }
}

// Initialization function
void init_robot_movement() {
    // Initialize GPIO for motors
    gpio_init(DIR_PIN1);
    gpio_init(DIR_PIN2);
    gpio_set_dir(DIR_PIN1, GPIO_OUT);
    gpio_set_dir(DIR_PIN2, GPIO_OUT);

    gpio_init(DIR_PIN3);
    gpio_init(DIR_PIN4);
    gpio_set_dir(DIR_PIN3, GPIO_OUT);
    gpio_set_dir(DIR_PIN4, GPIO_OUT);

    // Initialize wheel encoder
    init_wheel_encoder();

    // Create mutex for PID calculations
    pid_mutex = xSemaphoreCreateMutex();
}

// Task for robot movement
void robot_movement_task(void *pvParameters) {
    // Initialize movement system
    init_robot_movement();

    while(1) {
        // Example movement sequence
        move_robot(FORWARD, 20.0f);  // Move forward at 20 cm/s
        vTaskDelay(pdMS_TO_TICKS(2000));

        move_robot(LEFT, 15.0f);     // Turn left at 15 cm/s
        vTaskDelay(pdMS_TO_TICKS(1000));

        move_robot(RIGHT, 15.0f);    // Turn right at 15 cm/s
        vTaskDelay(pdMS_TO_TICKS(1000));

        move_robot(BACKWARD, 20.0f); // Move backward at 20 cm/s
        vTaskDelay(pdMS_TO_TICKS(2000));

        move_robot(STOP, 0.0f);      // Stop
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Main function
int main() {
    stdio_init_all();
    init_robot_movement();

    // Create robot movement task
    xTaskCreate(robot_movement_task, "Robot Movement", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

    // Start FreeRTOS scheduler
    vTaskStartScheduler();

    // This point will not be reached because the scheduler is running
    while (1) {
        tight_loop_contents();
    }

    return 0;
}
