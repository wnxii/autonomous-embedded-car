/**
 * @file motor.c
 * @brief Motor control implementation with PID speed regulation
 *
 * This module implements motor control functionality for a two-wheeled robot,
 * including:
 * - PWM-based motor speed control
 * - PID speed regulation
 * - Directional control (forward, backward, steering)
 * - Remote control mapping
 * - Pivot turning capabilities
 *
 * The implementation uses FreeRTOS for task management and includes
 * safety features like duty cycle limits and motor shutdown.
 *
 */

#include "FreeRTOS.h"
#include "task.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <stdio.h>
#include <stdbool.h>
#include "semphr.h"
#include <math.h>
#include "../wheel_encoder/wheel_encoder.h" // Adjust this path if necessary
#include "motor.h"
#include "../wifi/client_server_socket/client_server_socket.h"

// Global motor configurations
static MotorConfig left_motor = {PWM_PIN1, DIR_PIN1, DIR_PIN2, 0.0f, 0.0f, 0.0f, 0.0f};
static MotorConfig right_motor = {PWM_PIN2, DIR_PIN3, DIR_PIN4, 0.0f, 0.0f, 0.0f, 0.0f};

// Mutex for PID calculations
static SemaphoreHandle_t pid_mutex;

static PIDState left_speed_pid = {0.0f, 0.0f};
static PIDState right_speed_pid = {0.0f, 0.0f};

// Global variable to store the current movement direction
static MovementDirection current_movement = STOP;

// Global variable to store if car is currently turning.
bool pivot_turning_active = false;

// Global variable to store latest duty cycle set to control speed
static float current_duty_cycle = 1.0;

/**
 * @brief Configure and set PWM for a motor
 *
 * Sets up PWM configuration for a motor including frequency and duty cycle.
 * The function handles all necessary GPIO setup and PWM slice configuration.
 *
 * @param gpio The GPIO pin number for PWM output
 * @param duty_cycle The desired duty cycle (0.0 to 1.0)
 * @param freq The desired PWM frequency in Hz
 */
void set_motor_pwm(uint gpio, float duty_cycle, float freq) {
    current_duty_cycle = duty_cycle;
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(gpio);

    float clock_freq = 125000000.0f;  // System clock frequency
    uint32_t divider = clock_freq / (freq * 65536);
    pwm_set_clkdiv(slice_num, divider);
    pwm_set_wrap(slice_num, 65535);

    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(gpio), duty_cycle * 65535);

    pwm_set_enabled(slice_num, true);
}

/**
 * @brief Calculate PID control output
 *
 * Implements a PID controller for motor speed regulation.
 * Includes anti-windup protection and output limiting.
 *
 * @param set_point The desired speed setpoint
 * @param current_value The current measured speed
 * @param pid_state Pointer to PID state structure
 * @return float The calculated control output (duty cycle)
 */
float calculate_pid(float set_point, float current_value, PIDState* pid_state) {
    float error = set_point - current_value;

    // Proportional term
    float p_term = (KP * error);

    // Integral term
    float tmp_integral = pid_state->integral + error;
    float i_term = KI * tmp_integral;

    // Derivative term
    float d_term = KD * (error - pid_state->prev_error);

    // Update previous error
    pid_state->prev_error = error;

    // Calculate output
    float output = p_term + i_term + d_term;

    float corrected_speed = current_duty_cycle += output;

    if (corrected_speed< MAX_DUTY_CYCLE && corrected_speed > MIN_DUTY_CYCLE) {
        pid_state->integral = tmp_integral;  // Update the integrator if within bounds to prevent integrator windup
    } else if (corrected_speed > MAX_DUTY_CYCLE) {
       corrected_speed = MAX_DUTY_CYCLE;
    } else if (corrected_speed < MIN_DUTY_CYCLE) {
        corrected_speed = MIN_DUTY_CYCLE;
    }
    return corrected_speed;
}

/**
 * @brief Control motor direction and speed
 *
 * Sets the direction pins and target speed for a specified motor.
 *
 * @param motor Pointer to motor configuration structure
 * @param forward True for forward direction, false for backward
 * @param target_speed The desired motor speed
 */
void control_motor_direction(MotorConfig* motor, bool forward, float target_speed) {
    gpio_put(motor->dir_pin1, !forward);
    gpio_put(motor->dir_pin2, forward);

    motor->target_speed = target_speed;

}

/**
 * @brief Calculate angle turned during pivot maneuver
 *
 * Calculates the approximate angle turned by the robot during a pivot turn
 * based on wheel encoder readings and robot geometry.
 *
 * @return float The calculated angle in degrees
 */
float get_angle_turned_pivot() {
    // Implement an approximate calculation for pivot turn angle
    float wheel_base = 10.0f; // Distance between wheels in cm, adjust based on robot dimensions
    float left_dist = fabs(get_left_distance());  // Take absolute values to sum distances
    float right_dist = fabs(get_right_distance());

    // Calculate the angle turned in degrees using the sum of wheel distances
    float angle = (left_dist + right_dist) / wheel_base * (180.0f / M_PI);
    return angle;
}

/**
 * @brief Execute a pivot turn maneuver
 *
 * Controls the motors to perform a pivot turn for a specified duration.
 * One wheel moves forward while the other moves backward.
 *
 * @param duration_ms Duration of the turn in milliseconds
 */
void control_motor_pivot_turn(float duration_ms) {
    // Ensure direction is valid
    // Set motor directions based on the desired pivot direction
    if (current_movement == PIVOT_RIGHT) {
        // Right turn: left motor forward, right motor reverse
        gpio_put(left_motor.dir_pin1, 0);
        gpio_put(left_motor.dir_pin2, 1);
        gpio_put(right_motor.dir_pin1, 1);
        gpio_put(right_motor.dir_pin2, 0);
    } else if (current_movement == PIVOT_LEFT) {
        // Left turn: left motor reverse, right motor forward
        gpio_put(left_motor.dir_pin1, 1);
        gpio_put(left_motor.dir_pin2, 0);
        gpio_put(right_motor.dir_pin1, 0);
        gpio_put(right_motor.dir_pin2, 1);
    }

    // Set PWM for motors
    set_motor_pwm(left_motor.pwm_pin, MAX_DUTY_CYCLE - 0.2, 256.0f);
    set_motor_pwm(right_motor.pwm_pin, MAX_DUTY_CYCLE - 0.2, 256.0f);

    // Wait for the specified duration
    busy_wait_ms((int)duration_ms);

    // Stop motors after the duration
    gpio_put(left_motor.dir_pin1, 0);
    gpio_put(left_motor.dir_pin2, 0);
    gpio_put(right_motor.dir_pin1, 0);
    gpio_put(right_motor.dir_pin2, 0);

} 


/**
 * @brief Map remote control inputs to motor control parameters
 *
 * Converts remote control direction and steering inputs into appropriate
 * motor speeds and directions for both wheels.
 *
 * @param remote_output_direction Forward/backward control value (-20 to 20)
 * @param remote_output_steering Left/right control value (-20 to 20)
 * @return MotorControl Structure containing calculated wheel speeds and direction
 */
MotorControl map_remote_output_to_direction(int remote_output_direction, int remote_output_steering) {
    MotorControl control;

    // Initialize speeds to zero
    control.left_wheel_speed = 0.0;
    control.right_wheel_speed = 0.0;

     // Handle backward motion
    if (remote_output_direction < 0) {
        if (remote_output_steering < 0) {
            // Backward and steering left
            control.direction = STEER_BACKWARD_LEFT;
            control.left_wheel_speed = (MAX_SPEED / 20.0f) * -remote_output_direction * (20 + remote_output_steering) / 20.0f; // Slow left wheel
            control.right_wheel_speed = (MAX_SPEED / 20.0f) * -remote_output_direction;                                     // Full speed for right wheel
        } else if (remote_output_steering > 0) {
            // Backward and steering right
            control.direction = STEER_BACKWARD_RIGHT;
            control.left_wheel_speed = (MAX_SPEED / 20.0f) * -remote_output_direction;                                      // Full speed for left wheel
            control.right_wheel_speed = (MAX_SPEED / 20.0f) * -remote_output_direction * (20 - remote_output_steering) / 20.0f; // Slow right wheel
        } else {
            // Straight backward
            control.direction = BACKWARD;
            control.left_wheel_speed = (MAX_SPEED / 20.0f) * -remote_output_direction;
            control.right_wheel_speed = (MAX_SPEED / 20.0f) * -remote_output_direction;
        }
        // Handle forward motion
    } else if (remote_output_direction > 0) {
        if (remote_output_steering < 0) {
             // Forward and steering left
            control.direction = STEER_FORWARD_LEFT;
            control.left_wheel_speed = (MAX_SPEED / 20.0f) * remote_output_direction * (20 + remote_output_steering) / 20.0f; // Slow left wheel
            control.right_wheel_speed = (MAX_SPEED / 20.0f) * remote_output_direction;                                       // Full speed for right wheel
        } else if (remote_output_steering > 0) {
            // Forward and steering right
            control.direction = STEER_FORWARD_RIGHT;
            control.left_wheel_speed = (MAX_SPEED / 20.0f) * remote_output_direction;                                        // Full speed for left wheel
            control.right_wheel_speed = (MAX_SPEED / 20.0f) * remote_output_direction * (20 - remote_output_steering) / 20.0f; // Slow right wheel
        } else {
            // Straight forward
            control.direction = FORWARD;
            control.left_wheel_speed = (MAX_SPEED / 20.0f) * remote_output_direction;
            control.right_wheel_speed = (MAX_SPEED / 20.0f) * remote_output_direction;
        }
        // Handle stopping
    } else {
        // If remote_output is 0, stop the motors
        control.direction = STOP;
        control.left_wheel_speed = 0.0;
        control.right_wheel_speed = 0.0;
    }

    return control;
} 

/**
 * @brief Stop all motor movement
 *
 * Immediately stops both motors by disabling all control pins.
 * Used for emergency stops or when movement is complete.
 */
void stop_motor() {
    gpio_put(left_motor.pwm_pin, 0);
    gpio_put(right_motor.pwm_pin, 0);
    gpio_put(left_motor.dir_pin1, 0);
    gpio_put(left_motor.dir_pin2, 0);
    gpio_put(right_motor.dir_pin1, 0);
    gpio_put(right_motor.dir_pin2, 0);
}

/**
 * @brief Unified function for controlling car movement
 *
 * Controls the car's movement including forward, backward, and turning motions.
 * Implements speed ramping for smooth acceleration and deceleration.
 *
 * @param direction The desired movement direction
 * @param left_target_speed Target speed for left motor
 * @param right_target_speed Target speed for right motor
 * @param duration Duration of movement in milliseconds
 */
void move_car(MovementDirection direction, float left_target_speed, float right_target_speed, float duration) {
    current_movement = direction; // Update current direction
    switch(direction) {
        case FORWARD:
            control_motor_direction(&left_motor, true, left_target_speed);
            control_motor_direction(&right_motor, true, right_target_speed);
            break;

        case BACKWARD:
            control_motor_direction(&left_motor, false, left_target_speed);
            control_motor_direction(&right_motor, false, right_target_speed);
            break;

        case PIVOT_LEFT:
            control_motor_pivot_turn(duration);
            break;

        case PIVOT_RIGHT:
            control_motor_pivot_turn(duration);
            break;

        case STEER_FORWARD_LEFT:
            control_motor_direction(&left_motor, true, left_target_speed);
            control_motor_direction(&right_motor, true, right_target_speed);
            break;

        case STEER_FORWARD_RIGHT:
            control_motor_direction(&left_motor, true, left_target_speed);
            control_motor_direction(&right_motor, true, right_target_speed);
            break;

        case STEER_BACKWARD_LEFT:
            control_motor_direction(&left_motor, false, left_target_speed);
            control_motor_direction(&right_motor, false, right_target_speed);
            break;

        case STEER_BACKWARD_RIGHT:
            control_motor_direction(&left_motor, false, left_target_speed);
            control_motor_direction(&right_motor, false, right_target_speed);
            break;

        case STOP:
            stop_motor();
            break;
    }
}

/**
 * @brief FreeRTOS task for PID control updates
 *
 * Periodically updates PID calculations to maintain stable motor speeds.
 * Runs as a separate task to ensure consistent timing of PID updates.
 *
 * @param pvParameters Task parameters (unused)
 */
void pid_update_task(void *pvParameters) {
    xSemaphoreTake(wifiConnectedSemaphore, portMAX_DELAY);
    while (1) {
        if (current_movement != STOP || current_movement != PIVOT_LEFT || current_movement != PIVOT_RIGHT) {
            // Only adjust if the target speed is greater than zero
            if (left_motor.target_speed > 0 || right_motor.target_speed > 0) {
                // Update the PID output for left motor
                float left_current_speed = get_left_speed();
                left_motor.current_speed = left_current_speed;
                if (xSemaphoreTake(pid_mutex, portMAX_DELAY)) {
                    float left_pwm_output = calculate_pid(left_motor.target_speed, left_current_speed, &left_speed_pid);
                    set_motor_pwm(left_motor.pwm_pin, left_pwm_output, 256.0f);
                    xSemaphoreGive(pid_mutex);
                }

                // Update the PID output for right motor
                float right_current_speed = get_right_speed();
                right_motor.current_speed = right_current_speed;
                if (xSemaphoreTake(pid_mutex, portMAX_DELAY)) {
                    float right_pwm_output = calculate_pid(right_motor.target_speed, right_current_speed, &right_speed_pid);
                    set_motor_pwm(right_motor.pwm_pin, right_pwm_output, 256.0f);
                    xSemaphoreGive(pid_mutex);
                }
            } 
        } 
        taskYIELD();  
        vTaskDelay(pdMS_TO_TICKS(10)); // Adjust delay as needed for responsiveness
    }
}

/**
 * @brief Initialize motor control system
 *
 * Sets up all necessary components for motor control:
 * - Configures GPIO pins
 * - Initializes PWM
 * - Creates PID control task
 * - Sets up synchronization primitives
 */
void init_motor() {
    // Initialize GPIO for motors
    gpio_init(DIR_PIN1);
    gpio_init(DIR_PIN2);
    gpio_set_dir(DIR_PIN1, GPIO_OUT);
    gpio_set_dir(DIR_PIN2, GPIO_OUT);

    gpio_init(DIR_PIN3);
    gpio_init(DIR_PIN4);
    gpio_set_dir(DIR_PIN3, GPIO_OUT);
    gpio_set_dir(DIR_PIN4, GPIO_OUT);
    
    // Create mutex for PID calculations
    pid_mutex = xSemaphoreCreateMutex();

    // Create PID update task for maintaining speed alignment between wheels
    xTaskCreate(pid_update_task, "PID Update", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+4, NULL);
}
