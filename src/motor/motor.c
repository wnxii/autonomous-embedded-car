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

// Function to set PWM for a motor
void set_motor_pwm(uint gpio, float duty_cycle, float freq) {
    current_duty_cycle = duty_cycle;
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(gpio);

    float clock_freq = 125000000.0f;  // System clock frequency
    uint32_t divider = clock_freq / (freq * 65536);
    pwm_set_clkdiv(slice_num, divider);
    pwm_set_wrap(slice_num, 65535);
    
    printf("Setting PWM on GPIO %d with duty cycle %.2f\n", gpio, duty_cycle);

    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(gpio), duty_cycle * 65535);

    pwm_set_enabled(slice_num, true);
}

// PID calculation function
float calculate_pid(float set_point, float current_value, PIDState* pid_state) {
    float error = set_point - current_value;
    printf("Calculating PID: Set Point = %.2f, Current Value = %.2f, Error = %.2f\n", set_point, current_value, error);

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
    // output = scaleToPWM(output);

    float corrected_speed = current_duty_cycle += output;

    if (corrected_speed< MAX_DUTY_CYCLE && corrected_speed > MIN_DUTY_CYCLE) {
        pid_state->integral = tmp_integral;  // Update the integrator if within bounds to prevent integrator windup
    } else if (corrected_speed > MAX_DUTY_CYCLE) {
       corrected_speed = MAX_DUTY_CYCLE;
    } else if (corrected_speed < MIN_DUTY_CYCLE) {
        corrected_speed = MIN_DUTY_CYCLE;
    }

    // printf("PID output = %.2f (P: %.2f, I: %.2f, D: %.2f)\n", corrected_speed, p_term, i_term, d_term);
    return corrected_speed;
}

// Control motor forward, backward, steer left, steer right
void control_motor_direction(MotorConfig* motor, bool forward, float target_speed) {
    // printf("Controlling motor on PWM pin %d - %s\n", motor->pwm_pin, (motor == &left_motor) ? "Left motor" : "Right motor");
    // printf("Setting Target Speed (%.2f) on - %s\n", target_speed, (motor == &left_motor) ? "Left motor" : "Right motor");

    gpio_put(motor->dir_pin1, !forward);
    gpio_put(motor->dir_pin2, forward);

    motor->target_speed = target_speed;

    // Obtain current speed from encoder
    // motor->current_speed = (motor == &left_motor) ? get_left_speed() : get_right_speed();
    // printf("Current speed from encoder: %.2f\n", motor->current_speed);



    // set_motor_pwm(motor->pwm_pin, MAX_DUTY_CYCLE, 256.0f);
}

// Helper function to calculate angle when performing pivot turns
float get_angle_turned_pivot() {
    // Implement an approximate calculation for pivot turn angle
    float wheel_base = 10.0f; // Distance between wheels in cm, adjust based on robot dimensions
    float left_dist = fabs(get_left_distance());  // Take absolute values to sum distances
    float right_dist = fabs(get_right_distance());
    printf("Left Distance: %f", left_dist);
    printf("Right Distance: %f", right_dist);

    // Calculate the angle turned in degrees using the sum of wheel distances
    float angle = (left_dist + right_dist) / wheel_base * (180.0f / M_PI);
    printf("Pivot turn angle turned: %.2f degrees\n", angle);
    return angle;
}

// Task to control the motor during turning
void pivot_turn_control_task(void *pvParameters) {
    float target_angle = *(float *)pvParameters;
    MovementDirection direction = current_movement;
    
    // Turn until the angle is reached (+-5.0) 
    float current_angle = 0.0f;
    while (current_angle < target_angle - 5.0f || current_angle > target_angle + 5.0f) {  // Keep adjusting until close to target angle
        if ((current_angle - target_angle) > 5.0f) {
            break;
        } 
        current_angle = get_angle_turned_pivot(); // Update current angle

        if (direction == PIVOT_RIGHT) {
            // Apply reverse and forward direction to left and right motor respectively for right turn
            gpio_put(left_motor.dir_pin1, 0);
            gpio_put(left_motor.dir_pin2, 1);
            gpio_put(right_motor.dir_pin1, 1);
            gpio_put(right_motor.dir_pin2, 0);
            
            set_motor_pwm(left_motor.pwm_pin, MAX_DUTY_CYCLE-0.2, 1000.0f);
            set_motor_pwm(right_motor.pwm_pin, MAX_DUTY_CYCLE-0.2, 1000.0f);
                
        } else if (direction == PIVOT_LEFT) {
            // Apply forward and reverse direction to left and right motor respectively for left turn
            gpio_put(left_motor.dir_pin1, 1);
            gpio_put(left_motor.dir_pin2, 0);
            gpio_put(right_motor.dir_pin1, 0);
            gpio_put(right_motor.dir_pin2, 1);

            set_motor_pwm(left_motor.pwm_pin, MAX_DUTY_CYCLE-0.2, 1000.0f); 
            set_motor_pwm(right_motor.pwm_pin, MAX_DUTY_CYCLE-0.2, 1000.0f); 
        }

        // Update current angle based on encoder data
        current_angle = get_angle_turned_pivot();
        
        vTaskDelay(pdMS_TO_TICKS(2.5));  // Delay to let motors adjust
    }
    
    // Reset turning flag and clean up
    pivot_turning_active = false;
    vPortFree(pvParameters); // Free allocated memory for angle
    vTaskDelete(NULL); // Delete this task
}

// Control motor turning Left or Right
void control_motor_pivot_turn(float target_angle) {
    if (!pivot_turning_active) {
        // Set pivot_turning_active to true to indicate tasks are running
        pivot_turning_active = true;
        // Allocate memory for passing target angle and direction
        float *angle_param = pvPortMalloc(sizeof(float));
        if (angle_param == NULL) {
            printf("Failed to allocate memory for angle parameter\n");
            return;
        }
        *angle_param = target_angle;
        reset_left_encoder();
        reset_right_encoder();
         // Start the turn control task with the angle and direction as parameters
        if (xTaskCreate(pivot_turn_control_task, "Pivot Turn Control", configMINIMAL_STACK_SIZE, (void *)angle_param, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
            printf("Failed to create Pivot Turn Control task\n");
            vPortFree(angle_param);
            pivot_turning_active = false;
        }
    }   
}

// Function to map remote controller output to forward / backwards direction and target speed
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

// Disable all pins to stop motor
void stop_motor() {
    gpio_put(left_motor.pwm_pin, 0);
    gpio_put(right_motor.pwm_pin, 0);
    gpio_put(left_motor.dir_pin1, 0);
    gpio_put(left_motor.dir_pin2, 0);
    gpio_put(right_motor.dir_pin1, 0);
    gpio_put(right_motor.dir_pin2, 0);
}

// Unified movement function
void move_car(MovementDirection direction, float left_target_speed, float right_target_speed, float angle) {
    current_movement = direction; // Update current direction
    // printf("Moving car - Direction: %d, Left Target Speed: %.2f, Right Target Speed: %.2f\n", direction, left_target_speed, right_target_speed);
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
            control_motor_pivot_turn(angle);
            break;

        case PIVOT_RIGHT:
            control_motor_pivot_turn(angle);
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

// PID control task that stabilse car when moving forward
void pid_update_task(void *pvParameters) {
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
            
        vTaskDelay(pdMS_TO_TICKS(50)); // Adjust delay as needed for responsiveness
    }
}

// Initialization function
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
    xTaskCreate(pid_update_task, "PID Update", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
    // xTaskCreate(control_motor_on_line_task, "Control Motor on Line", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
}


