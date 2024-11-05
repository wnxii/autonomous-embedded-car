#include "FreeRTOS.h"
#include "task.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <stdio.h>
#include <stdbool.h>
#include "semphr.h"
#include <math.h>
#include "../main_wheel_encoder/wheel_encoder.h" // Adjust this path if necessary

// Define GPIO pins
#define PWM_PIN1 2     // GP2 for PWM (Left Motor)
#define DIR_PIN1 0     // Direction pin 1 for Left Motor (L298N IN1)
#define DIR_PIN2 1     // Direction pin 2 for Left Motor (L298N IN2)

#define PWM_PIN2 3     // GP3 for PWM (Right Motor)
#define DIR_PIN3 26    // Direction pin 1 for Right Motor (L298N IN3)
#define DIR_PIN4 27    // Direction pin 2 for Right Motor (L298N IN4)

// PID Constants
#define KP 0.1f 
#define KI 0.05f 
#define KD 4.0f

#define M_PI 3.14159265358979323846

#define MAX_DUTY_CYCLE 65535.0f
#define MIN_DUTY_CYCLE 49151.0f


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

// static PIDState left_pid = {0.0f, 0.0f};
// static PIDState right_pid = {0.0f, 0.0f};
static PIDState turning_pid = {0.0f, 0.0f};  // Initialize PID state for turning
// Global variable to store the current movement direction
static MovementDirection current_movement = STOP;
static bool turning_active = false;

// Initialization function
void init_robot_movement() {
    printf("Initializing robot movement\n");
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
    init_wheel_encoders();

    // Create mutex for PID calculations
    pid_mutex = xSemaphoreCreateMutex();
}


// Function to set PWM for a motor
void set_motor_pwm(uint gpio, float duty_cycle, float freq) {
    
    
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(gpio);

    float clock_freq = 125000000.0f;  // System clock frequency
    uint32_t divider = clock_freq / (freq * 65536);
    pwm_set_clkdiv(slice_num, divider);
    pwm_set_wrap(slice_num, 65535);
    
    printf("Setting PWM on GPIO %d with duty cycle %.2f\n", gpio, duty_cycle/65535);

    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(gpio), duty_cycle);

    pwm_set_enabled(slice_num, true);
}

// Control motor direction and speed
void control_motor_forward_backward(MotorConfig* motor, bool forward, float target_speed) {
    //printf("Controlling motor on PWM pin %d - Direction: %s, Target speed: %.2f\n", motor->pwm_pin, forward ? "Forward" : "Backward", target_speed, (motor == &left_motor) ? "Left motor" : "Right motor");
    printf("Controlling motor on PWM pin %d - %s\n", motor->pwm_pin, (motor == &left_motor) ? "Left motor" : "Right motor");

    gpio_put(motor->dir_pin1, !forward);
    gpio_put(motor->dir_pin2, forward);

    // Obtain current speed from encoder
    motor->current_speed = (motor == &left_motor) ? get_left_speed() : get_right_speed();
    // motor->current_speed = get_left_speed();
    motor->target_speed = target_speed;
    printf("Current speed from encoder: %.2f\n", motor->current_speed);

    set_motor_pwm(motor->pwm_pin, 65535.0f, 1000.0f);
}

void stop_motor() {
    gpio_put(left_motor.pwm_pin, 0);
    gpio_put(right_motor.pwm_pin, 0);
    gpio_put(left_motor.dir_pin1, 0);
    gpio_put(left_motor.dir_pin2, 0);
    gpio_put(right_motor.dir_pin1, 0);
    gpio_put(right_motor.dir_pin2, 0);
}

uint32_t scaleToPWM(double pidOutput) { 
    double pwmRange = MAX_DUTY_CYCLE - MIN_DUTY_CYCLE; 
    double normalizedOutput = (pidOutput + 32768.0) / 65536.0; // Normalize to 0-1
    uint32_t pwmValue = MIN_DUTY_CYCLE + (uint32_t)(normalizedOutput * pwmRange);
    return pwmValue;
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
    output = scaleToPWM(output);

    if (output < MAX_DUTY_CYCLE && output > MIN_DUTY_CYCLE) {
        pid_state->integral = tmp_integral;  // Update the integrator if within bounds to prevent integrator windup
    } else if (output > MAX_DUTY_CYCLE) {
        output = MAX_DUTY_CYCLE;
    } else if (output < MIN_DUTY_CYCLE) {
        output = MIN_DUTY_CYCLE;
    }

    printf("PID output = %.2f (P: %.2f, I: %.2f, D: %.2f)\n", output, p_term, i_term, d_term);
    return output;
}

// Helper function to calculate angle turned
float get_angle_turned() {
    // Implement an approximate calculation for the angle turned,
    // depending on the distance difference between left and right wheels
    float wheel_base = 12.0f; // Distance between wheels in cm, adjust based on robot dimensions
    float left_dist = get_left_distance();
    float right_dist = get_right_distance();
     // Calculate the angle turned in degrees using wheel distance difference
    float angle = fabs((right_dist - left_dist) / wheel_base * (180.0f / M_PI));
    printf("Angle turned: %.2f degrees\n", angle);
    return angle;
}

float get_angle_turned_pivot() {
    // Implement an approximate calculation for pivot turn angle
    float wheel_base = 10.0f; // Distance between wheels in cm, adjust based on robot dimensions
    float left_dist = fabs(get_left_distance());  // Take absolute values to sum distances
    float right_dist = fabs(get_right_distance());

    // Calculate the angle turned in degrees using the sum of wheel distances
    float angle = (left_dist + right_dist) / wheel_base * (180.0f / M_PI);
    printf("Pivot turn angle turned: %.2f degrees\n", angle);
    return angle;
}

// Task to control the motor during turning
void turn_control_task(void *pvParameters) {
    float target_angle = *(float *)pvParameters;
    MovementDirection direction = current_movement;
    
    // Turn until the angle is reached
    float current_angle = 0.0f;
    while (current_angle < target_angle - 3.5f || current_angle > target_angle + 3.5f) {  // Keep adjusting until close to target angle
        current_angle = get_angle_turned_pivot(); // Update current angle
        float pid_output = 0.0;
        // Use PID controller to get the speed adjustment
        if (xSemaphoreTake(pid_mutex, portMAX_DELAY)) {
            pid_output = calculate_pid(target_angle, current_angle, &turning_pid);
            xSemaphoreGive(pid_mutex);
        }

        if (direction == RIGHT) {
            gpio_put(left_motor.dir_pin1, 0);
            gpio_put(left_motor.dir_pin2, 1);
            gpio_put(right_motor.dir_pin1, 1);
            gpio_put(right_motor.dir_pin2, 0);
            // Apply differential speed for right turn
            
             // Calculate right motor speed to be lower by a differential amount
            set_motor_pwm(left_motor.pwm_pin, pid_output, 1000.0f);  // Increase left wheel speed
            set_motor_pwm(right_motor.pwm_pin, pid_output , 1000.0f);
                
                  // Slow right wheel
        } else if (direction == LEFT) {
            gpio_put(left_motor.dir_pin1, 1);
            gpio_put(left_motor.dir_pin2, 0);
            gpio_put(right_motor.dir_pin1, 0);
            gpio_put(right_motor.dir_pin2, 1);
            // Apply differential speed for left turn

            set_motor_pwm(left_motor.pwm_pin, pid_output, 1000.0f);  // Slow left wheel
            set_motor_pwm(right_motor.pwm_pin, pid_output, 1000.0f);  // Increase right wheel speed
            xSemaphoreGive(pid_mutex);
        }

        // Update current angle based on encoder data
        current_angle = get_angle_turned_pivot();  // Function to compute angle turned from encoder data
        
        vTaskDelay(pdMS_TO_TICKS(2.5));  // Delay to let motors adjust
    }
    
    // Reset turning flag and clean up
    turning_active = false;
    vPortFree(pvParameters); // Free allocated memory for angle
    vTaskDelete(NULL); // Delete this task
}

void control_motor_turn(float target_angle) {
    if (!turning_active) {
        // Set turning_active to true to indicate tasks are running
        turning_active = true;
        // Allocate memory for passing target angle and direction
        float *angle_param = pvPortMalloc(sizeof(float));
        if (angle_param == NULL) {
            printf("Failed to allocate memory for angle parameter\n");
            return;
        }
        *angle_param = target_angle;
         // Start the turn control task with the angle and direction as parameters
        if (xTaskCreate(turn_control_task, "Turn Control", configMINIMAL_STACK_SIZE, (void *)angle_param, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
            printf("Failed to create Turn Control task\n");
            vPortFree(angle_param);
            turning_active = false;
        }
    }   
}

// Unified movement function
void move_robot(MovementDirection direction, float speed, float angle) {
    current_movement = direction;
    printf("Moving robot - Direction: %d, Speed: %.2f\n", direction, speed);
    switch(direction) {
        case FORWARD:
            control_motor_forward_backward(&left_motor, true, speed);
            control_motor_forward_backward(&right_motor, true, speed);
            break;

        case BACKWARD:
            control_motor_forward_backward(&left_motor, false, speed);
            control_motor_forward_backward(&right_motor, false, speed);
            break;

        case LEFT:
            control_motor_turn(angle);
            break;

        case RIGHT:
            control_motor_turn(angle);
            break;

        case STOP:
            stop_motor();
            break;
    }
}






// Helper function to get average distance traveled by both wheels
float get_average_distance() {
    float average_distance = (get_left_distance() + get_right_distance()) / 2.0f;
    // printf("Average Distance: %f", average_distance);
    return average_distance;
}

// PID control task that checks the current target speed for each motor
/* void pid_update_task(void *pvParameters) {
    while (1) {
        // Only adjust if the target speed is greater than zero
        if (current_movement == LEFT || current_movement == RIGHT) {
            // Update the PID output for left motor
            float left_current_speed = get_left_speed();
            float right_current_speed = get_right_speed();
            left_motor.current_speed = left_current_speed;
            right_motor.current_speed = right_current_speed;
            if (xSemaphoreTake(pid_mutex, portMAX_DELAY)) {
                float left_pwm_output = calculate_pid(left_motor.target_speed, left_current_speed, &left_pid);
                float right_pwm_output = calculate_pid(right_motor.target_speed, right_current_speed, &right_pid);
                 // Calculate the average PWM output if the target speeds are equal
                float average_pwm_output = left_motor.target_speed == right_motor.target_speed 
                                        ? (left_pwm_output + right_pwm_output) / 2.0f 
                                        : left_pwm_output; // Use individual output if targets differ
                if(!average_pwm_output) {
                    set_motor_pwm(left_motor.pwm_pin, left_pwm_output, 1000.0f);
                    set_motor_pwm(right_motor.pwm_pin, right_pwm_output, 1000.0f);
                } else {
                     // Set the same average PWM for both motors if targets match
                    set_motor_pwm(left_motor.pwm_pin, average_pwm_output, 1000.0f);
                    set_motor_pwm(right_motor.pwm_pin, average_pwm_output, 1000.0f);
                    xSemaphoreGive(pid_mutex);
                }
            }  

            // Update the PID output for left motor
            float left_current_speed = get_left_speed();
            left_motor.current_speed = left_current_speed;
            if (xSemaphoreTake(pid_mutex, portMAX_DELAY)) {
                float left_pwm_output = calculate_pid(left_motor.target_speed, left_current_speed, &left_pid);
                if (left_motor.current_speed > left_motor.target_speed) {
                    set_motor_pwm(left_motor.pwm_pin, left_pwm_output, 1000.0f);
                }
                
                xSemaphoreGive(pid_mutex);
            }

            // Update the PID output for right motor
            float right_current_speed = get_right_speed();
            right_motor.current_speed = right_current_speed;
            if (xSemaphoreTake(pid_mutex, portMAX_DELAY)) {
                float right_pwm_output = calculate_pid(right_motor.target_speed, right_current_speed, &right_pid);
                set_motor_pwm(right_motor.pwm_pin, right_pwm_output, 1000.0f);
                xSemaphoreGive(pid_mutex);
            }
        } 

        // Delay to maintain periodic checks
        vTaskDelay(pdMS_TO_TICKS(75)); // Adjust delay as needed for responsiveness
    }
} */

// Task for robot movement
void robot_movement_task(void *pvParameters) {
    // Initialize movement system
    init_robot_movement();
    float target_speed = 25.0f;
    //while(1) {
        // Move forward 90 cm
        /* printf("Moving forward 90 cm\n");
        reset_left_encoder();
        reset_right_encoder();
        double start_timestamp = time_us_64() / 1000000.0; // Converts microseconds to seconds
        move_robot(FORWARD, target_speed, 0.0f);  // Set speed (e.g., 20 cm/s)
        while (get_average_distance() < 100.0f) {
            vTaskDelay(pdMS_TO_TICKS(5)); // Delay to periodically check distance
        }
        double end_timestamp = time_us_64() / 1000000.0; // Converts microseconds to seconds
        double time_diff = end_timestamp - start_timestamp;
        float average_speed = 100 / time_diff;
        printf("Average speed: %f \n", average_speed);
        move_robot(STOP, 0.0f, 0.0f); // Stop after reaching target distance
        vTaskDelay(pdMS_TO_TICKS(500)); // Small pause */

        // Turn right 90 degrees
        printf("Turning right 90 degrees\n");
        reset_left_encoder();
        reset_right_encoder();
        move_robot(LEFT, target_speed, 90.0);  // Set turn speed
        while (turning_active) {
            vTaskDelay(pdMS_TO_TICKS(10)); // Small delay to prevent busy-waiting
        }
        move_robot(STOP, 0.0f, 0.0f); // Stop after completing the turn
        vTaskDelay(pdMS_TO_TICKS(500)); // Small pause

        // Move forward 90 cm
        /* printf("Moving forward 90 cm\n");
        reset_left_encoder();
        reset_right_encoder();
        move_robot(FORWARD, 20.0f);
        while (get_average_distance() < 90.0f) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        move_robot(STOP, 0.0f);
        vTaskDelay(pdMS_TO_TICKS(500)); */

        /* // Turn left 90 degrees
        printf("Turning left 90 degrees\n");
        reset_left_encoder();
        reset_right_encoder();
        move_robot(LEFT, 15.0f);
        while (get_angle_turned() < 90.0f) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        move_robot(STOP, 0.0f);
        vTaskDelay(pdMS_TO_TICKS(500));

        // Move forward 20 cm
        printf("Moving forward 20 cm\n");
        reset_left_encoder();
        reset_right_encoder();
        move_robot(FORWARD, 20.0f);
        while (get_average_distance() < 20.0f) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        move_robot(STOP, 0.0f);
        vTaskDelay(pdMS_TO_TICKS(500));

        // Move backward 20 cm
        printf("Moving backward 20 cm\n");
        reset_left_encoder();
        reset_right_encoder();
        move_robot(BACKWARD, 20.0f);
        while (get_average_distance() < 20.0f) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        move_robot(STOP, 0.0f);
        vTaskDelay(pdMS_TO_TICKS(500)); */

        /* printf("Final stop\n");
        move_robot(STOP, 0.0f); // Final stop at end of sequence */
    //}
}

// Main function
int main() {
    stdio_init_all();
    printf("Starting robot control system\n");
    // init_robot_movement();

    // Create robot movement task
    xTaskCreate(robot_movement_task, "Robot Movement", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    
    // Create PID update task for maintaining speed alignment between wheels
    // xTaskCreate(control_motor_turn, "PID Update", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

    // Start FreeRTOS scheduler
    vTaskStartScheduler();

    // This point will not be reached because the scheduler is running
    while (1) {
        tight_loop_contents();
    }

    return 0;
}
