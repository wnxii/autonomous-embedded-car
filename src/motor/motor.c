#include "FreeRTOS.h"
#include "task.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include <stdbool.h>

// Define GPIO pins
#define PWM_PIN1 2     // GP2 for PWM
#define DIR_PIN1 0     // GP0 for direction
#define DIR_PIN2 1     // GP1 for direction

#define PWM_PIN2 3     // GP3 for PWM
#define DIR_PIN3 26    // GP26 for direction
#define DIR_PIN4 27    // GP27 for direction

#define BTN_PIN20 20   // Forward Backward Movement
#define BTN_PIN21 21   // Left movement 
#define BTN_PIN22 22   // Right movement

// Motor control structure
typedef struct {
    uint32_t pwm_pin;
    uint32_t dir_pin1;
    uint32_t dir_pin2;
    float speed;
} MotorConfig;

// Global motor configurations
static MotorConfig motor1 = {PWM_PIN1, DIR_PIN1, DIR_PIN2, 0.0f};
static MotorConfig motor2 = {PWM_PIN2, DIR_PIN3, DIR_PIN4, 0.0f};

// Function to set up the PWM
void setup_pwm(uint32_t gpio, float freq, float duty_cycle) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint32_t slice_num = pwm_gpio_to_slice_num(gpio);
    
    float clock_freq = 125000000.0f;
    uint32_t divider = clock_freq / (freq * 65536);
    pwm_set_clkdiv(slice_num, divider);
    pwm_set_wrap(slice_num, 65535);
    pwm_set_gpio_level(gpio, (uint16_t)(duty_cycle * 65535));
    pwm_set_enabled(slice_num, true);
}

// Function to control motor direction and speed
void set_motor(MotorConfig* motor, bool forward, float speed) {
    gpio_put(motor->dir_pin1, forward);
    gpio_put(motor->dir_pin2, !forward);
    setup_pwm(motor->pwm_pin, 3000.0f, speed);
}

// Task to handle motor control
void motor_control_task(void *pvParameters) {
    while (1) {
        // Forward/Backward Movement
        if (!gpio_get(BTN_PIN20)) {
            set_motor(&motor1, true, 0.5f);   // Forward, 50% speed
            set_motor(&motor2, true, 0.5f);   // Forward, 50% speed
            vTaskDelay(pdMS_TO_TICKS(2000));  // Run for 2 seconds
            
            set_motor(&motor1, false, 0.5f);  // Backward, 50% speed
            set_motor(&motor2, false, 0.5f);  // Backward, 50% speed
            vTaskDelay(pdMS_TO_TICKS(2000));  // Run for 2 seconds
        }
        // Left Movement
        else if (!gpio_get(BTN_PIN21)) {
            set_motor(&motor1, true, 0.9f);   // Forward, 90% speed
            set_motor(&motor2, true, 0.3f);   // Forward, 30% speed
            vTaskDelay(pdMS_TO_TICKS(2000));  // Run for 2 seconds
            
            set_motor(&motor1, false, 0.9f);  // Backward, 90% speed
            set_motor(&motor2, false, 0.3f);  // Backward, 30% speed
            vTaskDelay(pdMS_TO_TICKS(2000));  // Run for 2 seconds
        }
        // Right Movement
        else if (!gpio_get(BTN_PIN22)) {
            set_motor(&motor1, true, 0.3f);   // Forward, 30% speed
            set_motor(&motor2, true, 0.9f);   // Forward, 90% speed
            vTaskDelay(pdMS_TO_TICKS(2000));  // Run for 2 seconds
            
            set_motor(&motor1, false, 0.3f);  // Backward, 30% speed
            set_motor(&motor2, false, 0.9f);  // Backward, 90% speed
            vTaskDelay(pdMS_TO_TICKS(2000));  // Run for 2 seconds
        }
        else {
            // Stop motors when no button is pressed
            set_motor(&motor1, true, 0.0f);
            set_motor(&motor2, true, 0.0f);
        }
        
        vTaskDelay(pdMS_TO_TICKS(50));  // Small delay to prevent tight polling
    }
}

int main() {
    // Initialize GPIO pins
    // Motor 1
    gpio_init(DIR_PIN1);
    gpio_init(DIR_PIN2);
    gpio_set_dir(DIR_PIN1, GPIO_OUT);
    gpio_set_dir(DIR_PIN2, GPIO_OUT);

    // Motor 2
    gpio_init(DIR_PIN3);
    gpio_init(DIR_PIN4);
    gpio_set_dir(DIR_PIN3, GPIO_OUT);
    gpio_set_dir(DIR_PIN4, GPIO_OUT);

    // Buttons
    gpio_init(BTN_PIN20);
    gpio_init(BTN_PIN21);
    gpio_init(BTN_PIN22);
    gpio_set_dir(BTN_PIN20, GPIO_IN);
    gpio_set_dir(BTN_PIN21, GPIO_IN);
    gpio_set_dir(BTN_PIN22, GPIO_IN);
    gpio_pull_up(BTN_PIN20);
    gpio_pull_up(BTN_PIN21);
    gpio_pull_up(BTN_PIN22);

    // Create the motor control task
    xTaskCreate(
        motor_control_task,       // Function that implements the task
        "MotorControl",          // Text name for the task
        configMINIMAL_STACK_SIZE, // Stack size in words, not bytes
        NULL,                    // Parameter passed into the task
        1,                       // Task priority
        NULL                     // Task handle
    );

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();

    // We should never get here as control is now taken by the scheduler
    while(1);
    return 0;
}