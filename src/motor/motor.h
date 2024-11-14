#ifndef motor_H
#define motor_H


#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

// Define GPIO pins
#define PWM_PIN1 2     // GP2 for PWM (Left Motor)
#define DIR_PIN1 0     // Direction pin 1 for Left Motor (L298N IN1)
#define DIR_PIN2 1     // Direction pin 2 for Left Motor (L298N IN2)

#define PWM_PIN2 3     // GP3 for PWM (Right Motor)
#define DIR_PIN3 8    // Direction pin 1 for Right Motor (L298N IN3)
#define DIR_PIN4 9     // Direction pin 2 for Right Motor (L298N IN4)

// PID Constants (Tuned at home)
/* #define KP 0.005f
#define KI 0.0030f 
#define KD 0.0040f */

// PID Constants (Tuned in school - can be refine further)
/* #define KP 0.009f 
#define KI 0.002f 
#define KD 0.006f  */

#define KP 0.002f 
#define KI 0.000f 
#define KD 0.000f

#define M_PI 3.14159265358979323846

// Min and Max duty cycles for PID Calculation - forward movement
#define MAX_DUTY_CYCLE 1.0f
#define MIN_DUTY_CYCLE 0.0f

// Min and Max duty cycles for line following
#define MAX_LINE_DUTY_CYCLE 0.60f
#define MIN_LINE_DUTY_CYCLE 0.50f

// Movement Enum for clear direction control
typedef enum {
    FORWARD,
    BACKWARD,
    STOP,
    PIVOT_LEFT,
    PIVOT_RIGHT,
    MOTOR_ON_LINE
} MovementDirection;

// Motor control structure
typedef struct {
    uint32_t pwm_pin;
    uint32_t dir_pin1;
    uint32_t dir_pin2;
    float current_speed;
    float target_speed;
    float target_position;
    float current_position;
} MotorConfig;

// PID control variables
typedef struct {
    float integral;
    float prev_error;
} PIDState;

// Prototype functions
void init_motor();
void move_car(MovementDirection direction, float speed, float angle);

// Global values to be used in main()
extern bool turning_active;
extern volatile int stop_running;

#endif