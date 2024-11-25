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
/* #define KP 0.001f
#define KI 0.000f 
#define KD 0.000f */

// PID Constants (Tuned in school - can be refine further)
/* #define KP 0.009f 
#define KI 0.002f 
#define KD 0.006f  */

// PID Constants (Tuned at home. Car can move straight with some deviations and more responsive during steering)
#define KP 0.01f 
#define KI 0.003f 
#define KD 0.005f

/* #define KP 0.09f 
#define KI 0.03f 
#define KD 0.08f */

#define M_PI 3.14159265358979323846

// Min and Max duty cycles for PID Calculation - forward movement
#define MAX_DUTY_CYCLE 1.0f
#define MIN_DUTY_CYCLE 0.0f
#define MAX_SPEED 40.0

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
    STEER_FORWARD_LEFT,
    STEER_FORWARD_RIGHT,
    STEER_BACKWARD_LEFT,
    STEER_BACKWARD_RIGHT
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

typedef struct {
    MovementDirection direction;
    float left_wheel_speed;
    float right_wheel_speed;
} MotorControl;

// Prototype functions
void init_motor();
void move_car(MovementDirection direction, float left_target_speed, float right_target_speed, float angle);
MotorControl map_remote_output_to_direction(int remote_output, int remote_steering);

// Global values to be used in main()
extern bool pivot_turning_active;

#endif