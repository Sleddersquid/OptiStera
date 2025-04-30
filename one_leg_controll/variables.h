#pragma once

#include "pin_layout.h"

#define NUM_MOTORS 3

#define PI 3.1415926535897932384626433832795

#define PERIOD 4000
#define AMPLUTIDE 200
#define VERTICAL_SHIFT 824

#define MEASUREMENTS 255

#define BAUD_RATE 115200

// ----------- GOOD PLACEMENT ------------ //

#define RESET_DELAY 4000

#define CALIBRATE 0

// ------------ MATH THINGS -------------- //

#define MOTOR_BIAS_1 0*PI 
#define MOTOR_BIAS_2 2*PI/3
#define MOTOR_BIAS_3 4*PI/3

const float MOTOR_BIAS[NUM_MOTORS] = {MOTOR_BIAS_1, MOTOR_BIAS_2, MOTOR_BIAS_3};

#define PWM_GAIN 5

#define OFF_THRESHOLD 500

// ------------ MOTOR THINGS -------------- //


#define MIN_POS 0
#define MAX_POS 1024
#define MIN_PWM 30
#define MAX_PWM 255

#define POS_THRESHOLD 4

int16_t ZERO_POS[NUM_MOTORS]      = {0, 0, 0};
int16_t END_POS[NUM_MOTORS]       = {671, 675, 669};

const uint8_t MOTOR_DIR_PINS[NUM_MOTORS] = {MOTOR_DIR_PIN_1, MOTOR_DIR_PIN_2, MOTOR_DIR_PIN_3};
const uint8_t MOTOR_PWM_PINS[NUM_MOTORS] = {MOTOR_PWM_PIN_1, MOTOR_PWM_PIN_2, MOTOR_PWM_PIN_3};
const uint8_t MOTOR_POT_PINS[NUM_MOTORS] = {MOTOR_POT_PIN_1, MOTOR_POT_PIN_2, MOTOR_POT_PIN_3};


typedef enum _MotorDirection  // to clarify the direction in which actuators move
{
    RETRACT = 0,
    EXTEND  = 1
} MotorDirection;


