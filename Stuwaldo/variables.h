#pragma once

#include "pin_layout.h"

#define NUM_MOTORS 3

#define PI 3.1415926535897932384626433832795

#define PERIOD 6000 // in ms
#define AMPLUTIDE 190 // pot values
#define VERTICAL_SHIFT 764 // pot values

#define MEASUREMENTS 255 // measurements to take

#define BAUD_RATE 115200

// ----------- GOOD PLACEMENT ------------ //

#define RESET_DELAY 4000

#define CALIBRATE 0

// ------------ MATH THINGS -------------- //

#define ENABLE_BIAS 1

#define MOTOR_BIAS_1 0
#define MOTOR_BIAS_2 0
#define MOTOR_BIAS_3 0

// Wave motion
#if ENABLE_BIAS
  #define MOTOR_BIAS_1 1*PI/4
  #define MOTOR_BIAS_2 0
  #define MOTOR_BIAS_3 0
#endif // ENABLE_BIAS

const float MOTOR_BIAS[NUM_MOTORS] = {MOTOR_BIAS_1, MOTOR_BIAS_2, MOTOR_BIAS_3};

#define PWM_GAIN 6

// ------------ MOTOR THINGS -------------- //

#define MIN_POS 0
#define MAX_POS 1024
#define MIN_PWM 60
#define MAX_PWM 255

#define POS_THRESHOLD 4

#define OFF_THRESHOLD 500
int16_t ZERO_POS[NUM_MOTORS]      = {350, 345, 350};
int16_t END_POS[NUM_MOTORS]       = {972, 971, 975};

const uint8_t MOTOR_DIR_PINS[NUM_MOTORS] = {MOTOR_DIR_PIN_1, MOTOR_DIR_PIN_2, MOTOR_DIR_PIN_3};
const uint8_t MOTOR_PWM_PINS[NUM_MOTORS] = {MOTOR_PWM_PIN_1, MOTOR_PWM_PIN_2, MOTOR_PWM_PIN_3};
const uint8_t MOTOR_POT_PINS[NUM_MOTORS] = {MOTOR_POT_PIN_1, MOTOR_POT_PIN_2, MOTOR_POT_PIN_3};


typedef enum _MotorDirection  // to clarify the direction in which actuators move
{
    RETRACT = 0,
    EXTEND  = 1
} MotorDirection;


