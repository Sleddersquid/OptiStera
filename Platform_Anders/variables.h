// Header file containing software configurations, constants, and clarifying definitions for the Arduino.
//
#pragma once

#include "pins.h"

// Platform parameters
#define NUM_MOTORS 3

// Actuator value bounds
#define MIN_POS 0
#define MAX_POS 1024
#define MIN_PWM 30
#define MAX_PWM 255

#define ZERO_POS 350
#define END_POS 972

#define POS_THRESHOLD 4

// Movement parameters
#define RESET_DELAY 4000      // at full PWM, the actuator should fully extend/retract by 4s (6" stroke, 2.00"/s)
typedef enum _MotorDirection  // to clarify the direction in which actuators move
{
    RETRACT = 0,
    EXTEND  = 1
} MotorDirection;

// PID feedback parameters
// const uint8_t POS_THRESHOLD[NUM_MOTORS] = { 4, 4, 4 }; // 4, 4, 4,
#define P_COEFF 2
#define I_COEFF 0.035 // 0.03, 0.035, 0.03,
#define D_COEFF 0.02 // 0.025, 0.025, 0.025,

// Serial configuration parameters
#define BAUD_RATE 115200  // baud rate for serial port (also needs to be set on host side)

// Serial input parameters
#define INPUT_TRIGGER 2   // at minimum, 3 numbers + 5 spaces, was 10
#define NUM_READINGS  255  // number of analog readings to average to acquire position