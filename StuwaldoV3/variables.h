#pragma once

#include "pin_layout.h"

#define NUM_ACTUATORS 3

#define PI 3.1415926535897932384626433832795

#define AMPLUTIDE 400 // pot values
#define VERTICAL_SHIFT 511.5 // pot values

#define GRADTIENT 2 // 0.01 // In binary, float point arithmatic

#define ACTUATOR_MEASUREMENTS 255 // measurements to take
#define BUTTON_MEASUREMENTS 30 // measurements to take

#define BAUD_RATE 115200

#define ENABLE_BUTTON_LED 1

// ----------- LCD DISPLAY ------------ //
#define ENABLE_LCD_DISPLAY 1

#define LCD_ADDRESS 0x27

// ----------- GOOD PLACEMENT ------------ //

#define RESET_DELAY 5000

#define CALIBRATE 0

// ------------ MATH THINGS -------------- //

#define ENABLE_BIAS 0

#define ACTUATOR_BIAS_1 0
#define ACTUATOR_BIAS_2 0
#define ACTUATOR_BIAS_3 0

// Wave motion
#if ENABLE_BIAS
  #define ACTUATOR_BIAS_1 1*PI/4
  #define ACTUATOR_BIAS_2 0
  #define ACTUATOR_BIAS_3 0
#endif // ENABLE_BIAS

const float ACTUATOR_BIAS[NUM_ACTUATORS] = {ACTUATOR_BIAS_1, ACTUATOR_BIAS_2, ACTUATOR_BIAS_3};

#define PWM_GAIN 1

// ------------ ACTUATOR CONFIGURATION -------------- //

#define MIN_POS 0
#define MAX_POS 1023
#define MIN_PWM 60
#define MAX_PWM 255 // 255

#define POS_THRESHOLD 4

#define OFF_THRESHOLD 500
int16_t ZERO_POS[NUM_ACTUATORS]      = {45, 47, 43};
int16_t END_POS[NUM_ACTUATORS]       = {671, 675, 669};

const uint8_t ACTUATOR_DIR_PINS[NUM_ACTUATORS] = {ACTUATOR_DIR_PIN_1, ACTUATOR_DIR_PIN_2, ACTUATOR_DIR_PIN_3};
const uint8_t ACTUATOR_PWM_PINS[NUM_ACTUATORS] = {ACTUATOR_PWM_PIN_1, ACTUATOR_PWM_PIN_2, ACTUATOR_PWM_PIN_3};
const uint8_t ACTUATOR_POT_PINS[NUM_ACTUATORS] = {ACTUATOR_POT_PIN_1, ACTUATOR_POT_PIN_2, ACTUATOR_POT_PIN_3};


typedef enum _ACTUATORDirection  // to clarify the direction in which actuators move
{
    RETRACT = 0,
    EXTEND  = 1
} ACTUATORDirection;


