/**
 * @file StuwaldoV3/variables.h
 * @author OptiStera 
 * @brief Actuator configuration, math constants and LCD display variables. Configuration is enabled/disabled here. Read before starting program for the first time
*/
#pragma once

#include "pin_layout.h"

/// Defines the time for the watchdog, in ms
#define WATCHDOG_TIMEOUT 50

/// The number of actuators in the platform.
#define NUM_ACTUATORS 3

/// Pi for the cosine function in \f$ l_k(t)\f$
#define PI 3.1415926535897932384626433832795

/// Configuration constant for \f$ l_k(t)\f$. Amplitude \f$(A)\f$ for the cosine function.
#define AMPLUTIDE 380
/// Configuration constant for \f$ l_k(t)\f$. Vertical shift \f$(C)\f$ for cosine function.
#define VERTICAL_SHIFT 420

/// The speed of the actuators when in state #REPOSITION and #RETURN_HOME, to not induce big moment of inertia.
#define GRADTIENT 2

/// Measurements to take for the actuators, to calculate the average.
#define ACTUATOR_MEASUREMENTS 255 
/// Measurements to take for the buttons, denoise the signal.
#define BUTTON_MEASUREMENTS 20

/// Baud rate for the serial communication.
#define BAUD_RATE 115200

/// If the button should light up according to the states.
#define ENABLE_BUTTON_LED 1

// ----------- LCD DISPLAY ------------ //
#define ENABLE_LCD_DISPLAY 1

#define LCD_ADDRESS 0x27

// ----------- CALIBRATION ------------ //
/// The time to wait before turning the actuators while calibrating, in milliseconds
#define RESET_DELAY 5000

/// If calibration should be performed at startup. See #MANUAL_CALIBRATION for configuration.
#define CALIBRATE 0

/** @brief If calibration should be performed manually or automatically.
 * - Automatically = 0: Program start with calibrated values
 * - Manual = 1: Serial Monitor. Program will stop even if calibration is not performed. 
*/
#define MANUAL_CALIBRATION 0

/// Used to check if the actuator is powered or not. If the actuator is not powered, then the readings sould exceed this value. 
#define OFF_THRESHOLD 500
/// Readings after calibration, when the actuators are at neutral position
int16_t ZERO_POS[NUM_ACTUATORS]      = {48, 47, 43};
/// Readings after calibration, when the actuators are fully extended
int16_t END_POS[NUM_ACTUATORS]       = {671, 675, 669};

// ------------ MATH CONSTANTS -------------- //
/// If a bias should be induced on the actuators. Enables wave motion (heave and pitch).
#define ENABLE_BIAS 0

/// \f$ B_1 \in \left[0, 2\pi \right\rangle \f$
#define ACTUATOR_BIAS_1 0
/// \f$ B_2 \in \left[0, 2\pi \right\rangle \f$
#define ACTUATOR_BIAS_2 0
/// \f$ B_3 \in \left[0, 2\pi \right\rangle \f$
#define ACTUATOR_BIAS_3 0

// Wave motion (heave and pitch)
#if ENABLE_BIAS
  #define ACTUATOR_BIAS_1 1*PI/8
  #define ACTUATOR_BIAS_2 0
  #define ACTUATOR_BIAS_3 0
#endif // ENABLE_BIAS

/// Array for the actuators bias
const float ACTUATOR_BIAS[NUM_ACTUATORS] = {ACTUATOR_BIAS_1, ACTUATOR_BIAS_2, ACTUATOR_BIAS_3};

/// If the actuators do not have enough speed with the set configurations, increase #PWM_GAIN
#define PWM_GAIN 1

// ------------ ACTUATOR CONFIGURATION -------------- //
/// The minimum position for the actuators, after calibration
#define MIN_POS 0
/// The maximum position for the actuators, after calibration
#define MAX_POS 1023

/// Min speed for the actuators
#define MIN_PWM 80
/// Max speed for the actuators
#define MAX_PWM 255

/// The margin of error to accept in position for the actuators
#define POS_THRESHOLD 10

/// Array for the actuators direction pins
const uint8_t ACTUATOR_DIR_PINS[NUM_ACTUATORS] = {ACTUATOR_DIR_PIN_1, ACTUATOR_DIR_PIN_2, ACTUATOR_DIR_PIN_3};
/// Array for the actuators PWM pins
const uint8_t ACTUATOR_PWM_PINS[NUM_ACTUATORS] = {ACTUATOR_PWM_PIN_1, ACTUATOR_PWM_PIN_2, ACTUATOR_PWM_PIN_3};
/// Array for the actuators potentiometer pins
const uint8_t ACTUATOR_POT_PINS[NUM_ACTUATORS] = {ACTUATOR_POT_PIN_1, ACTUATOR_POT_PIN_2, ACTUATOR_POT_PIN_3};
