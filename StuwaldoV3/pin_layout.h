/**
 * @file StuwaldoV3/pin_layout.h
 * @author OptiStera
 * @brief Pin layout for the actuators, buttons, three position switch and H-bridge
 */

#pragma once

#define ANALOGUE_DISCORVEY_PIN 40

/// Pin for when the emergency has been activated.
#define EMERGENCY_STOP_PIN 50

/// For the button pin that changes states.
#define STATE_BUTTON_PIN 53

/// The LED pin on the button.
#define BUTTON_LED_PIN 48

/// Pin for the three switch position, when at left position \f$ (10_2) \f$.
#define LEFT_SWITCH_PIN 52
/// Pin for the three switch position, when at right position \f$ (01_2) \f$.
#define RIGTH_SWITCH_PIN 51

#define ACTUATOR_POT_PIN_1 A9
#define ACTUATOR_POT_PIN_2 A8
#define ACTUATOR_POT_PIN_3 A7

#define ACTUATOR_DIR_PIN_1 35
#define ACTUATOR_DIR_PIN_2 33
#define ACTUATOR_DIR_PIN_3 31

#define ACTUATOR_PWM_PIN_1 10
#define ACTUATOR_PWM_PIN_2 9
#define ACTUATOR_PWM_PIN_3 8

/// The pin for the H-bridge, which is connected to the actuators.
#define PIN_H_BRIDGE 25
