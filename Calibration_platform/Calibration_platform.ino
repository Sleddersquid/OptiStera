#include "variables.h"
#include <math.h>

// How to control the actuators (motors), file organising and calibrate function are taken from,  
// https://github.com/progressiveautomations/Stewart-Platform/blob/master/arduino/platform/platform.ino

// To do list
// 1. Add ENABLE PRINT macro for SerialUSB.print()
// 2. Shorten the list of global variables, which is all for now.


// All global variables. No bueno. 
int time_read;
int time_set[NUM_MOTORS];

int desired_pos[NUM_MOTORS];
int current_pos[NUM_MOTORS];
int pos_diff[NUM_MOTORS];
int pwm_value[NUM_MOTORS];
bool direction[NUM_MOTORS];

// For calibration
int16_t end_readings[NUM_MOTORS];
int16_t zero_readings[NUM_MOTORS];
bool calibration_valid = true;

bool reset_bool = true;

void calibrate()
{
  if (!CALIBRATE) { return; }

  // calibration_valid = false;

  // Extend all actuators
  for (int kth_motor = 0; kth_motor < NUM_MOTORS; ++kth_motor)
  {
    // Start with extension
    digitalWrite(MOTOR_DIR_PINS[kth_motor], EXTEND);
    analogWrite(MOTOR_PWM_PINS[kth_motor], MAX_PWM);
  }
  delay(RESET_DELAY);

  // Stop the extension, get averaged analog readings
  for (int kth_motor = 0; kth_motor < NUM_MOTORS; ++kth_motor)
  {
    analogWrite(MOTOR_PWM_PINS[kth_motor], 0);
    end_readings[kth_motor] = readMotor_pos(MOTOR_POT_PINS[kth_motor]);

    // SerialUSB.print("kth_motor: ");
    // SerialUSB.print(kth_motor);

    // SerialUSB.print("\t end_readings: ");
    // SerialUSB.println(end_readings[kth_motor]);

    // SerialUSB.print("threshold: ");
    // SerialUSB.print(abs(end_readings[kth_motor] - END_POS[kth_motor]));

    // Check if the kth_motors are powered (reading is valid)
    calibration_valid = (abs(end_readings[kth_motor] - END_POS[kth_motor]) < OFF_THRESHOLD);
    if (!calibration_valid) { break; }
  }

  while (true);

  // Retract all actuators
  for (int kth_motor = 0; kth_motor < NUM_MOTORS; ++kth_motor)
  {
    digitalWrite(MOTOR_DIR_PINS[kth_motor], RETRACT);
    analogWrite(MOTOR_PWM_PINS[kth_motor], MAX_PWM);
  }
  delay(RESET_DELAY);

  // Stop the retraction, get averaged analog readings
  if (calibration_valid)
  {
    for (int kth_motor = 0; kth_motor < NUM_MOTORS; ++kth_motor)
    {
      analogWrite(MOTOR_PWM_PINS[kth_motor], 0);
      zero_readings[kth_motor] = readMotor_pos(MOTOR_POT_PINS[kth_motor]);

      // SerialUSB.print("kth_motor: ");
      // SerialUSB.print(kth_motor);

      // SerialUSB.print("\t zero_readings: ");
      // SerialUSB.println(zero_readings[kth_motor]);

      // SerialUSB.print("threshold: ");
      // SerialUSB.print(abs(zero_readings[kth_motor] - ZERO_POS[kth_motor]));

      // Check if the motors are powered (reading is valid)
      calibration_valid = (abs(zero_readings[kth_motor] - ZERO_POS[kth_motor]) < OFF_THRESHOLD);
      if (!calibration_valid) { break; }
    }
  }

  SerialUSB.print("calibration_valid: ");
  SerialUSB.println(calibration_valid);

  for (int kth_motor = 0; kth_motor < NUM_MOTORS; ++kth_motor)
    {
      SerialUSB.print(" Motor ");
      SerialUSB.print(kth_motor);
      SerialUSB.print(": ");
      SerialUSB.print(zero_readings[kth_motor]);
      SerialUSB.print(", ");
      SerialUSB.print(end_readings[kth_motor]);

    }

  SerialUSB.println("");

  // Set the new calibration values if found to be valid
  if (calibration_valid)
  {
    for (int kth_motor = 0; kth_motor < NUM_MOTORS; ++kth_motor)
    {
      END_POS[kth_motor] = end_readings[kth_motor];
      ZERO_POS[kth_motor] = zero_readings[kth_motor];
    }
  }
}

void setup()
{
  // While not connected to the USB port, do not proceed
  // while (!SerialUSB);

  SerialUSB.begin(BAUD_RATE);
  // analogReadResolution(12);
  // pinMode(POTENSIAMETER_PIN, INPUT);

  // -------- MOTOR SETUP -------- //
  pinMode(MOTOR_POT_PIN_1, INPUT);
  pinMode(MOTOR_POT_PIN_2, INPUT);
  pinMode(MOTOR_POT_PIN_3, INPUT);

  pinMode(MOTOR_DIR_PIN_1, OUTPUT);
  pinMode(MOTOR_DIR_PIN_2, OUTPUT);
  pinMode(MOTOR_DIR_PIN_3, OUTPUT);

  pinMode(MOTOR_PWM_PIN_1, OUTPUT);
  pinMode(MOTOR_PWM_PIN_2, OUTPUT);
  pinMode(MOTOR_PWM_PIN_3, OUTPUT);

  pinMode(ENABLE_MOTORS, OUTPUT);
  // Is inverted, so /Enable
  digitalWrite(ENABLE_MOTORS, LOW);

  calibrate();

  digitalWrite(ENABLE_MOTORS, HIGH);
}

int readMotor_pos(int pin)
{
  int reads = 0;
  for (int i = 0; i < MEASUREMENTS; i++)
  {
    reads += analogRead(pin);
  }
  return reads / MEASUREMENTS;
}

void loop()
{
  while (!calibration_valid);
}