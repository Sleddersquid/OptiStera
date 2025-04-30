#include "variables.h"
#include <math.h>

int MOTOR = 1;

int PWM_VALUE = 0;

int pot_value_read;

bool direction;

int var1, var2;

int time_read;
int time_set[NUM_MOTORS];

int plott;

int desired_pos[NUM_MOTORS];
int actual_pos[NUM_MOTORS];
int pos_diff[NUM_MOTORS];

// For calibration
int16_t end_readings[NUM_MOTORS];
int16_t zero_readings[NUM_MOTORS];
bool calibration_valid = true;

void calibrate()
{
  if (!CALIBRATE) { return; }

  calibration_valid = false;

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

    // Check if the kth_motors are powered (reading is valid)
    calibration_valid = (abs(end_readings[kth_motor] - END_POS[kth_motor]) < OFF_THRESHOLD);
    // if (!calibration_valid) { break; }
  }
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

      // Check if the motors are powered (reading is valid)
      calibration_valid = (abs(zero_readings[kth_motor] - ZERO_POS[kth_motor]) < OFF_THRESHOLD);
      // if (!calibration_valid) { break; }
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

float position_function(int t, float bias)
{
  return -AMPLUTIDE * cos(((2 * PI * t) / PERIOD) + bias) + VERTICAL_SHIFT;
}

void move_to_pos()
{
  actual_pos[0] = map(readMotor_pos(MOTOR_POT_PINS[0]), ZERO_POS[0], END_POS[0], MIN_POS, MAX_POS);

  pos_diff[0] = desired_pos[0] - actual_pos[0];

  // Se nærmere på
  if (actual_pos[0] > desired_pos[0]){
    direction = EXTEND;
  } else {
    direction = RETRACT;
  }

  if (abs(pos_diff[0]) >= POS_THRESHOLD)
  {
    int PWM_VALUE = constrain(abs(pos_diff[0]) * PWM_GAIN, MIN_PWM, MAX_PWM);

    // SerialUSB.print("PWM value: ");
    // SerialUSB.println(PWM_VALUE);

    digitalWrite(MOTOR_DIR_PINS[0], direction);
    analogWrite(MOTOR_PWM_PINS[0], PWM_VALUE);
  }
  else
  {
    analogWrite(MOTOR_PWM_PINS[0], 0);
  }
}

void setup()
{

  while (!SerialUSB); // While not connected to the USB port, do not proceed

  SerialUSB.begin(BAUD_RATE);
  pinMode(POTENSIAMETER_PIN, INPUT);

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
  // will not run unless calibration is valid.
  // If no claibration is done, we assume that calibration is already done previously
  // while (!calibration_valid);

  // actual_pos = map(readMany(255, POT_PIN), ZERO_POS, END_POS, MIN_POS, MAX_POS);

  time_read = millis();

  time_set[MOTOR] = time_read % PERIOD;

  // desired_pos = readMany(255, POT_PIN_V);
  desired_pos[MOTOR] = position_function(time_set[MOTOR], MOTOR_BIAS[MOTOR]);

  if (true) {
    move_to_pos();
  } else {
    analogWrite(MOTOR_POT_PINS[MOTOR], MOTOR);
  }

  SerialUSB.print("Desired pos: ");
  SerialUSB.println(desired_pos[MOTOR]);

  SerialUSB.print("Actual pos: ");
  SerialUSB.println(actual_pos[MOTOR]);
}