#include "variables.h"
#include <math.h>

// How to control the actuators (motors), file organising and calibrate function are taken from,
// https://github.com/progressiveautomations/Stewart-Platform/blob/master/arduino/platform/platform.ino

// To do list
// 1. Add ENABLE PRINT macro for SerialUSB.print()?
// 2. Shorten the list of global variables, which is all for now.
// 3. Add enable and diabling of H-bridge
// 4. Is it a concern that the lights light up when reset is done?
// 5. IMPORTANT: Add led/light on the button

// #include <LiquidCrystal_I2C.h>

// LiquidCrystal_I2C lcd(LCD_ADDRESS, 16, 2);

// Enum for different speeds for the actuator
enum MotorSpeed
{
  SLOW = 11000,    // in ms
  MODERATE = 8000, // in ms
  FAST = 5000      // in ms
};

// ENUM for modes
enum running_modes
{
  IDLE,
  SET_TIME,
  RUNNING,
  RESET,
  STOPPING
};

// When the program starts, the actuators reset and go to IDLE
volatile running_modes mode{ STOPPING };

// The period decides how fast the actuators are to move
int period = MODERATE;
int next_period = period;

// All global variables. No bueno.
uint32_t time_read, last_time;

int time_set[NUM_MOTORS];

int desired_pos[NUM_MOTORS];
int current_pos[NUM_MOTORS];
int pos_diff[NUM_MOTORS];
int pwm_value[NUM_MOTORS];
bool direction[NUM_MOTORS];

// Only used for calibration
int16_t end_readings[NUM_MOTORS];
int16_t zero_readings[NUM_MOTORS];
bool calibration_valid = true; // If no calibration is done, assume it has been done previously

// bool reset_bool = true;

const int debounce_interval = 200;
volatile unsigned long last_interrupt = 0;

void change_state()
{
  bool button_state = read_digital_avg(MODE_BUTTON);

  if (button_state == 0)
  {
    return;
  }

  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  // If interrupts come faster than 2000ms, assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time > debounce_interval)
  {
    SerialUSB.println("BUTTON WAS PRESSED FOR FUCKS SACKE");
    // Don't need to jump to RUNNING and IDLE, Because
    // SET_TIME -> RUNNING (stay until interrupt) and STOPPING -> IDLE (stay until interrupt)
    if (mode == IDLE)
    {
      mode = SET_TIME;
    }
    else if (mode == RUNNING)
    {
      mode = STOPPING;
    }
    // else if (mode == STOPPING) {mode = IDLE;}
  }
  last_interrupt_time = interrupt_time;
}

float positionFunction(int t, float bias)
{
  return -AMPLUTIDE * cos(((2 * PI * t) / period) + bias) + VERTICAL_SHIFT;
}

void calibrate()
{
  if (!CALIBRATE)
  {
    return;
  }

  // Extend all actuators
  for (int kth_motor = 0; kth_motor < NUM_MOTORS; ++kth_motor)
  {
    // Start with extension
    digitalWrite(MOTOR_DIR_PINS[kth_motor], EXTEND);
    analogWrite(MOTOR_PWM_PINS[kth_motor], MAX_PWM);
  }
  delay(RESET_DELAY);

  // Stop the extension, get avrage anlogue readings
  for (int kth_motor = 0; kth_motor < NUM_MOTORS; ++kth_motor)
  {
    analogWrite(MOTOR_PWM_PINS[kth_motor], 0);
    end_readings[kth_motor] = read_analogue_avg(MOTOR_POT_PINS[kth_motor]);

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
  for (int kth_motor = 0; kth_motor < NUM_MOTORS; ++kth_motor)
  {
    analogWrite(MOTOR_PWM_PINS[kth_motor], 0);
    zero_readings[kth_motor] = read_analogue_avg(MOTOR_POT_PINS[kth_motor]);

    // Check if the motors are powered (reading is valid)
    calibration_valid = (abs(zero_readings[kth_motor] - ZERO_POS[kth_motor]) < OFF_THRESHOLD);
    // if (!calibration_valid) { break; }
  }

  // IF CALIBRATION IS TO BE DONE MANUALY, uncomment this
  // SerialUSB.print("calibration_valid: ");
  // SerialUSB.println(calibration_valid);

  // for (int kth_motor = 0; kth_motor < NUM_MOTORS; ++kth_motor)
  //   {
  //     SerialUSB.print(" Motor ");
  //     SerialUSB.print(kth_motor);
  //     SerialUSB.print(": ");
  //     SerialUSB.print(zero_readings[kth_motor]);
  //     SerialUSB.print(", ");
  //     SerialUSB.print(end_readings[kth_motor]);
  //   }

  // SerialUSB.println("");

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

void move_to_pos(uint32_t set_time)
{
  String to_lcd_display = "";

  // Determine the position for each actuator
  for (int kth_motor = 0; kth_motor < NUM_MOTORS; kth_motor++)
  {
    time_set[kth_motor] = set_time % period;
    desired_pos[kth_motor] = positionFunction(time_set[kth_motor], MOTOR_BIAS[kth_motor]);
  }

  // Get the current positon of all actuators and set dirrection based of the
  for (int kth_motor = 0; kth_motor < NUM_MOTORS; kth_motor++)
  {
    current_pos[kth_motor] = map(read_analogue_avg(MOTOR_POT_PINS[kth_motor]), ZERO_POS[kth_motor], END_POS[kth_motor], MIN_POS, MAX_POS);

    pos_diff[kth_motor] = desired_pos[kth_motor] - current_pos[kth_motor];

    to_lcd_display += String(kth_motor) + ":" + String(pos_diff[kth_motor] * PWM_GAIN) + " ";

    // SerialUSB.println(to_lcd_display.length());

    // Se nærmere på
    if (current_pos[kth_motor] < desired_pos[kth_motor])
    {
      direction[kth_motor] = EXTEND;
    }
    else
    {
      direction[kth_motor] = RETRACT;
    }
  }
  SerialUSB.println(to_lcd_display);

  // move the motors, with speed (pwm) and direction
  for (int kth_motor = 0; kth_motor < NUM_MOTORS; kth_motor++)
  {
    if (abs(pos_diff[kth_motor]) >= POS_THRESHOLD)
    {
      pwm_value[kth_motor] = constrain(abs(pos_diff[kth_motor]) * PWM_GAIN, MIN_PWM, MAX_PWM);

      digitalWrite(MOTOR_DIR_PINS[kth_motor], direction[kth_motor]);
      analogWrite(MOTOR_PWM_PINS[kth_motor], pwm_value[kth_motor]);
    }
    else
    {
      analogWrite(MOTOR_PWM_PINS[kth_motor], 0);
    }
  }
}

// Will retract all the motors
void reset_motors(uint16_t point_to_reset_to)
{
  int motor_count_reset = 0;

  // Retract all motors to zero position
  for (int kth_motor = 0; kth_motor < NUM_MOTORS; kth_motor++)
  {
    digitalWrite(MOTOR_DIR_PINS[kth_motor], RETRACT);
    analogWrite(MOTOR_PWM_PINS[kth_motor], MAX_PWM / 2); // Running at half speed
  }

  // While not all motors are fully reset, read each motor individually and
  while (motor_count_reset != NUM_MOTORS)
  {
    // SerialUSB.println("adad");
    for (int kth_motor = 0; kth_motor < NUM_MOTORS; kth_motor++)
    {
      current_pos[kth_motor] = map(read_analogue_avg(MOTOR_POT_PINS[kth_motor]), ZERO_POS[kth_motor], END_POS[kth_motor], MIN_POS, MAX_POS);

      if (abs(current_pos[kth_motor] - point_to_reset_to) < POS_THRESHOLD)
      {
        motor_count_reset++;
      }
    }
  }
}

void setup()
{
  // For the three position switch
  pinMode(LEFT_SWITCH_PIN, INPUT);
  pinMode(RIGTH_SWITCH_PIN, INPUT);

  pinMode(MODE_BUTTON, INPUT);
  attachInterrupt(digitalPinToInterrupt(MODE_BUTTON), change_state, RISING);

  SerialUSB.begin(BAUD_RATE);
  // analogReadResolution(10); // 10 bit is default

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

  delay(2000);
}

int read_analogue_avg(int pin)
{
  int reads = 0;
  for (int i = 0; i < ACTUATOR_MEASUREMENTS; i++)
  {
    reads += analogRead(pin);
  }
  return reads / ACTUATOR_MEASUREMENTS;
}

bool read_digital_avg(int pin)
{
  int reads = 0;
  for (int i = 0; i < BUTTON_MEASUREMENTS; i++)
  {
    reads += digitalRead(pin);
  }
  return (bool)(reads / BUTTON_MEASUREMENTS);
}

void loop()
{
  // If no claibration is done, we assume that calibration is already done previously

  // 10 - SLOW mode      - Switch is at the left most position
  // 01 - FAST mode      - Switch is at the right most position
  // 00 - Moderate mode  - Switch is at the middle position
  if (read_digital_avg(LEFT_SWITCH_PIN) == 1) next_period = SLOW;
  else if (read_digital_avg(RIGTH_SWITCH_PIN) == 1) next_period = FAST;
  else next_period = MODERATE;

  if (next_period != period)
  {
    period = next_period;
    if (mode == RUNNING)
      mode = RESET;
  }
  // SerialUSB.println(mode);

  time_read = millis();
  switch (mode)
  {
  case IDLE:
    // digitalWrite(ENABLE_MOTORS, HIGH); // Diables the H-bridge
    break;

  case SET_TIME:
    // digitalWrite(ENABLE_MOTORS, LOW); // Enables the H-bridge
    last_time = time_read;
    mode = RUNNING;
    break;

  case RUNNING:
    move_to_pos(time_read - last_time);
    break;

  case RESET:
    mode = SET_TIME;
    reset_motors(VERTICAL_SHIFT - AMPLUTIDE);
    break;

  case STOPPING:
    SerialUSB.println("Stopping...");
    reset_motors(0);
    mode = IDLE;
    SerialUSB.println("Stopped");
    break;
  }
}