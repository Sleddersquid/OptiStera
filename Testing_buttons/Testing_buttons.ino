#include "variables.h"
#include <math.h>

// How to control the actuators (motors), file organising and calibrate function are taken from,
// https://github.com/progressiveautomations/Stewart-Platform/blob/master/arduino/platform/platform.ino

// To do list
// 1. Add ENABLE PRINT macro for SerialUSB.print()
// 2. Shorten the list of global variables, which is all for now.

typedef enum _MotorSpeed // The period decides how fast the actuators are to move
{
  SLOW = 11000,
  MODERATE = 9000,
  FAST = 7000
} MotorSpeed;

static float PERIOD = 8000; // in ms
static float desired_PERIOD = PERIOD;
static float d_PERIOD;

// All global variables. No bueno.
static int time_read, last_time;

int time_set[NUM_MOTORS];

static int desired_pos[NUM_MOTORS];
static int current_pos[NUM_MOTORS];
static int pos_diff[NUM_MOTORS];
static int pwm_value[NUM_MOTORS];
static bool direction[NUM_MOTORS];

// For calibration
int16_t end_readings[NUM_MOTORS];
int16_t zero_readings[NUM_MOTORS];
bool calibration_valid = true;

bool reset_bool = true;

const int debounce_interval{700};
volatile unsigned long last_interrupt{0};

// ENUM for states
enum running_modes
{
  IDLE,
  RUNNING,
  STOPPING
};

volatile running_modes mode{STOPPING};

void change_state()
{
  bool button_state = read_switch_pos(MODE_BUTTON);

  if (button_state == 0) return;

  SerialUSB.print(button_state);
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  // If interrupts come faster than 200ms, assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time > debounce_interval)
  {
    // Don't need to jump to RUNNING and IDLE, Because
    // SET_TIME -> RUNNING (stay until interrupt) and STOPPING -> IDLE (stay until interrupt)
    if (mode == IDLE)
    {
      mode = RUNNING;
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
  return -AMPLUTIDE * cos(((2 * PI * t) / PERIOD) + bias) + VERTICAL_SHIFT;
}

float periodFunction_pos(int t)
{
  return 0.5 * -atan((2 * PI * t) / PERIOD) + 1;
}

float periodFunction_neg(int t)
{
  return 0.5 * atan((2 * PI * t) / PERIOD);
}

void move_to_pos()
{
  // determine distance difference and set direction
  for (int kth_motor = 0; kth_motor < NUM_MOTORS; kth_motor++)
  {
    current_pos[kth_motor] = map(readMotor_pos(MOTOR_POT_PINS[kth_motor]), ZERO_POS[kth_motor], END_POS[kth_motor], MIN_POS, MAX_POS);

    // SerialUSB.print(" Motor ");
    // SerialUSB.print(kth_motor);
    // SerialUSB.print(": ");
    // SerialUSB.print(current_pos[kth_motor]);

    pos_diff[kth_motor] = desired_pos[kth_motor] - current_pos[kth_motor];

    // SerialUSB.print(", ");
    // SerialUSB.print(pos_diff[kth_motor]);

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

  // SerialUSB.println("");

  // move the motors, with speed (pwm) and direction
  for (int kth_motor = 0; kth_motor < NUM_MOTORS; kth_motor++)
  {
    if (abs(pos_diff[kth_motor]) >= POS_THRESHOLD)
    {
      pwm_value[kth_motor] = constrain(abs(pos_diff[kth_motor]) * PWM_GAIN, MIN_PWM, MAX_PWM);

      // SerialUSB.print("PWM value: ");
      // SerialUSB.println(pwm_value[kth_motor]);

      digitalWrite(MOTOR_DIR_PINS[kth_motor], direction[kth_motor]);
      analogWrite(MOTOR_PWM_PINS[kth_motor], pwm_value[kth_motor]);
    }
    else
    {
      analogWrite(MOTOR_PWM_PINS[kth_motor], 0);
    }
  }
}

void reset_motors()
{

  // Will reset the motors
  for (int kth_motor = 0; kth_motor < NUM_MOTORS; kth_motor++)
  {
    // Start with extension
    digitalWrite(MOTOR_DIR_PINS[kth_motor], RETRACT);
    analogWrite(MOTOR_PWM_PINS[kth_motor], 128);
  }
  delay(7000);
}

// attachInterrupt(digitalPinToInterrupt(pin), ISR [function pointer], mode);

void setup()
{

  // While not connected to the USB port, do not proceed
  // while (!SerialUSB);
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

  delay(1000);
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

bool read_switch_pos(int pin)
{
  int reads = 0;
  for (int i = 0; i < 10; i++)
  {
    reads += digitalRead(pin);
  }
  return (bool)(reads / 10);
}

void loop()
{
  // If no claibration is done, we assume that calibration is already done previously
  time_read = millis();

  if (read_switch_pos(LEFT_SWITCH_PIN) == 1)
  {
    // SerialUSB.println("Slow mode");
    PERIOD = SLOW;
  }
  else if (read_switch_pos(RIGTH_SWITCH_PIN) == 1)
  {
    // SerialUSB.println("Fast mode");
    PERIOD = MODERATE;
  }
  else
  {
    // SerialUSB.println("Moderate mode");
    PERIOD = FAST;
  }

  // if (desired_PERIOD != PERIOD) {
  //   if (desired_PERIOD < PERIOD) { //
  //     PERIOD -= 0.5 * periodFunction_neg(time_read); // The derivative of positionFunction
  //   } else {
  //     PERIOD += periodFunction_pos(time_read);
  //   }
  //   SerialUSB.println(PERIOD);
  // }

  switch (mode)
  {
  case IDLE:
    // DO nothing. Waiting for user to press
    SerialUSB.println("Idle");
    break;
  case RUNNING:
    move_to_pos();
    break;
  case STOPPING:
    SerialUSB.println("Stopping...");
    reset_motors();
    mode = IDLE;
    SerialUSB.println("Stopped");
    break;
  }
}