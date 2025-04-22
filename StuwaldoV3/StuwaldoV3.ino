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

uint32_t blink_interval = 1000, previousMillis, current_time;  // in ms

bool ledState = 0;

// Enum for different speeds for the actuator
enum ActuatorSpeed {
  SLOW = 11000,     // in ms
  MODERATE = 8000,  // in ms
  FAST = 5000       // in ms
};

// Enum for modes
enum platform_state {
  IDLE,
  SET_TIME,
  RUNNING,
  RESET,
  STOPPING
};

// When the program starts, the actuators reset and go to IDLE
volatile platform_state current_state = STOPPING; // Will be populated at iteration of loop
platform_state next_state = STOPPING; 

// The period sets the speed for the actuators
uint16_t period = MODERATE;
uint16_t next_period = period;

uint32_t time_read, last_time;
uint32_t time_set[NUM_ACTUATORS];

uint16_t desired_pos[NUM_ACTUATORS];  // Ranges from 0..1023
uint16_t current_pos[NUM_ACTUATORS];  // Ranges from 0..1023
int16_t pos_diff[NUM_ACTUATORS];      // Ranges from -1023..1023
uint8_t pwm_value[NUM_ACTUATORS];     // Ranges from 0..255
bool direction[NUM_ACTUATORS];        // 0 = RETRACT, 1 = EXTEND

// Only used for calibration
int16_t end_readings[NUM_ACTUATORS];   // Readings from fully extended position
int16_t zero_readings[NUM_ACTUATORS];  // Readings from fully retracted position
// If no calibration is done, assume it has been done previously
bool calibration_valid = true;  // Set to false if calibration is not valid

const uint8_t debounce_interval = 200;
volatile unsigned long last_interrupt = 0;

void change_state() {
  bool button_reading = read_digital_avg(MODE_BUTTON);

  if (button_reading == 0) return;

  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  // If interrupts comes faster than debounce_interval, assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time > debounce_interval) {
    // SerialUSB.println("BUTTON WAS PRESSED FOR FUCKS SACKE");
    // Don't need to jump to RUNNING and IDLE, Because
    // SET_TIME -> RUNNING (stay until interrupt) and STOPPING -> IDLE (stay until interrupt)
    if (current_state == IDLE) next_state = SET_TIME;
    else if (current_state == RUNNING || current_state == RESET) next_state = STOPPING;
    last_interrupt_time = interrupt_time;
  }
}

// l_k(t) with k = 1, 2, 3 and t (ms)
float positionFunction(int t, float bias) { 
  return -AMPLUTIDE * cos(((2 * PI * t) / period) + bias) + VERTICAL_SHIFT;
}

void calibrate(bool run_calibration) {
  if (!run_calibration) {
    return;
  }

  // Extend all actuators
  for (int kth_motor = 0; kth_motor < NUM_ACTUATORS; ++kth_motor) {
    // Start with extension
    digitalWrite(ACTUATOR_DIR_PINS[kth_motor], EXTEND);
    analogWrite(ACTUATOR_PWM_PINS[kth_motor], MAX_PWM);
  }
  delay(RESET_DELAY);

  // Stop the extension, get avrage anlogue readings
  for (int kth_motor = 0; kth_motor < NUM_ACTUATORS; ++kth_motor) {
    analogWrite(ACTUATOR_PWM_PINS[kth_motor], 0);
    end_readings[kth_motor] = read_analogue_avg(ACTUATOR_POT_PINS[kth_motor]);

    // Check if the kth_motors are powered (reading is valid)
    calibration_valid = (abs(end_readings[kth_motor] - END_POS[kth_motor]) < OFF_THRESHOLD);
    if (!calibration_valid) { break; }
  }
  // Retract all actuators
  for (int kth_motor = 0; kth_motor < NUM_ACTUATORS; ++kth_motor) {
    digitalWrite(ACTUATOR_DIR_PINS[kth_motor], RETRACT);
    analogWrite(ACTUATOR_PWM_PINS[kth_motor], MAX_PWM);
  }
  delay(RESET_DELAY);

  // Stop the retraction, get averaged analog readings
  for (int kth_motor = 0; kth_motor < NUM_ACTUATORS; ++kth_motor) {
    analogWrite(ACTUATOR_PWM_PINS[kth_motor], 0);
    zero_readings[kth_motor] = read_analogue_avg(ACTUATOR_POT_PINS[kth_motor]);

    // Check if the motors are powered (reading is valid)
    calibration_valid = (abs(zero_readings[kth_motor] - ZERO_POS[kth_motor]) < OFF_THRESHOLD);
    if (!calibration_valid) { break; }
  }

  // IF CALIBRATION IS TO BE DONE MANUALY, uncomment this
  SerialUSB.print("calibration_valid: ");
  SerialUSB.println(calibration_valid);

  for (int kth_motor = 0; kth_motor < NUM_ACTUATORS; ++kth_motor) {
    SerialUSB.print(" Motor ");
    SerialUSB.print(kth_motor);
    SerialUSB.print(": ");
    SerialUSB.print(zero_readings[kth_motor]);
    SerialUSB.print(", ");
    SerialUSB.print(end_readings[kth_motor]);
  }

  SerialUSB.println("");

  // Set the new calibration values if found to be valid
  if (calibration_valid) {
    for (int kth_motor = 0; kth_motor < NUM_ACTUATORS; ++kth_motor) {
      END_POS[kth_motor] = end_readings[kth_motor];
      ZERO_POS[kth_motor] = zero_readings[kth_motor];
    }
  }
}

void move_to_pos(uint32_t set_time, bool reset=false) {
  // If not reset set desired length with l_k(t), else lower actuators to desired pos
  for (int kth_motor = 0; kth_motor < NUM_ACTUATORS; kth_motor++) {
    if (!reset) {
      // Determine the position for each actuator
        time_set[kth_motor] = set_time % period;
        desired_pos[kth_motor] = positionFunction(time_set[kth_motor], ACTUATOR_BIAS[kth_motor]);
      }
    else {
      desired_pos[kth_motor] = desired_pos[kth_motor] - 0.25; // 0.01 // In binary, float point arithmatic
    }
  }

  // Get the current positon of all actuators and set dirrection based of the
  for (int kth_motor = 0; kth_motor < NUM_ACTUATORS; kth_motor++) {
    current_pos[kth_motor] = map(read_analogue_avg(ACTUATOR_POT_PINS[kth_motor]), ZERO_POS[kth_motor], END_POS[kth_motor], MIN_POS, MAX_POS);

    pos_diff[kth_motor] = desired_pos[kth_motor] - current_pos[kth_motor];

    // Se nærmere på
    if (current_pos[kth_motor] < desired_pos[kth_motor]) {
      direction[kth_motor] = EXTEND;
    } else {
      direction[kth_motor] = RETRACT;
    }
  }

  // move the motors, with speed (pwm) and direction
  for (int kth_motor = 0; kth_motor < NUM_ACTUATORS; kth_motor++) {
    if (abs(pos_diff[kth_motor]) >= POS_THRESHOLD) {
      // pwm_value[kth_motor] = pos_diff[kth_motor]);

      digitalWrite(ACTUATOR_DIR_PINS[kth_motor], direction[kth_motor]);
      analogWrite(ACTUATOR_PWM_PINS[kth_motor], constrain(abs(pos_diff[kth_motor]* PWM_GAIN), MIN_PWM, MAX_PWM));
    } else {
      analogWrite(ACTUATOR_PWM_PINS[kth_motor], 0);
    }
  }
}

void setup() {
  // For SerialUSB communication
  SerialUSB.begin(BAUD_RATE);
  // analogReadResolution(10); // 10 bit is default
  
  pinMode(BUTTON_LED, OUTPUT);

  // For the three position switch
  pinMode(LEFT_SWITCH_PIN, INPUT);
  pinMode(RIGTH_SWITCH_PIN, INPUT);

  pinMode(MODE_BUTTON, INPUT);
  attachInterrupt(digitalPinToInterrupt(MODE_BUTTON), change_state, RISING);



  // -------- ACTUATOR SETUP -------- //
  pinMode(ACTUATOR_POT_PIN_1, INPUT);
  pinMode(ACTUATOR_POT_PIN_2, INPUT);
  pinMode(ACTUATOR_POT_PIN_3, INPUT);

  pinMode(ACTUATOR_DIR_PIN_1, OUTPUT);
  pinMode(ACTUATOR_DIR_PIN_2, OUTPUT);
  pinMode(ACTUATOR_DIR_PIN_3, OUTPUT);

  pinMode(ACTUATOR_PWM_PIN_1, OUTPUT);
  pinMode(ACTUATOR_PWM_PIN_2, OUTPUT);
  pinMode(ACTUATOR_PWM_PIN_3, OUTPUT);

  pinMode(ENABLE_ACTUATORS, OUTPUT);
  // Is inverted, so /Enable
  digitalWrite(ENABLE_ACTUATORS, LOW);

  // If no claibration is done, we assume that calibration is already done previously
  calibrate(CALIBRATE);

  delay(2000);
}

void on_off_lights(platform_state state, uint32_t time) {
  switch (current_state) {
    case IDLE:
      if (time - previousMillis >= blink_interval) {
        previousMillis = time;

        ledState = !ledState;

        digitalWrite(BUTTON_LED, ledState);
      }
      break;

    case RUNNING:
      digitalWrite(BUTTON_LED, HIGH);
      break;

    case STOPPING:
      digitalWrite(BUTTON_LED, LOW);
      break;
  }
}

int read_analogue_avg(int pin) {
  uint32_t reads = 0;
  for (int i = 0; i < ACTUATOR_MEASUREMENTS; i++) {
    reads += analogRead(pin);
  }
  return (int)(reads / ACTUATOR_MEASUREMENTS);
}

bool read_digital_avg(int pin) {
  int reads = 0;
  for (int i = 0; i < BUTTON_MEASUREMENTS; i++) {
    reads += digitalRead(pin);
  }
  return (bool)(reads / BUTTON_MEASUREMENTS);
}

void loop() {
  int motor_count_reset = 0;

  current_state = next_state; 

  // 10 - SLOW mode      - Switch is at the left most position
  // 01 - FAST mode      - Switch is at the right most position
  // 00 - Moderate mode  - Switch is at the middle position
  if (read_digital_avg(LEFT_SWITCH_PIN) == 1) next_period = SLOW;
  else if (read_digital_avg(RIGTH_SWITCH_PIN) == 1) next_period = FAST;
  else next_period = MODERATE;

  if (next_period != period) {
    period = next_period;
    if (current_state == RUNNING) current_state = RESET;
  }
  
  time_read = millis();
  on_off_lights(current_state, time_read);

  switch (current_state) {
    case IDLE:
      break;

    case SET_TIME:
      next_state = RUNNING;
      last_time = time_read;
      break;

    case RUNNING:
      current_time = time_read - last_time;

      move_to_pos(current_time);

      break;

    case RESET:
      next_state = RESET;

      move_to_pos(current_time, true);

      for (int kth_motor = 0; kth_motor < NUM_ACTUATORS; kth_motor++) {
        if (current_pos[kth_motor] - (VERTICAL_SHIFT - AMPLUTIDE) < POS_THRESHOLD) {
          motor_count_reset++;
        }
      }

      if (motor_count_reset == NUM_ACTUATORS) {
        next_state = SET_TIME;
      }

      break;

    case STOPPING:
      move_to_pos(current_time, true);

      for (int kth_motor = 0; kth_motor < NUM_ACTUATORS; kth_motor++) {
        if (current_pos[kth_motor] < POS_THRESHOLD) {
          motor_count_reset++;
        }
      }

      if (motor_count_reset == NUM_ACTUATORS) {
        next_state = IDLE;
      }

      break;
  }
}