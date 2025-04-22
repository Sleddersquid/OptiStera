#include "variables.h"
#include <math.h>

// How to control the actuators (motors), file organising and calibrate function are taken from,
// https://github.com/progressiveautomations/Stewart-Platform/blob/master/arduino/platform/platform.ino

// To do list
// 1. Beutify ti -- DONE
// 2. Shorten the list of global variables, which is all for now -- No need
// 3. Add enable and diabling of H-bridge -- No
// 4. Is it a concern that the lights light up when reset is done? -- No
// 5. IMPORTANT: Add led light on the button -- DONE
// 6. Soft reset/stop -- DONE, but works onyl for heave, not tested with bias of l_k(t)

uint32_t blink_interval = 1000, previousMillis, current_time;  // in ms

bool ledState = 0;

// Enum for different speeds for the actuator
enum ActuatorSpeed {
  SLOW = 11000,     // in ms
  MODERATE = 8000,  // in ms
  FAST = 5000       // in ms
};

// Enum for modes
enum PlatformState {
  IDLE,     // The platform goes nothing
  SET_TIME, // Stamp the time read, so that l_k(t) start at 0 every change between IDLE -> RUNNING
  RUNNING,  // The platform is moving and has heave motion
  RESET,    // For when the platform changes speeds (change in P in l_k(t)), reset to A - C and start again with current speed  
  STOPPING, // For stopping the platform completely, and goes back to IDLE state
  EMERGENCY // For when the emergency button has been pressed, and the H-bridge loses power. 
};

// When the program starts, the actuators go to fully retracted position and then goes to IDLE
volatile PlatformState current_state = EMERGENCY; // Will be populated at iteration of loop
PlatformState next_state = EMERGENCY;

// The period sets the speed for the actuators
uint16_t period = MODERATE;
uint16_t next_period = period;

uint32_t time_read, last_time; // the time to be read in ms, and the last read time at first iteration of IDLE -> RUNNING
uint32_t time_set[NUM_ACTUATORS];

uint16_t desired_pos[NUM_ACTUATORS];  // Ranges from 0..1023
uint16_t current_pos[NUM_ACTUATORS];  // Ranges from 0..1023
int16_t pos_diff[NUM_ACTUATORS];      // Ranges from -1023..1023
uint8_t pwm_value[NUM_ACTUATORS];     // Ranges from 0..255
bool direction[NUM_ACTUATORS];        // 0 = RETRACT, 1 = EXTEND

// Variables used only for calibration
int16_t end_readings[NUM_ACTUATORS];   // Readings from fully extended position
int16_t zero_readings[NUM_ACTUATORS];  // Readings from fully retracted position
// If no calibration is done, assume it has been done previously. IMPORTANT
bool calibration_valid = true;  // Will be set to false if calibration waas not done correctly

const uint8_t debounce_interval = 200;
volatile uint32_t last_interrupt_time = 0;
uint32_t interrupt_time = 0;

void calibrate(bool run_calibration) {
  if (!run_calibration) {
    return;
  }

  // Extend all actuators
  for (int kth_actuator = 0; kth_actuator < NUM_ACTUATORS; ++kth_actuator) {
    digitalWrite(ACTUATOR_DIR_PINS[kth_actuator], EXTEND);
    analogWrite(ACTUATOR_PWM_PINS[kth_actuator], MAX_PWM);
  }
  delay(RESET_DELAY);

  // Stop the extension, get average anlogue readings
  for (int kth_actuator = 0; kth_actuator < NUM_ACTUATORS; ++kth_actuator) {
    analogWrite(ACTUATOR_PWM_PINS[kth_actuator], 0);
    end_readings[kth_actuator] = read_analogue_avg(ACTUATOR_POT_PINS[kth_actuator]);

    // Check if the kth_actuators are powered (reading is valid)
    calibration_valid = (abs(end_readings[kth_actuator] - END_POS[kth_actuator]) < OFF_THRESHOLD);
    if (!calibration_valid) { break; }
  }
  // Retract all actuators
  for (int kth_actuator = 0; kth_actuator < NUM_ACTUATORS; ++kth_actuator) {
    digitalWrite(ACTUATOR_DIR_PINS[kth_actuator], RETRACT);
    analogWrite(ACTUATOR_PWM_PINS[kth_actuator], MAX_PWM);
  }
  delay(RESET_DELAY);

  // Stop the retraction, get average analog readings
  for (int kth_actuator = 0; kth_actuator < NUM_ACTUATORS; ++kth_actuator) {
    analogWrite(ACTUATOR_PWM_PINS[kth_actuator], 0);
    zero_readings[kth_actuator] = read_analogue_avg(ACTUATOR_POT_PINS[kth_actuator]);

    // Check if the actuators are powered (reading is valid)
    calibration_valid = (abs(zero_readings[kth_actuator] - ZERO_POS[kth_actuator]) < OFF_THRESHOLD);
    if (!calibration_valid) { break; }
  }

  // IF CALIBRATION IS TO BE DONE MANUALY, uncomment this
  SerialUSB.print("calibration_valid: ");
  SerialUSB.println(calibration_valid);

  for (int kth_actuator = 0; kth_actuator < NUM_ACTUATORS; ++kth_actuator) {
    SerialUSB.print(" Actuator ");
    SerialUSB.print(kth_actuator);
    SerialUSB.print(": ");
    SerialUSB.print(zero_readings[kth_actuator]);
    SerialUSB.print(", ");
    SerialUSB.print(end_readings[kth_actuator]);
  }

  SerialUSB.println("");

  // Set the new calibration values if found to be valid
  if (calibration_valid) {
    for (int kth_actuator = 0; kth_actuator < NUM_ACTUATORS; ++kth_actuator) {
      END_POS[kth_actuator] = end_readings[kth_actuator];
      ZERO_POS[kth_actuator] = zero_readings[kth_actuator];
    }
  }
}

void change_state() {
  // If button is not HIGH in all measurements assume noise and don't trigger state change, else proceed
  if (many_read_digital(MODE_BUTTON_PIN) != BUTTON_MEASUREMENTS) return;

  interrupt_time = millis();
  // If interrupts comes faster than debounce_interval, assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time > debounce_interval) {
    // Don't need to jump to RUNNING and IDLE, because
    // SET_TIME -> RUNNING (stay until interrupt) and STOPPING -> IDLE (stay until interrupt)
    if (current_state == IDLE) next_state = SET_TIME;
    else if (current_state == RUNNING || current_state == RESET) next_state = STOPPING;
    last_interrupt_time = interrupt_time;
  }
}

// l_k(t) with k = 1, 2, 3 and t in ms
float positionFunction(int t, float bias) { 
  return -AMPLUTIDE * cos(((2 * PI * t) / period) + bias) + VERTICAL_SHIFT;
}

void move_to_pos() {
  // Get the current positon of all actuators and set dirrection based of the
  for (int kth_actuator = 0; kth_actuator < NUM_ACTUATORS; kth_actuator++) {
    current_pos[kth_actuator] = map(read_analogue_avg(ACTUATOR_POT_PINS[kth_actuator]), ZERO_POS[kth_actuator], END_POS[kth_actuator], MIN_POS, MAX_POS);

    pos_diff[kth_actuator] = desired_pos[kth_actuator] - current_pos[kth_actuator];

    // Se nærmere på
    if (current_pos[kth_actuator] < desired_pos[kth_actuator]) {
      direction[kth_actuator] = EXTEND;
    } else {
      direction[kth_actuator] = RETRACT;
    }
  }

  // move the actuators, with speed (pwm) and direction
  for (int kth_actuator = 0; kth_actuator < NUM_ACTUATORS; kth_actuator++) {
    if (abs(pos_diff[kth_actuator]) >= POS_THRESHOLD) {
      // pwm_value[kth_actuator] = pos_diff[kth_actuator]);

      digitalWrite(ACTUATOR_DIR_PINS[kth_actuator], direction[kth_actuator]);
      analogWrite(ACTUATOR_PWM_PINS[kth_actuator], constrain(abs(pos_diff[kth_actuator]* PWM_GAIN), MIN_PWM, MAX_PWM));
    } else {
      analogWrite(ACTUATOR_PWM_PINS[kth_actuator], 0);
    }
  }
}

void on_off_lights(PlatformState state, uint32_t time) {
  switch (current_state) {
    case IDLE:
      if (time - previousMillis >= blink_interval) {
        previousMillis = time;

        ledState = !ledState;

        digitalWrite(BUTTON_LED_PIN, ledState);
      }
      break;

    case RUNNING:
      digitalWrite(BUTTON_LED_PIN, HIGH);
      break;

    case STOPPING:
      digitalWrite(BUTTON_LED_PIN, LOW);
      break;

    case EMERGENCY:
      digitalWrite(BUTTON_LED_PIN, LOW);
      break;
  }
}

int read_analogue_avg(int pin) {
  // Will never be negative and biggest possible integer is (2^10 - 1)* 255 = 260'865
  // Therfore, minimum bits needed are ln((2^10 - 1)* 255)/ln(2) = celi(17.99) = 18 bits
  uint32_t reads = 0; 
  for (int i = 0; i < ACTUATOR_MEASUREMENTS; i++) {
    reads += analogRead(pin);
  }
  return (reads / ACTUATOR_MEASUREMENTS);
}

int many_read_digital(int pin) {
  int reads = 0;
  for (int i = 0; i < BUTTON_MEASUREMENTS; i++) {
    reads += digitalRead(pin);
  }
  return reads;
}

void setup() {
  // For SerialUSB communication
  SerialUSB.begin(BAUD_RATE);
  // analogReadResolution(10); // 10 bit is default
  
  pinMode(BUTTON_LED_PIN, OUTPUT);

  // For the three position switch
  pinMode(LEFT_SWITCH_PIN, INPUT);
  pinMode(RIGTH_SWITCH_PIN, INPUT);

  pinMode(MODE_BUTTON_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(MODE_BUTTON_PIN), change_state, RISING);

  pinMode(EMERGENCY_STOP_PIN, INPUT);

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

  delay(1000);
}


void loop() {
  int actuator_count_reset = 0;

  // 10 - SLOW mode      - Switch is at the left most position
  // 01 - FAST mode      - Switch is at the right most position
  // 00 - Moderate mode  - Switch is at the middle position
  if (many_read_digital(LEFT_SWITCH_PIN) == BUTTON_MEASUREMENTS) next_period = SLOW;
  else if (many_read_digital(RIGTH_SWITCH_PIN) == BUTTON_MEASUREMENTS) next_period = FAST;
  else next_period = MODERATE;

  if (next_period != period) {
    period = next_period;
    if (current_state == RUNNING) next_state = RESET;
  }

  if(many_read_digital(EMERGENCY_STOP_PIN) == BUTTON_MEASUREMENTS) {
    next_state = EMERGENCY;
  }

  // Change state
  current_state = next_state;
  
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

      for (int kth_actuator = 0; kth_actuator < NUM_ACTUATORS; kth_actuator++) {
        time_set[kth_actuator] = current_time % period;
        desired_pos[kth_actuator] = positionFunction(time_set[kth_actuator], ACTUATOR_BIAS[kth_actuator]);
      }
    
      // Moves actuators to desired position, calculating direction and speed for actuators
      move_to_pos();

      break;

    case RESET:
      for (int kth_actuator = 0; kth_actuator < NUM_ACTUATORS; kth_actuator++) {
        desired_pos[kth_actuator] = desired_pos[kth_actuator] - GRADTIENT; // 0.01 // In binary, float point arithmatic
      }

      // Moves actuators to desired position, calculating direction and speed for actuators
      move_to_pos();

      for (int kth_actuator = 0; kth_actuator < NUM_ACTUATORS; kth_actuator++) {
        // Check if the desired position has been reached
        if (current_pos[kth_actuator] - (VERTICAL_SHIFT - AMPLUTIDE) < POS_THRESHOLD) {
          actuator_count_reset++;
        }
      }

      // If all three actuators have reached their desired position, go to next state
      if (actuator_count_reset == NUM_ACTUATORS) {
        next_state = SET_TIME;
      }

      break;

    case STOPPING:
      for (int kth_actuator = 0; kth_actuator < NUM_ACTUATORS; kth_actuator++) {
        desired_pos[kth_actuator] = desired_pos[kth_actuator] - GRADTIENT; // 0.01 // In binary, float point arithmatic
      }

      // Moves actuators to desired position, calculating direction and speed for actuators
      move_to_pos();

      // Check if the desired position has been reached
      for (int kth_actuator = 0; kth_actuator < NUM_ACTUATORS; kth_actuator++) {
        if (current_pos[kth_actuator] <= POS_THRESHOLD) {
          actuator_count_reset++;
        }
      }

      // If all three actuators have reached their desired position, go to next state
      if (actuator_count_reset == NUM_ACTUATORS) {
        next_state = IDLE;
      }

      break;

    case EMERGENCY:
      // Read and set desired pos for all actuators, so there is no big moment of inertia when going over to stopping state
      for (int kth_actuator = 0; kth_actuator < NUM_ACTUATORS; kth_actuator++) {
        current_pos[kth_actuator] = map(read_analogue_avg(ACTUATOR_POT_PINS[kth_actuator]), ZERO_POS[kth_actuator], END_POS[kth_actuator], MIN_POS, MAX_POS);

        desired_pos[kth_actuator] = current_pos[kth_actuator];
      }

      // If emergency is LOW, then emergency has been realeased and move to stopping state
      if (many_read_digital(EMERGENCY_STOP_PIN) == 0) {
        next_state = STOPPING;
      }

      break;

    default:
      // Do nothing.
      break;
  }

  // String csv_format = String(current_time) + ";" + String(current_pos[0]) + ";" + String(current_pos[1]) + ";" + String(current_pos[2]) + ";" + String(desired_pos[0]) + ";" + String(desired_pos[1]) + ";" + String(desired_pos[2]);
  // SerialUSB.println(csv_format);
}