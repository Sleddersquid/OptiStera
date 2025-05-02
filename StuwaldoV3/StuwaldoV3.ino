#include "variables.h"
#include <math.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>


// How to control the actuators (motors), file organising and calibrate function are taken from,
// https://github.com/progressiveautomations/Stewart-Platform/blob/master/arduino/platform/platform.ino

// To do list
// 1. Beutify ti -- DONE
// 2. Shorten the list of global variables, which is all for now -- No need
// 3. Add enable and diabling of H-bridge -- No
// 4. Is it a concern that the lights light up when reset is done? -- No
// 5. IMPORTANT: Add led light on the button -- DONE
// 6. Soft reset/stop -- DONE, but works onyl for heave, not tested with bias of l_k(t)
LiquidCrystal_I2C lcd(LCD_ADDRESS, 16, 2);

//making the symbols for the print
byte heart[] = {  //making a heart symbol
  B00000,
  B01010,
  B11111,
  B11111,
  B01110,
  B00100,
  B00000,
  B00000
};
byte bell[] = {  //bell symbol for emergency
  B00100,
  B01110,
  B01010,
  B01010,
  B01010,
  B11111,
  B00000,
  B00100
};
byte circle[] = {
  //two arrows going in circle, symbolising repositioning
  B01111,
  B01001,
  B11101,
  B01001,
  B10010,
  B10111,
  B10010,
  B11110,
};
byte hour_glass[] = {
  //For idle, a waiting signal
  B00000,
  B00000,
  B11111,
  B01110,
  B00100,
  B01110,
  B11111,
  B00000,
};
byte time_set[] = {
  // the time/ clock signal
  B00000,
  B00000,
  B00000,
  B00100,
  B00100,
  B00111,
  B00000,
  B00000,
};
byte runningstate[] = {
  B00000,
  B00000,
  B00001,
  B00010,
  B10100,
  B01000,
  B00000,
  B00000,
};

uint32_t blink_interval = 1000, previousMillis;  // in ms
bool ledState = 0;

// Enum for different speeds for the actuator
enum ActuatorSpeed {  // in ms
  SLOW = 11000,
  MODERATE = 8000,
  FAST = 5000
};

// Enum for modes
enum PlatformState {
  IDLE,         // The platform is waiting for a signal to start
  SET_TIME,     // Stamp the time read, so that l_k(t) start at 0 every change between IDLE -> RUNNING
  RUNNING,      // The platform is moving and has heave motion
  REPOSITION,   // For when the platform changes speeds (change in P in l_k(t)), reset to l_k(t) and start again with current speed
  RETURN_HOME,  // For stopping the platform completely, and goes back to IDLE state
  EMERGENCY     // For when the emergency button has been pressed, and the H-bridge loses power.
};

// When the program starts, the actuators go to fully retracted position and then goes to IDLE
volatile PlatformState current_state = EMERGENCY;  // Will be populated at iteration of loop
volatile PlatformState next_state = EMERGENCY;

int actuator_count_reset = 0;

// The period sets the speed for the actuators
uint16_t period = MODERATE;
uint16_t next_period = period;
bool enable_switch_change = false;

uint32_t time_read, last_timestamp;  // the time to be read in ms, and the last read time at first iteration of IDLE -> RUNNING

int32_t desired_pos[NUM_ACTUATORS] = { 0, 0, 0 };  // Ranges from 0..1023
int32_t current_pos[NUM_ACTUATORS] = { 0, 0, 0 };  // Ranges from 0..1023
int32_t pos_diff[NUM_ACTUATORS] = { 0, 0, 0 };     // Ranges from -1023..1023
int32_t pwm_value[NUM_ACTUATORS] = { 0, 0, 0 };
bool direction[NUM_ACTUATORS] = { 0, 0, 0 };         // 0 = RETRACT, 1 = EXTEND

// Variables used only for calibration
int16_t end_readings[NUM_ACTUATORS];   // Readings from fully extended position
int16_t zero_readings[NUM_ACTUATORS];  // Readings from fully retracted position
bool calibration_valid = true;         // Will be set to false if calibration was not done correctly, and program stops

uint32_t interrupt_time = 0;
volatile uint32_t last_interrupt_time = 0;
const uint8_t debounce_interval = 200;

void calibrate() {
  // Extend all actuators
  for (int kth_actuator = 0; kth_actuator < NUM_ACTUATORS; ++kth_actuator) {
    digitalWrite(ACTUATOR_DIR_PINS[kth_actuator], EXTEND);
    analogWrite(ACTUATOR_PWM_PINS[kth_actuator], MAX_PWM);
  }
  delay(RESET_DELAY);

  // Stop the extension, get average anlogue readings
  for (int kth_actuator = 0; kth_actuator < NUM_ACTUATORS; ++kth_actuator) {
    analogWrite(ACTUATOR_PWM_PINS[kth_actuator], 0);
    end_readings[kth_actuator] = readAnalogueAvg(ACTUATOR_POT_PINS[kth_actuator]);

    // Check if the kth_actuators are powered (reading is valid)
    calibration_valid = (abs(end_readings[kth_actuator] - END_POS[kth_actuator]) < OFF_THRESHOLD);
    //if (!calibration_valid) { break; }
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
    zero_readings[kth_actuator] = readAnalogueAvg(ACTUATOR_POT_PINS[kth_actuator]);

    // Check if the actuators are powered (reading is valid)
    calibration_valid = (abs(zero_readings[kth_actuator] - ZERO_POS[kth_actuator]) < OFF_THRESHOLD);
    //if (!calibration_valid) { break; }
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
  // If button is not HIGH in all measurements assume noise and don't trigger state change
  if (manyReadDigital(STATE_BUTTON_PIN) != BUTTON_MEASUREMENTS) return;

  interrupt_time = millis();
  // If interrupts comes faster than debounce_interval, assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time > debounce_interval) {
    // Don't need to jump to RUNNING and IDLE, because
    // SET_TIME -> RUNNING (stay until interrupt) and RETURN_HOME -> IDLE (stay until interrupt)
    if (current_state == IDLE) next_state = SET_TIME;
    else if (current_state == RUNNING || current_state == REPOSITION) next_state = RETURN_HOME;
    last_interrupt_time = interrupt_time;
  }
}

// l_k(t) with k = 1, 2, 3 and t in ms
int32_t positionFunction(int t, float bias) {
  if (t == 0) return VERTICAL_SHIFT - AMPLUTIDE;
  return -AMPLUTIDE * cos(((2 * PI * t) / period) + bias) + VERTICAL_SHIFT;
}

void moveToPos() {
  // Get the current positon of all actuators and set dirrection based of the
  for (int kth_actuator = 0; kth_actuator < NUM_ACTUATORS; kth_actuator++) {
    current_pos[kth_actuator] = readActuators(ACTUATOR_POT_PINS[kth_actuator], ZERO_POS[kth_actuator], END_POS[kth_actuator]);

    pos_diff[kth_actuator] = desired_pos[kth_actuator] - current_pos[kth_actuator];

    // Calculate speed and direction
    if (abs(pos_diff[kth_actuator]) >= POS_THRESHOLD) {
      pwm_value[kth_actuator] = constrain(abs(pos_diff[kth_actuator] * PWM_GAIN), MIN_PWM, MAX_PWM);
      if (current_pos[kth_actuator] < desired_pos[kth_actuator]) {
        direction[kth_actuator] = EXTEND;
      } else {
        direction[kth_actuator] = RETRACT;
      }
    }
  }

  // move the actuators, with speed (pwm) and direction
  for (int kth_actuator = 0; kth_actuator < NUM_ACTUATORS; kth_actuator++) {
    if (abs(pos_diff[kth_actuator]) >= POS_THRESHOLD) {

      digitalWrite(ACTUATOR_DIR_PINS[kth_actuator], direction[kth_actuator]);
      analogWrite(ACTUATOR_PWM_PINS[kth_actuator], pwm_value[kth_actuator]);
    } else {
      analogWrite(ACTUATOR_PWM_PINS[kth_actuator], 0);
    }
  }

  //   for (int kth_actuator = 0; kth_actuator < NUM_ACTUATORS; kth_actuator++) {
  //     SerialUSB.print(" Actuator ");
  //     SerialUSB.print(kth_actuator + 1);
  //     SerialUSB.print(": ");
  //     SerialUSB.print(current_pos[kth_actuator]);
  //     SerialUSB.print(", ");
  //     SerialUSB.print(desired_pos[kth_actuator]);
  //     SerialUSB.print(", ");
  //     SerialUSB.print(direction[kth_actuator]);
  //   }
  // SerialUSB.println("");
}

void stateButtonLight(PlatformState state, uint32_t time) {
  switch (current_state) {
    case IDLE:
      if (time - previousMillis >= blink_interval) {
        previousMillis = time;

        digitalWrite(BUTTON_LED_PIN, ledState);

        ledState = !ledState;
      }
      break;

    case RUNNING:
      digitalWrite(BUTTON_LED_PIN, HIGH);
      break;

    case RETURN_HOME:
      digitalWrite(BUTTON_LED_PIN, LOW);
      ledState = HIGH;
      break;

    case EMERGENCY:
      digitalWrite(BUTTON_LED_PIN, LOW);
      break;
  }
}

void lcdDisplayState(PlatformState state) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("State:");

  //spesifies where the current space should get printed
  lcd.setCursor(7, 0);
  lcd.print(state);

  lcd.setCursor(0, 1);
  //If else loop to print the current state
  if (state == 0) {
    //Prints IDLE and the symbol if the current state is 0
    lcd.print("IDLE       ");
    lcd.setCursor(14, 1);
    lcd.write(byte(3));
  } else if (state == 1) {
    lcd.print("SET TIME     ");
    lcd.setCursor(14, 1);
    lcd.write(byte(4));
  } else if (state == 2) {
    lcd.print("RUNNING        ");
    lcd.setCursor(14, 1);
    lcd.write(byte(5));
  } else if (state == 3) {
    lcd.print("REPOSITION   ");
    lcd.setCursor(14, 1);
    lcd.write(byte(2));
  } else if (state == 4) {
    lcd.print("RETURN HOME  ");
    lcd.setCursor(14, 1);
    lcd.write(byte(0));
  } else if (state == 5) {
    lcd.print("EMERGENCY   ");
    lcd.setCursor(14, 1);
    lcd.write(byte(1));
  }
}

long readAnalogueAvg(int pin) {
  // Will never be negative and biggest possible integer is (2^10 - 1)* 255 = 260'865
  // Therfore, minimum bits needed are ln((2^10 - 1)* 255)/ln(2) = celi(17.99) = 18 bits
  long reads = 0;
  for (int i = 0; i < ACTUATOR_MEASUREMENTS; i++) {
    reads += analogRead(pin);
  }
  return (long)(reads / ACTUATOR_MEASUREMENTS);
}

int manyReadDigital(int pin) {
  int reads = 0;
  for (int i = 0; i < BUTTON_MEASUREMENTS; i++) {
    reads += digitalRead(pin);
  }
  return reads;
}

int16_t readActuators(int actuator_pin, int16_t zero_pos, int16_t end_pos) {
  long read = readAnalogueAvg(actuator_pin);
  return constrain(map(read, zero_pos, end_pos, MIN_POS, MAX_POS), MIN_POS, MAX_POS);
}

void setup() {
  // For SerialUSB communication
  SerialUSB.begin(BAUD_RATE);
  // analogReadResolution(10); // 10 bit is default

  // -------- LCD DISPLAY SETUP -------- //
  lcd.init();
  //lcd.begin(16, 2);
  lcd.backlight();
  //The syboles gets created
  lcd.createChar(0, heart);
  lcd.createChar(1, bell);
  lcd.createChar(2, circle);
  lcd.createChar(3, hour_glass);
  lcd.createChar(4, time_set);
  lcd.createChar(5, runningstate);

  pinMode(BUTTON_LED_PIN, OUTPUT);

  // For the three position switch
  pinMode(LEFT_SWITCH_PIN, INPUT);
  pinMode(RIGTH_SWITCH_PIN, INPUT);

  pinMode(STATE_BUTTON_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(STATE_BUTTON_PIN), change_state, RISING);

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
  if (CALIBRATE) {
    calibrate();
  }

  //delay(1000);
}

void loop() {
  // If the calibration runs and fails (e.g one of the actuators are not connected), the program will stop
  // while (!calibration_valid);

  // 10 - SLOW mode      - Switch is at the left most position
  // 01 - FAST mode      - Switch is at the right most position
  // 00 - Moderate mode  - Switch is at the middle position
  if(enable_switch_change) {
    if (manyReadDigital(LEFT_SWITCH_PIN) == BUTTON_MEASUREMENTS) next_period = SLOW;
    else if (manyReadDigital(RIGTH_SWITCH_PIN) == BUTTON_MEASUREMENTS) next_period = FAST;
    else next_period = MODERATE;
  }

  if (period != next_period) {
    period = next_period;
    if (current_state == RUNNING) next_state = REPOSITION;
  }

  if (manyReadDigital(EMERGENCY_STOP_PIN) == BUTTON_MEASUREMENTS) {
    next_state = EMERGENCY;
  }

  if (ENABLE_LCD_DISPLAY && (current_state != next_state)) {
    lcdDisplayState(next_state);
  }

  // Change state
  current_state = next_state;

  // SerialUSB.print("Current State: ");
  // SerialUSB.println(current_state);

  SerialUSB.print("BRUH: ");
  SerialUSB.println(enable_switch_change);

  time_read = millis();

  if (ENABLE_BUTTON_LED) {
    stateButtonLight(current_state, time_read);
  }

  switch (current_state) {
    case IDLE:
      break;

    case SET_TIME:
      last_timestamp = time_read;
      next_state = RUNNING;
      break;

    case RUNNING:
      time_read = time_read - last_timestamp;

      for (int kth_actuator = 0; kth_actuator < NUM_ACTUATORS; kth_actuator++) {
        desired_pos[kth_actuator] = positionFunction(time_read, ACTUATOR_BIAS[kth_actuator]);
      }

      // Moves actuators to desired position, calculating direction and speed for actuators
      moveToPos();

      if(time_read > 1000) {
        enable_switch_change = true;
      } else {
        enable_switch_change = false;
      }

      break;

    case REPOSITION:
      actuator_count_reset = 0;

      for (int kth_actuator = 0; kth_actuator < NUM_ACTUATORS; kth_actuator++) {
        if (desired_pos[kth_actuator] > positionFunction(0, ACTUATOR_BIAS[kth_actuator])) {
          desired_pos[kth_actuator] = constrain(desired_pos[kth_actuator] - GRADTIENT, MIN_POS, MAX_POS);
        }
      }

      for (int kth_actuator = 0; kth_actuator < NUM_ACTUATORS; kth_actuator++) {
        // Check if the desired position has been reached
        if (current_pos[kth_actuator] <= positionFunction(0, ACTUATOR_BIAS[kth_actuator]) + POS_THRESHOLD) {
          actuator_count_reset++;
        }
      }

      // Moves actuators to desired position, calculating direction and speed for actuators
      moveToPos();

      if (actuator_count_reset == NUM_ACTUATORS) {
        // If all three actuators have reached their desired position, go to next state
        next_state = SET_TIME;
      }

      break;

    case RETURN_HOME:
      actuator_count_reset = 0;
      enable_switch_change = 0;

      for (int kth_actuator = 0; kth_actuator < NUM_ACTUATORS; kth_actuator++) {
        if (desired_pos[kth_actuator] > 0) {
          desired_pos[kth_actuator] = constrain(desired_pos[kth_actuator] - GRADTIENT, MIN_POS, MAX_POS);
        }
      }
      // Check if the desired position has been reached
      for (int kth_actuator = 0; kth_actuator < NUM_ACTUATORS; kth_actuator++) {
        if (current_pos[kth_actuator] <= POS_THRESHOLD) {
          actuator_count_reset++;
        }
      }

      // Moves actuators to desired position, calculating direction and speed for actuators
      moveToPos();

      if (actuator_count_reset == NUM_ACTUATORS) {
        // If all three actuators have reached their desired position, go to next state
        next_state = IDLE;
      }

      break;

    case EMERGENCY:
      // Read and set desired pos for all actuators, so there is no big moment of inertia when going over to return home state
      for (int kth_actuator = 0; kth_actuator < NUM_ACTUATORS; kth_actuator++) {
        current_pos[kth_actuator] = readActuators(ACTUATOR_POT_PINS[kth_actuator], ZERO_POS[kth_actuator], END_POS[kth_actuator]);

        desired_pos[kth_actuator] = current_pos[kth_actuator];
      }

      // If emergency is LOW, then emergency has been realeased and move to return home state
      if (manyReadDigital(EMERGENCY_STOP_PIN) == 0) {
        next_state = RETURN_HOME;
      }

      break;
    default:
      SerialUSB.println("The fuck");
      break;
  }

  // String csv_format = String(current_time) + ";" + String(current_pos[0]) + ";" + String(current_pos[1]) + ";" + String(current_pos[2]) + ";" + String(desired_pos[0]) + ";" + String(desired_pos[1]) + ";" + String(desired_pos[2]);
  // SerialUSB.println(csv_format);
}
