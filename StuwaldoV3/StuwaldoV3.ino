/**
 * @file StuwaldoV3/StuwaldoV3.ino
 * @author OptiStera
 * @brief Main control logic for platform states and actuator control.
 *
 * - Controls the motion of the platform with three actuators, and the use of a function \f$ l_k(t) \f$ to control the position of the actuators.
 * - It includes functions for calibration, state transitions, actuator movement, and updates to a LCD display.
 * - The platform operates in various states such as #IDLE, #RUNNING, #REPOSITION and #RETURN_HOME.
 * - Calibration ensures the actuators are correctly aligned and functional.
 * - Emergency stop functionality is implemented to handle emergency situations.
 * - How to control the actuators (motors), file organising and calibrate function are taken from \cite PA2 .
 *
 * @todo Better functionality for invalid actuator calibration.
 * @todo More detailed logging for debugging purposes.
 * @todo Implement a better soft reset/stop functionality for \f$ B_k\neq 0 \f$.
 * @todo Find original source of why actuators become unresponsive when the three position switch is turned too many times within a small timeframe. 
 * @todo Instead of #IDLE \f$ \to \f$ #RUNNING, there should be a middle state to home the actuators. #RETURN_HOME could be as a middle state, where the actuators align themselves with \f$ l_k(0) \f$ and then start (#RUNNING state).
 * @todo Caluclate the gradient based on a speed = distance/time, so that the actautors all reach the bottom at the same tim. 
 * 
 * @test Test actuators with \f$ B_k\neq 0 \f$ and how they behave in #REPOSITION and #RETURN_HOME state.
 *
 * @bug Changing the speed of the actuators too many times with the three position switch within small timeframe causes the actuators to become unresponsive. Has been fixed by disabling the switch for 1 second when in #RUNNING state.
 * @bug #REPOSITION and #RETURN_HOME have only been tested with \f$ B_k=0 \f$. If a bias should be induced, test first with the actuators not mounted onto the platform. This is due to the actuators that are going inwards will continue inwards, but the actuators that are going outwards must gradually stop, turn and then go inwards. This creates a bigger difference between the length of the actuators than initialy defined by \f$ B_k \f$ . 
 * @bug If the switch changes position when #IDLE \f$ \to \f$ #RUNNING and #desired_pos is still less than \f$ l_k(0) \f$ within 1 second, there will be a small jerk of motion, with the current configuration of \f$ l_k(t) \f$.
*/

#include "variables.h"
#include <math.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <Scheduler.h>

#if ENABLE_LCD_DISPLAY
  /// Initialize the LCD display with address, columns and rows
  LiquidCrystal_I2C lcd(LCD_ADDRESS, 16, 2);
#endif

/// Creating the symbols for the display
/// Heart symbol for #RETURN_HOME state
byte heart[]= 
{
    B00000,
    B01010,
    B11111,
    B11111,
    B01110,
    B00100,
    B00000,
    B00000
};

/// Bell symbol for #EMERGENCY state
byte bell[]= 
{ 
    B00100,
    B01110,
    B01010,
    B01010,
    B01010,
    B11111,
    B00000,
    B00100
};

/// Two arrows going in circle, for #REPOSITION state
byte circle[]= 
{
    B01111,
    B01001,
    B11101,
    B01001,
    B10010,
    B10111,
    B10010,
    B11110,
};

/// A waiting symbol for #IDLE state
byte hour_glass[]= 
{
    B00000,
    B00000,
    B11111,
    B01110,
    B00100,
    B01110,
    B11111,
    B00000,
};

/// The time/clock symbol for #SET_TIME state
byte time_set[]= 
{
    B00000,
    B00000,
    B00000,
    B00100,
    B00100,
    B00111,
    B00000,
    B00000,
};

/// Checkmark for #RUNNING state
byte runningstate[]= 
{
    B00000,
    B00000,
    B00001,
    B00010,
    B10100,
    B01000,
    B00000,
    B00000,
};


bool first_emergency = true;

/// Interval between button LED blinks in IDLE state.
const uint16_t blink_interval = 1000;
/// Last time the button LED was turned on/off.
uint32_t previousMillis;
/// Current state of the button LED.
bool ledState = 0;

/// Enum for different speeds for the actuator. In milliseconds.
enum ActuatorSpeed
{
  /// \f$ (10_2) \f$ - Switch is in the left most position.
  SLOW = 11000,
  /// \f$ (00_2) \f$ - Switch is in the middle position.
  MODERATE = 8000,
  /// \f$ (01_2) \f$ - Switch is in the right most position.
  FAST = 5000
};

/// Enum to clarify the direction in which actuators move.
enum ActuatorDirection
{
  /// The actuator is moving inwards itself.
  RETRACT = 0,
  /// The actuator is moving outwards itself.
  EXTEND = 1
};

/// Enum for the diffent states for the actuators.
enum PlatformState
{
  /// The platform is waiting for the user to start the platform.
  IDLE,        
  /// Stamp the time read, so that \f$ l_k(t) \f$ start at \f$ t=0\f$ when changing between IDLE \f$ \to \f$ RUNNING.
  SET_TIME,    
  /// The platform is moving, degrees of freedom defined by \f$ B_k\f$.
  RUNNING,
  /// For when the platform changes speeds (change in \f$ P\f$ for \f$ l_k(t)\f$, reset to \f$ l_k(0)\f$ and start again with current speed).
  REPOSITION,  
  /// For stopping the platform completely (move actuators to neutral position), and go back to IDLE state.
  RETURN_HOME, 
  /// For when the emergency button has been pressed, and the H-bridge loses power. The speed for the actuators are set to 0.
  EMERGENCY    
};

/// When the program starts assume emergency. If not emergency, the actuators go to fully neutral position and is then ready to start (in IDLE state).
/// Will be populated at first iteration of loop() by #next_state.
volatile PlatformState current_state = RETURN_HOME;
/// Controls the next state. Is controlled by the three position switch, button and emergency.
volatile PlatformState next_state = EMERGENCY;
/// Locks the state change for the button, when the LCD display is updating.

/// Helper variable to count the actuators that have reached their desired position.
/// If all actuators have reached their desired position, change state.
uint8_t actuator_count_reset = 0;

/// The period sets the speed for the actuators.
uint16_t period = MODERATE;
uint16_t next_period = period;
/// The switch is enabled after 1 second in #RUNNING state.
bool enable_switch_change = false;

/// The time to be read in ms, input to positionFunction(int t, float bias).
uint32_t time_read = 0; 
/// The timestamp at change between states #IDLE \f$ \to \f$ #RUNNING.
uint32_t last_timestamp = 0;

/// Desired position for the actuators, ranges from 0..1023.
int32_t desired_pos[NUM_ACTUATORS] = {0, 0, 0};
/// Current position of the actuators, ranges from 0..1023.
int32_t current_pos[NUM_ACTUATORS] = {0, 0, 0};
/// Difference between desired and current position, ranges from -1023..1023.
int32_t pos_diff[NUM_ACTUATORS] = {0, 0, 0};
/// Speed of the actuators, ranges from 0..255.
int32_t pwm_value[NUM_ACTUATORS] = {0, 0, 0};
/// The direction for the actuators.
bool direction[NUM_ACTUATORS] = {0, 0, 0};

/// Readings from fully retracted position. Only used when calibrating.
int16_t zero_readings[NUM_ACTUATORS];
/// Readings from fully extended position. Only used when calibrating.
int16_t end_readings[NUM_ACTUATORS]; 
/// Will be set to false if calibration was not done correctly, and program halts completely.
/// If #calibration_valid is false, the actuators are not powered and the program halts completely.
bool calibration_valid = true;


/// For when the state change button is pressed
uint32_t interrupt_time = 0;
/// The last time the state change button was pressed
volatile uint32_t last_interrupt_time = 0;
/// Debounce interval for the state change button
const uint8_t debounce_interval = 200;

/**
 * @brief Calibrates the actuators by extending and retracting them to get the zero and end positions.
 * @param manual If true, the calibration is to be done manually, i.e. the values from the actuators are to be read from the Serial Monitor.
*/
void calibrate(bool manual)
{
  // Extend all actuators
  for (int kth_actuator = 0; kth_actuator < NUM_ACTUATORS; ++kth_actuator)
  {
    digitalWrite(ACTUATOR_DIR_PINS[kth_actuator], EXTEND);
    analogWrite(ACTUATOR_PWM_PINS[kth_actuator], MAX_PWM);
  }
  delay(RESET_DELAY);

  // Stop the extension, get average anlogue readings
  for (int kth_actuator = 0; kth_actuator < NUM_ACTUATORS; ++kth_actuator)
  {
    analogWrite(ACTUATOR_PWM_PINS[kth_actuator], 0);
    end_readings[kth_actuator] = readAnalogueAvg(ACTUATOR_POT_PINS[kth_actuator]);

    // Check if the kth_actuators are powered (reading is valid)
    calibration_valid = (abs(end_readings[kth_actuator] - END_POS[kth_actuator]) < OFF_THRESHOLD);
    if (!manual && !calibration_valid)
    {
      break;
    }
  }
  // Retract all actuators
  for (int kth_actuator = 0; kth_actuator < NUM_ACTUATORS; ++kth_actuator)
  {
    digitalWrite(ACTUATOR_DIR_PINS[kth_actuator], RETRACT);
    analogWrite(ACTUATOR_PWM_PINS[kth_actuator], MAX_PWM);
  }
  delay(RESET_DELAY);

  // Stop the retraction, get average analog readings
  for (int kth_actuator = 0; kth_actuator < NUM_ACTUATORS; ++kth_actuator)
  {
    analogWrite(ACTUATOR_PWM_PINS[kth_actuator], 0);
    zero_readings[kth_actuator] = readAnalogueAvg(ACTUATOR_POT_PINS[kth_actuator]);

    // Check if the actuators are powered (reading is valid)
    calibration_valid = (abs(zero_readings[kth_actuator] - ZERO_POS[kth_actuator]) < OFF_THRESHOLD);
    if (!manual && !calibration_valid)
    {
      break;
    }
  }

  // Set the new calibration values if found to be valid
  if (!manual && calibration_valid)
  {
    for (int kth_actuator = 0; kth_actuator < NUM_ACTUATORS; ++kth_actuator)
    {
      END_POS[kth_actuator] = end_readings[kth_actuator];
      ZERO_POS[kth_actuator] = zero_readings[kth_actuator];
    }
  }
  else
  {
    SerialUSB.print("calibration_valid: ");
    SerialUSB.println(calibration_valid);

    for (int kth_actuator = 0; kth_actuator < NUM_ACTUATORS; ++kth_actuator)
    {
      SerialUSB.print(" Actuator ");
      SerialUSB.print(kth_actuator);
      SerialUSB.print(": ");
      SerialUSB.print(zero_readings[kth_actuator]);
      SerialUSB.print(", ");
      SerialUSB.print(end_readings[kth_actuator]);
    }
    SerialUSB.println("");
  }
}


/**
 * @brief Change the state of the platform based on the button press. 
 * If button is not HIGH in all #BUTTON_MEASUREMENTS assume noise and don't trigger state change.
 * Changes only between #IDLE \f$ \to \f$ #RUNNING, and #RUNNING \f$ \to \f$ #IDLE.
 * If interrupts comes faster than #debounce_interval, assume it's a bounce and ignore.
*/
void change_state()
{
  if (manyReadDigital(STATE_BUTTON_PIN) != BUTTON_MEASUREMENTS)
    return;

  interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > debounce_interval)
  {
    if (current_state == IDLE)
      next_state = SET_TIME;
    else if (current_state == RUNNING || current_state == REPOSITION)
      next_state = RETURN_HOME;
    last_interrupt_time = interrupt_time;
  }
}

/**
 * @brief Function to calculate the position of the actuators \f$ l_k(t)\f$ given \f$ t\f$ (in ms) and bias \f$ B_k \in \left[0, 2\pi \right\rangle \f$.
 * @param t Time in milliseconds.
 * @param bias Bias for the actuator \f$ B_k\f$.
 * @return Position of the actuator.
*/
int32_t positionFunction(int t, float bias)
{
  // if (t == 0)
  //   return VERTICAL_SHIFT - AMPLUTIDE;
  return -AMPLUTIDE * cos(((2 * PI * t) / period) + bias) + VERTICAL_SHIFT;
}


/**
 * @brief Move the actuators to the desired position.
 * It calculates the direction and speed of the actuators based on the difference between the desired and current position.
 * If the difference is less than \f$ \pm \f$ #POS_THRESHOLD, it stops the actuator.
 * Else it moves the actuator in the direction of the desired position.
*/
void moveToPos()
{
  // Get the current positon of all actuators and set dirrection based of the difference between desired and current position
  for (int kth_actuator = 0; kth_actuator < NUM_ACTUATORS; kth_actuator++)
  {
    current_pos[kth_actuator] = readActuators(ACTUATOR_POT_PINS[kth_actuator], ZERO_POS[kth_actuator], END_POS[kth_actuator]);

    pos_diff[kth_actuator] = desired_pos[kth_actuator] - current_pos[kth_actuator];

    // Calculate speed and direction
    if (abs(pos_diff[kth_actuator]) >= POS_THRESHOLD)
    {
      pwm_value[kth_actuator] = constrain(abs(pos_diff[kth_actuator] * PWM_GAIN), MIN_PWM, MAX_PWM);
      if (current_pos[kth_actuator] < desired_pos[kth_actuator])
      {
        direction[kth_actuator] = EXTEND;
      }
      else
      {
        direction[kth_actuator] = RETRACT;
      }
    }
  }

  // move the actuators, with speed (pwm) and direction
  for (int kth_actuator = 0; kth_actuator < NUM_ACTUATORS; kth_actuator++)
  {
    if (abs(pos_diff[kth_actuator]) >= POS_THRESHOLD)
    {

      digitalWrite(ACTUATOR_DIR_PINS[kth_actuator], direction[kth_actuator]);
      analogWrite(ACTUATOR_PWM_PINS[kth_actuator], pwm_value[kth_actuator]);
    }
    else
    {
      analogWrite(ACTUATOR_PWM_PINS[kth_actuator], 0);
    }
  }
}


/**
 * @brief Control the LED on the button based on the current state of the platform.
 * LED blinks in #IDLE state, is always on in #RUNNING state, and is off in #RETURN_HOME and #EMERGENCY state.
 * @param state Current state of the platform.
 * @param time Current time in milliseconds.
*/
void stateButtonLight(PlatformState state, uint32_t time)
{
  switch (current_state)
  {
  case IDLE:
    if (time - previousMillis >= blink_interval)
    {
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

  default:
    // Do nothing
    break;
  }
}


/**
 * @brief Display the current state of the platform on the LCD display.
 * Includes symbols for each state. The LCD display is only updated when the state changes.
 * Considered a critical region, interrupts are therefore disabled in this process. 
 * @param state Current state of the platform.
*/
void lcdDisplayState(PlatformState state)
{
  noInterrupts();
  /// Clearing the LCD display to avoid mix up with the different displays
  // lcd.clear();
  /// Prints the State: first, since this is stationary 
  lcd.setCursor(0, 0);
  lcd.print("State:");

  /// spesifies where the current state should get printed
  lcd.setCursor(7, 0);
  lcd.print(state);

  lcd.setCursor(0, 1);
  /// If else loop to print the current state
  if (state == 0)
  {
    // Prints IDLE and the symbol if the current state is 0
    lcd.print("IDLE           ");
    lcd.setCursor(14, 1);
    lcd.write(byte(3));
  }
  else if (state == 1)
  {
    lcd.print("SET TIME       ");
    lcd.setCursor(14, 1);
    lcd.write(byte(4));
  }
  else if (state == 2)
  {
    lcd.print("RUNNING        ");
    lcd.setCursor(14, 1);
    lcd.write(byte(5));
  }
  else if (state == 3)
  {
    lcd.print("REPOSITION     ");
    lcd.setCursor(14, 1);
    lcd.write(byte(2));
  }
  else if (state == 4)
  {
    lcd.print("RETURN HOME    ");
    lcd.setCursor(14, 1);
    lcd.write(byte(0));
  }
  else if (state == 5)
  {
    lcd.print("EMERGENCY      ");
    lcd.setCursor(14, 1);
    lcd.write(byte(1));
  }
  interrupts();
}


/**
 * @brief Reads the analogue value and returns the average value.
 * @param pin analogue pin to read from.
 * @return Average analogue reading.
*/
long readAnalogueAvg(int pin)
{
  long reads = 0;
  for (int i = 0; i < ACTUATOR_MEASUREMENTS; i++)
  {
    reads += analogRead(pin);
  }
  return (long)(reads / ACTUATOR_MEASUREMENTS);
}


/**
 * @brief Reads the digital value of a button given pin.
 * @param pin Digital pin to read measurements from.
 * @return Sum of measurements.
*/
int manyReadDigital(int pin)
{
  int reads = 0;
  for (int i = 0; i < BUTTON_MEASUREMENTS; i++)
  {
    reads += digitalRead(pin);
  }
  return reads;
}


/** 
 * @brief Reads the analogue value from the actuator potentiometer and maps it to a position between #MIN_POS and #MAX_POS. 
 * @param actuator_pin Pin number for the actuator potentiometer.
 * @param zero_pos Zero position from calibration.
 * @param end_pos End position from calibration.
 * @return Mapped position of the actuator.
*/
int32_t readActuators(int actuator_pin, int16_t zero_pos, int16_t end_pos)
{
  long read = readAnalogueAvg(actuator_pin);
  return constrain(map(read, zero_pos, end_pos, MIN_POS, MAX_POS), MIN_POS, MAX_POS);
}


/** @brief This function has to be present, otherwise watchdog won't work \cite watch_dog. See \cite watchDog_commit_need_void_func for more information.
*/
void watchdogSetup(void) 
{
  // Do nothing
}


/** 
 * @brief Intializes the pins for actuators and button, and sets up the LCD display.
 * SerialUSB for native port and Serial for programming port. H-bridge is enabled here and never powered down after this.
 * The LCD display is started, blacklight enabled and symbols created. Button, switch and LED pins are set up.
 * The actuators are calibrated if #CALIBRATE.
*/
void setup()
{
  // pinMode(ANALOGUE_DISCORVEY_PIN, OUTPUT);

  // -------- SERIAL SETUP -------- //
  // Serial.begin(BAUD_RATE);
  // analogReadResolution(10); // 10 bit is default

  // -------- LCD DISPLAY SETUP -------- //
  if(ENABLE_LCD_DISPLAY) {
    /// Start the LCD display
    lcd.init();
    /// Enable the backlight
    lcd.backlight();
    //create the symbols
    lcd.createChar(0, heart);
    lcd.createChar(1, bell);
    lcd.createChar(2, circle);
    lcd.createChar(3, hour_glass);
    lcd.createChar(4, time_set);
    lcd.createChar(5, runningstate);
  }
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

  pinMode(PIN_H_BRIDGE, OUTPUT);
  // Is inverted, so /Enable 
  digitalWrite(PIN_H_BRIDGE, LOW);

  // If no claibration is done, we assume that calibration is already done previously
  if (CALIBRATE)
  {
    calibrate(MANUAL_CALIBRATION);
  }

  // Scheduler.start();

  // This is Jose Manuel, the watchdog that oversees that everything goes as planned. Especially when it is time for the presentation.
  // If Jose Manuel is not happy with the frequency of the loop, he will reset the Arduino. 
  watchdogEnable(WATCHDOG_TIMEOUT);
}


/**
 * @brief Handels state transition and actuator movement.
 * Will stop if calibration is not valid.
 * Starts in #EMERGENCY state, and will go to #IDLE state when the actuators are at neutral position.
 * The LCD display will only update when the state changes.
 * The three position switch is enabled after 1 second in #RUNNING state.
*/
void loop()
{
  watchdogReset();
  // digitalWrite(ANALOGUE_DISCORVEY_PIN, HIGH);
  // If the calibration runs and fails (e.g one of the actuators are not connected) or if #MANUAL_CALIBRATION is true, the program will stop
  // while ((calibration_valid == false) || (MANUAL_CALIBRATION == true));

  if (enable_switch_change || (current_state != RUNNING))
  {
    if (manyReadDigital(LEFT_SWITCH_PIN) == BUTTON_MEASUREMENTS)
      next_period = SLOW;
    else if (manyReadDigital(RIGTH_SWITCH_PIN) == BUTTON_MEASUREMENTS)
      next_period = FAST;
    else
      next_period = MODERATE;
  }

  if (period != next_period)
  {
    period = next_period;
    if (current_state == RUNNING)
      next_state = REPOSITION;
  }

  if (manyReadDigital(EMERGENCY_STOP_PIN) == BUTTON_MEASUREMENTS)
  {
    next_state = EMERGENCY;
  }

  // Checks if state has changed, if so it will go to the next state
  if (ENABLE_LCD_DISPLAY && (current_state != next_state))
  {
    lcdDisplayState(next_state);
  }

  // Change state
  current_state = next_state;

  time_read = millis();

  if (ENABLE_BUTTON_LED)
  {
    stateButtonLight(current_state, time_read);
  }

  // digitalWrite(ANALOGUE_DISCORVEY_PIN, LOW);

  switch (current_state)
  {
  case IDLE:
    digitalWrite(PIN_H_BRIDGE, HIGH); // Disable H-bridge
    break;

  case SET_TIME:
    digitalWrite(PIN_H_BRIDGE, LOW); // Enable H-bridge
    last_timestamp = time_read;
    next_state = RUNNING;
    break;

  case RUNNING:
    time_read = time_read - last_timestamp;

    for (int kth_actuator = 0; kth_actuator < NUM_ACTUATORS; kth_actuator++)
    {
      desired_pos[kth_actuator] = positionFunction((time_read % period), ACTUATOR_BIAS[kth_actuator]);
    }

    // Moves actuators to desired position, calculating direction and speed for actuators
    moveToPos();

    if (time_read > 1000)
    {
      enable_switch_change = true;
    }
    else
    {
      enable_switch_change = false;
    }

    break;

  case REPOSITION:
    actuator_count_reset = 0;
    enable_switch_change = false;

    for (int kth_actuator = 0; kth_actuator < NUM_ACTUATORS; kth_actuator++)
    {
      if (desired_pos[kth_actuator] > positionFunction(0, ACTUATOR_BIAS[kth_actuator]))
      {
        desired_pos[kth_actuator] = constrain(desired_pos[kth_actuator] - GRADTIENT, MIN_POS, MAX_POS);
      }
    }

    for (int kth_actuator = 0; kth_actuator < NUM_ACTUATORS; kth_actuator++)
    {
      // Check if the desired position has been reached
      if (current_pos[kth_actuator] <= positionFunction(0, ACTUATOR_BIAS[kth_actuator]) + POS_THRESHOLD)
      {
        actuator_count_reset++;
      }
    }

    // Moves actuators to desired position, calculating direction and speed for actuators
    moveToPos();

    if (actuator_count_reset == NUM_ACTUATORS)
    {
      // If all three actuators have reached their desired position, go to next state
      next_state = SET_TIME;
    }

    break;

  case RETURN_HOME:
    digitalWrite(PIN_H_BRIDGE, LOW); // Enable H-bridge
    actuator_count_reset = 0;
    enable_switch_change = false;

    for (int kth_actuator = 0; kth_actuator < NUM_ACTUATORS; kth_actuator++)
    {
      if (desired_pos[kth_actuator] > 0)
      {
        desired_pos[kth_actuator] = constrain(desired_pos[kth_actuator] - GRADTIENT, MIN_POS, MAX_POS);
      }
    }
    // Check if the desired position has been reached
    for (int kth_actuator = 0; kth_actuator < NUM_ACTUATORS; kth_actuator++)
    {
      if (current_pos[kth_actuator] <= POS_THRESHOLD)
      {
        // SerialUSB.println(kth_actuator + 1);
        actuator_count_reset++;
      }
    }

    // Moves actuators to desired position, calculating direction and speed for actuators
    moveToPos();

    if (actuator_count_reset == NUM_ACTUATORS)
    {
      // If all three actuators have reached their desired position, go to next state
      next_state = IDLE;
    }

    break;

  case EMERGENCY:
    digitalWrite(PIN_H_BRIDGE, HIGH); // Disable H-bridge
    // Read and set desired pos for all actuators, so there is no big moment of inertia when going over to return home state
    for (int kth_actuator = 0; kth_actuator < NUM_ACTUATORS; kth_actuator++)
    {
      current_pos[kth_actuator] = readActuators(ACTUATOR_POT_PINS[kth_actuator], ZERO_POS[kth_actuator], END_POS[kth_actuator]);

      desired_pos[kth_actuator] = current_pos[kth_actuator];

      analogWrite(ACTUATOR_PWM_PINS[kth_actuator], 0);
    }

    // If emergency pin reads LOW, then emergency has been realeased and move to return home state
    if (manyReadDigital(EMERGENCY_STOP_PIN) == 0)
    {
      next_state = RETURN_HOME;
    }

    break;

  default:
    // Do nothing
    break;
  }
}
