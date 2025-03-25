#include "variables.h"
#include <math.h>

#define PI 3.1415926535897932384626433832795

#define _DELAY 50

int PWM_VALUE = 0;

#define AMPLUTIDE 5000

int pot_value_read;

#define POT_PIN_V A6

bool direction = 1;

uint32_t time_read = 0;

int current_inst;
int previous_diff;
int previous_inst;
int total_diff;
int prop_diff;

int pos_diff;

int desired_pos;

int input;

int dir;

int measured;

float corr;
float p_corr;
float i_corr;
float d_corr;
int pwm;

// FOr the y function
// float t_zero = 0;
// float t_final = 8;
// float y_zero = ;
// float y_final;


// PID control parameters
float Kp = 1.2;
float Ki = 0.01;
float Kd = 0.05;


// int computePID(float setpoint, float measured) {

//   pos = measured - 


//   float integral = 0;
//   float prev_error = 0;
//   float currentPWM = startPWM;


//   float error = setpoint - measured;
//   integral += error;
//   float derivative = error - prev_error;
//   prev_error = error;
//   float output = Kp * error + Ki * integral + Kd * derivative;
//   if (output > 255) output = 255;
//   if (output < 0) output = 0;
//   return output;
// }
void readSerial() {
    // Parse ints from serial
    
    input = SerialUSB.parseInt();

    // Check that inputs are valid
    
    
        if (input < MIN_POS || input > MAX_POS)
        {
            return;
        }


    // Set the input as the desired position
    
        total_diff = 0;  // reset integral feedback given a new target
        desired_pos = input;
        input = desired_pos;
    
}


int move() {
  // Compute the error value

  pos_diff = desired_pos - measured;

  // Compute error-dependent PID variables
  if (abs(pos_diff) <= POS_THRESHOLD)
  {
      prop_diff = 0;
      total_diff = 0;
  }
  else
  {
      prop_diff = pos_diff;
      total_diff += pos_diff;
  }
  if (previous_diff != pos_diff)
  {
      previous_inst = current_inst;
      current_inst = 1;
  }

  // Compute PID gain values
  p_corr = P_COEFF * prop_diff;
  i_corr = I_COEFF * total_diff;
  d_corr = D_COEFF * ((float) pos_diff - previous_diff) / (current_inst + previous_inst);

  // Compute the correction value and set the direction and PWM for the actuator
  corr = p_corr + i_corr + d_corr;
  dir = (corr > 0) ? 0 : 1;  // direction based on the sign of the error
  pwm = constrain(abs(corr), MIN_PWM, MAX_PWM);  // bound the correction by the PWM limits

  // SerialUSB.print("PWM: " );
  // SerialUSB.println(pwm);

  // Move the actuator with the new direction and PWM values
  digitalWrite(DIR_PIN, dir);
  analogWrite(PWM_PIN, pwm);

  // Increment sample-dependent PID variables for the next sampling time
  current_inst += 1;
  previous_diff = pos_diff;

  return previous_diff;
}


float y(float t) {
  return -200*cos(2*PI*t/AMPLUTIDE) + 824;
}


void move_to_pos(int _desired_pos) {

  measured = map(readMany(255, POT_PIN), ZERO_POS, END_POS, MIN_POS, MAX_POS);

  if (measured < _desired_pos) {
    direction = false;
  } else {
      direction = true;
  }

  if(abs(_desired_pos - measured) >= POS_THRESHOLD) {
    int PWM_VALUE = constrain(abs(_desired_pos - measured)*5, MIN_PWM, MAX_PWM);

    SerialUSB.print("PWM value :");
    SerialUSB.println(PWM_VALUE);

    digitalWrite(DIR_PIN, direction);
    analogWrite(PWM_PIN, PWM_VALUE);
  } else {
    analogWrite(PWM_PIN, 0);
  }
}



void setup() {

  while (!SerialUSB);

  // put your setup code here, to run once:
  // Serial.begin(BAUD_RATE);
  SerialUSB.begin(BAUD_RATE);

  pinMode(POT_PIN, INPUT);

  pinMode(DIR_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);

  pinMode(ENABLE_MOTOR, OUTPUT);
  // Inverted
  digitalWrite(ENABLE_MOTOR, LOW);
}

int readMany(int measurements, int pin) {
  int reads = 0;
  for(int i = 0; i < measurements; i++) {
    reads += analogRead(pin);
  }
  return reads / measurements; 

}

void loop() {
  // put your main code here, to run repeatedly:
  // SerialUSB.println("Hello World");

  // Start with extension
  digitalWrite(DIR_PIN, 1);
  analogWrite(PWM_PIN, MAX_PWM);

  delay(RESET_DELAY);

  int extracted_pot_value = readMany(1000, POT_PIN);

  SerialUSB.print("Extracted valeus: ");
  SerialUSB.println(extracted_pot_value);

  // Retract all actuators
  digitalWrite(DIR_PIN, 0);
  analogWrite(PWM_PIN, MAX_PWM);

  delay(RESET_DELAY);

  int rteacted_pot_value = readMany(1000, POT_PIN);

  SerialUSB.print("Retracetd valeus: ");
  SerialUSB.println(rteacted_pot_value);

  // time_read = millis() % AMPLUTIDE;

  // desired_pos = y(time_read);

  // desired_pos = readMany(255, POT_PIN_V);

  // move_to_pos(desired_pos);


  // if (SerialUSB.available() > INPUT_TRIGGER){
  //       readSerial();
  // }

  // diff = move();

  // SerialUSB.print("Desired pos: ");
  // SerialUSB.println(desired_pos);

  // SerialUSB.print("Actual pos: ");
  // SerialUSB.println(measured);

  // SerialUSB.print("Input pos: ");
  // SerialUSB.println(input);

  // int i = 0;

  // int AnalogRead1 = readMany(255, POT_PIN_1);
  // int AnalogRead2 = readMany(255, POT_PIN_2);
  // int AnalogRead3 = readMany(255, POT_PIN_3);
  // int AnalogRead4 = readMany(255, POT_PIN_4);
  // int AnalogRead5 = readMany(255, POT_PIN_5);
  // int AnalogRead6 = readMany(255, POT_PIN_6);


  // Serial.print("A1: ");
  // Serial.println(AnalogRead1);

  // Serial.print("A2: ");
  // Serial.println(AnalogRead2);

  // Serial.print("A3: ");
  // Serial.println(AnalogRead3);

  // Serial.print("A4: ");
  // Serial.println(AnalogRead4);

  // Serial.print("A5: ");
  // Serial.println(AnalogRead5);

  // Serial.print("A6: ");
  // Serial.println(AnalogRead6);

  // int AnalogRead = readMany(255, POT_PIN);
  // Serial.print("Read values from pot pin: ");
  // Serial.println(AnalogRead);
  // // SerialUSB.println(AnalogRead);

  // // analogWrite(PWM_PIN, 0);

  // digitalWrite(DIR_PIN, direction);
  // for (int i = 0; i < 255; i++) {
  //   analogWrite(PWM_PIN, constrain(i, MIN_PWM, MAX_PWM));
  //   delay(10);
  // }
  // //   delay(10);
  // //   i++;
  // Serial.print("Dir: ");
  // Serial.println(direction);

  // if (direction) {
  //   direction = false;
  // } else {
  //   direction = true;
  // }

  // delay(2000);

  while (true);

}