#include <LiquidCrystal_I2C.h>
#include <Wire.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

enum PlatformState {
  IDLE,     // The platform is waiting for a signal to start
  SET_TIME, // Stamp the time read, so that l_k(t) start at 0 every change between IDLE -> RUNNING
  RUNNING,  // The platform is moving and has heave motion
  REPOSITION,    // For when the platform changes speeds (change in P in l_k(t)), reset to l_k(t) and start again with current speed  
  RETURN_HOME, // For stopping the platform completely, and goes back to IDLE state 
  EMERGENCY // For when the emergency button has been pressed, and the H-bridge loses power. 
};
//making the symbols
byte Heart[] = { //making a heart symbol
  B00000,
  B01010,
  B11111,
  B11111,
  B01110,
  B00100,
  B00000,
  B00000
};
byte Bell[] = { //bell symbol for emergency 
  B00100,
  B01110,
  B01010,
  B01010,
  B01010,
  B11111,
  B00000,
  B00100
};
byte circle[] = { //two arrows going in circle, symbolising repositioning
  B01111,
  B01001,
  B11101,
  B01001,
  B10010,
  B10111,
  B10010,
  B11110,
};
byte hourGlass[] = { //For idle, a waiting signal
  B00000,
  B00000,
  B11111,
  B01110,
  B00100,
  B01110,
  B11111,
  B00000,
};
byte time_set[] = { // the time/ clock signal 
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

volatile PlatformState current_state = EMERGENCY; // Will be populated at iteration of loop
PlatformState next_state = EMERGENCY;

uint32_t interrupt_time = 0;
volatile uint32_t last_interrupt_time = 0;
const uint8_t debounce_interval = 200;

void change_state() {
  interrupt_time = millis();
  // If interrupts comes faster than debounce_interval, assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time > debounce_interval) {
    // Don't need to jump to RUNNING and IDLE, because
    // SET_TIME -> RUNNING (stay until interrupt) and RETURN_HOME -> IDLE (stay until interrupt)
    if (current_state == IDLE) next_state = SET_TIME;
    else if (current_state == SET_TIME) next_state = RUNNING;
    else if (current_state == RUNNING) next_state = REPOSITION;
    else if (current_state == REPOSITION) next_state = RETURN_HOME;
    else if (current_state == RETURN_HOME) next_state = EMERGENCY;
    else if (current_state == EMERGENCY) next_state = IDLE;
    last_interrupt_time = interrupt_time;
  }
}

void setup() {
  // specifies the bits in use
  lcd.init();
  //lcd.begin(16, 2);
  //Serial.begin(9600);
  lcd.backlight();
  //The syboles gets created
  lcd.createChar(0, Heart);
  lcd.createChar(1, Bell);
  lcd.createChar(2, circle);
  lcd.createChar(3, hourGlass);
  lcd.createChar(4, time_set);
  lcd.createChar(5, runningstate);
  
  //lcd.createChar(5, running);
  // pinMode(5, INPUT);
  attachInterrupt(digitalPinToInterrupt(2), change_state, RISING);
}

void loop() {
  // main code here, to run repeatedly:
  current_state = next_state;
  lcd.setCursor(0, 0);
  lcd.print("State:");

  Serial.println(current_state);
  //spesifies where the current space should get printed 
  lcd.setCursor(7,0);
  lcd.print(current_state);

  lcd.setCursor(0,1);
  //If else loop to print the current state
  if (current_state == 0){ 
    //Prints IDLE and the symbol if the current state is 0
    lcd.print("IDLE      ");
    lcd.setCursor(14, 1);
    lcd.write(byte(3));
    
  }
  else if(current_state == 1){
    lcd.print("SET_TIME     ");
    lcd.setCursor(14, 1);
    lcd.write(byte(4));
  }
  else if(current_state == 2){
    lcd.print("RUNNING        ");
    lcd.setCursor(14, 1);
    lcd.write(byte(5));
    
  }
  else if(current_state == 3){
    lcd.print("REPOSITION   ");
    lcd.setCursor(14, 1);
    lcd.write(byte(2));
  }
  else if(current_state == 4){
    lcd.print("RETURN_HOME  ");
    lcd.setCursor(14, 1);
    lcd.write(byte(0));
  }
  else if(current_state == 5){
    lcd.print("EMERGENCY   ");
    lcd.setCursor(14, 1);
    lcd.write(byte(1));
  }
}
  
