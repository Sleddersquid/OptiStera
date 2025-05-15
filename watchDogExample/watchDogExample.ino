
// From https://forum.arduino.cc/t/due-watchdog/337497/3

int counter = 0;

// Defines the time for the watchdog in ms
int watchdogTime = 1000;

// this function has to be present, otherwise watchdog won't work
void watchdogSetup(void) 
{
// Do here
}

void setup() 
{
  // Enable watchdog.
  watchdogEnable(watchdogTime);
  SerialUSB.begin(9600);
}


void loop() 
{
  // Reset watchdog
  watchdogReset();
  SerialUSB.println(counter);
  delay(500);
  if(counter >= 10)
  { 
    while(true)
    {
      SerialUSB.println("loop");
      delay(100);
    }
  }
  counter++;
}