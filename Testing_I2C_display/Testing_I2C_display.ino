// Following code by Electromania is based on...
// https://github.com/build2master/arduino-lcd-scroll-long-text/blob/master/arduino-lcd-scroll-long-text.ino  for scrolling text concept
// https://github.com/MarkusLange/Arduino-Due-RTC-Library  for RTC library
// https://github.com/marcoschwartz/LiquidCrystal_I2C   for Arduino due library for LCD
// http://playground.arduino.cc/Main/I2cScanner  for Arduino I2C address scanner sketch
// Thanks to these people for their previous work....

// From https://www.theelectromania.com/2016/01/clock-and-calendar-using-arduino-due.html

// #include <RTCDue.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h> // make sure to change return value to 1 , in LiquidCrystal_I2C.cpp file of this library to avoid problem of only first character being displayed

LiquidCrystal_I2C lcd(0x27, 16, 2); // put here correct address of lcd...  if not known, use Arduino I2CScanner given in above link

char *daynames[] = {"Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun"};
int hh, mm, ss, dow, dd, mon, yyyy;

String scrollingmessage = "                 Using Arduino Due internal RTC, no need of external RTC chip. Have fun.... :)        ";
int ii = 0;
int strLength;
String toShow;

void setup()
{

  // Serial.begin(9600); just in case if you want to get date and time in serial port
  //  RTCDue.begin();
  //  rtc_clock.set_clock(__DATE__, __TIME__);
  //---------------------------------------Optional part begin--------------------------------------------
  //  This optional part can be removed later to show only time and date, I added this section here just as fun and to experiment with scrolling text
  strLength = scrollingmessage.length();
  lcd.init(); // initialize the lcd
  lcd.backlight();
  // lcd.blink();
  lcd.home();
  lcd.setCursor(0, 0); // column , row 1

} // end of for loop

//--------------------------Optional part ends here---------------------------------------------------------

void loop()
{
  for (int timeCounter = 0; timeCounter < 90; timeCounter++)
  { // this loop is to wait until whole scrolling message is displayed

    lcd.home();
    lcd.setCursor(0, 1); // column , row 1

    toShow = scrollingmessage.substring(ii, ii + 16); // Get 16 characters so that we can display on the LCD

    lcd.print(toShow); // print the number of seconds since reset:

    ii = ii + 1; // move 1 step left

    if (ii > (strLength - 16))
    { // We have to reset ii after there is less text displayed.
      ii = 0;
    }

    delay(210); // set this as per the required speed of scrolling

    lcd.clear(); // clear the lcd
  }
}