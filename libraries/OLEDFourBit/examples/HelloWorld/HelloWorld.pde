// include the library code:
#include <OLEDFourBit.h>

// initialize the library with the numbers of the Arduino pins

const int RSpin = 12;
const int RWpin = 10;
const int ENpin = 11;
const int D4pin = 5;
const int D5pin = 4;
const int D6pin = 3;
const int D7pin = 2;

OLEDFourBit lcd(RSpin, RWpin, ENpin, D4pin, D5pin, D6pin, D7pin);

void setup() {
  // set up the LCD's number of columns and rows: 
  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.print("hello, world!");
}

void loop() {
  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  lcd.setCursor(0, 1);
  // print the number of seconds since reset:
  lcd.print(millis()/1000);
}

