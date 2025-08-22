/*
The Traveling GeoBug
Version 1.1. 20140228
Written by Mikael Carlsson
Copyright (c) 2013, 2014 Mikael Carlsson
All Rights Reserved.
Portions Copyright (c) 2008-2013 The Sundial Group
Although I have completely rewritten the code for my purpose I have included the copyright for The Sundial Group
as some code, primary the gps routine, are the same. But it needs to be the same, as it is based on Mikal Hart's TinyGPS

The Sundial group claims that "Travelling Geocache" is a TradeMark. It is NOT! In fact it was dismissed on April, 30th, 2012
As it says on https://tsdr.uspto.gov/#caseNumber=85181466
"TM5 Common Status Descriptor:
DEAD/APPLICATION/Refused/Dismissed or Invalidated
This trademark application was refused, dismissed, or invalidated by the Office and this application is no longer active."

This software is licensed under the terms of the Creative
Commons "Attribution Non-Commercial Share Alike" license, version
3.0, which grants the limited right to use or modify it NON-
COMMERCIALLY, so long as appropriate credit is given and
derivative works are licensed under the IDENTICAL TERMS.  For
license details see http://creativecommons.org/licenses/by-nc-sa/3.0/
 
This source code is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

External library used:
OLEDFourBit, PWMServo, TinyGPS and EEPROMAnything

Code for RFID reader is from http://playground.arduino.cc/Code/ID12

I have used the V2 of the Sundial shield. https://www.sundial.com/shop/version-2-0-shield-kit/
However - I made a few adjustments to it
1. I use a 4 * 20 OLED display from Newhaven. 
   So much more fun with more rows and columns. It is an NHD-0420DZW-AG5, green display.
   Fully readable in bright sunshine.
   To use that I have to use a modified library instead of LiquidCrystal:
   from http://code.google.com/p/uberfridge/source/browse/#svn/trunk/arduino/OLEDFourBit
2. I use an EM-411 GPS
(NOTE! The TX and RX pins are swapped on this module, I have taken care of that in the code.)
3. I have added a RFID reader for accessing the box as admin, it is a RDM630 module.
4. I have added a programming switch to be used to program new coordinates when goal is achieved, no more upload of code to the Arduino
5. I have added a radius switch to be able to toggle between 10 or 25 meter radius

Then I modified the shield:
I didn't solder cap2, nor the CTST resistor. OLED don't use contrast.

Pins used on the shield and my additions
0  - Not used
1  - Not used
2  - LED used in start button
3  - GPS RX (swapped because EM-411 has TX and RX swapped)
4  - GPS TX (Swap these if you are using an EM-406)
5  - OLED RS
6  - RFID RX (Don't solder CTST resistor or cap)
7  - OLED E
8  - OLED RW
9  - ServoControl
10 - RFID TX - do NOT connect this pin
11 - Piezo pin
12 - Pololu switch
13 - Not used
14 (A0) - OLED DB4
15 (A1) - OLED DB5
16 (A2) - OLED DB6
17 (A3) - OLED DB7
18 (A4) - Radius switch - used for setting radius to 10 or 25 meters
19 (A5) - Program switch - used when programming new coordinate

I have also used the F macro for some strings to save memory.

How to use:
First set your start coordinates by changing these variables in the code:
Destination.LATITUDE = 55.478412;
Destination.LONGITUDE = 13.136882;
Then change the RFID code for the admin tags.
I have used four RFID tags:
One for resetting the whole thing (RED tag), resets coordiantes, attempt counter
One for resetting the attempt counter (YELLOW tag)
One for open the box without changing anything (one of my BLUE tags)
One for swithing display the programmed coordinates (another BLUE tag)

The Program switch is to be used by the last user that found a place. 
The user then needs to go to a new place and press the start button
to progam new coordinates.

*/
// Uncomment to display debug code, useful when adding functions or check the GPS
//#define DEBUGCODE
// Uncomment to use LCD display instead
//#define USEOLED
// Include the necessary libraries
#include <SoftwareSerial.h> // Standard Arduino library
#include <EEPROM.h>         // Standard Arduino Library
#include <PWMServo.h>       // Include PWMServo
#include <TinyGPS.h>        // Include TinyGPS
#ifdef USEOLED
  #include <OLEDFourBit.h>    // Include a custom OLED library, see http://code.google.com/p/uberfridge/source/browse/#svn/trunk/arduino/OLEDFourBit
#else
  #include <LiquidCrystal.h>  // Include LCD library
#endif
#include "EEPROMAnything.h" // Include the library for writing floats to EEPROM: http://playground.arduino.cc/Code/EEPROMWriteAnything

// -----------------------------------------------------------------------
// User changeable variables and constants. 
// NOTE: Coordinates gets written into eeprom on startup if it is a new Arduino
static const float DEST_LATITUDE =  55.478412;                            // Change to your starting position for latitude
static const float DEST_LONGITUDE = 13.136882;                            // Change to your starting position for longitude
char RFID_ADMIN_TAG[]  = "000000000000";                                  // Tag serial number, for reset everything, RED
char RFID_RESET_TAG[]  = "111111111111";                                  // Tag serial number, for resetting attempt counter, YELLOW
char RFID_OPEN_TAG[]   = "222222222222";                                  // Tag serial number, for open the box without changing anything, BLUE
char RFID_COORD_TAG[]  = "333333333333";                                  // Tag serial number, to display the programmed coordinates, BLUE

static const byte SERVO_CLOSE = 140;                                      // Servo position (in degrees) for closed lock
static const byte SERVO_OPEN = 20;                                        // Servo position (in degrees) for open box
static const byte DEF_ATTEMPT_MAX = 50;                                   // Max attempts to try before the box locks the user out
// End of user changeable variables and constants
// ----------------------------------------------
// Other definitions, should not be changed
static const byte EEPROM_ATTEMPT_COUNTER = 1;                             // Position in eeprom memory to store the attempt counter.
static const byte EEPROM_FOUND_TARGET = 2;                                // Position in eeprom memory to store the found flag.
static const byte EEPROM_COORDINATES = 10;                                // Position in eeprom to store coordinates. Saved as 2 * float, eg. 8 bytes
static const byte OLED_Columns = 20;                                      // OLED Columns. I Use a 20 * 4 OLED, so much more useful <big grin>
static const byte OLED_Rows = 4;                                          // OLED Rows
static const int  RFIDSerialRate = 9600 ;                                 // RFID Reader Serial Port Speed
static const int  TIME_TO_WAIT_FOR_RFID = 5000;                           // A 5 second delay to detect RFID tags
// PIN Definitions
static const byte LED_BUTTON = 2;                                         // Pin for the LED in the start button
static const byte GPS_RxPin = 3, GPS_TxPin = 4;                           // Pins for GPS module
static const byte RFID_RxPin = 6, RFID_TxPin = 10;                        // Pins for RFID reader, don't connect pin 10, it is not used
static const byte OLED_RS = 5, OLED_E = 7, OLED_RW = 8;                   // OLED RS, E and RW
static const byte OLED_DB4=14, OLED_DB5=15, OLED_DB6=16, OLED_DB7=17;     // OLED DataPins
static const byte ServoControl = 9;                                       // Pin för the servo
static const byte Speaker = 11;                                           // Pin for the Speaker / Piezo
static const byte PololuSwitch = 12;                                      // Pin for pololu switch
static const byte RadiusSwitch = 18;                                      // Pin for radius switch
static const byte ProgramSwitch = 19;                                     // Pin for program switch

struct destination_t                                                      // I use EEPROMAnything to store coordinates in eeprom.
{
  float LATITUDE;                                                         // Latitude
  float LONGITUDE;                                                        // Longitude
} Destination;                                                            // Our destination in eeprom

boolean AdminTag = false;                                                 // Placeholder for RFID admin tag, true or false
boolean ProgramMode = false;                                              // Placeholder for programming new coordinates
boolean BeginnerMode = false;                                             // Placeholder for beginner mode
boolean FoundFlag = false;                                                // This flag is set if user found the target
byte AttemptCounter;                                                      // This holds the attempt counter
String RFIDTAG="";                                                        // Holds the RFID Code read from a tag
int Radius = 10;                                                          // Radius in meters

static SoftwareSerial RFIDReader(RFID_RxPin, RFID_TxPin);                 // Initialize serial port library for RFID-reader
static SoftwareSerial GPS(GPS_RxPin, GPS_TxPin);                          // Initialize serial port library for GPS module

// I use an EM-411 GPS
// NOTE!! The TX and RX are swapped on this module compared to an EM-406
static TinyGPS EM411gps;                                                  // Initialize GPS library.
#ifdef USEOLED
  static OLEDFourBit oled(OLED_RS, OLED_RW, OLED_E, OLED_DB4, OLED_DB5, OLED_DB6, OLED_DB7);   // Set up OLED library
#else
  static LiquidCrystal oled(OLED_RS, OLED_RW, OLED_E, OLED_DB4, OLED_DB5, OLED_DB6, OLED_DB7); // Set up LCD library 
#endif
static PWMServo servo;                                                                         // Set up servo
// End of definitions and initialization

void setup() // The Arduino setup() function
{
  servo.attach(ServoControl);                               // Attach the Servo Motor
  servo.write(SERVO_CLOSE);                                 // Make sure the lock is in closed position
  pinMode(ProgramSwitch, INPUT);                            // Set port to input mode to read the program switch.
  digitalWrite(ProgramSwitch, HIGH);                        // Activate internal pullup resitor, we are using a NO switch
  pinMode(RadiusSwitch, INPUT);                             // Set port to input mode to read the radius switch.
  digitalWrite(RadiusSwitch, HIGH);                         // Activate internal pullup resitor, we are using a NO switch
  pinMode(PololuSwitch, OUTPUT);                            // Make sure Pololu switch pin is OUTPUT
  digitalWrite(PololuSwitch, LOW);                          // and LOW
  pinMode(LED_BUTTON, OUTPUT);                              // Set pin as output
  digitalWrite(LED_BUTTON, HIGH);                           // and turn on LED in the start button
  oled.begin(OLED_Columns, OLED_Rows);                      // Establish communication with the OLED
  RFIDReader.begin(RFIDSerialRate);                         // Establish communication with the RFID-reader
  RFIDReader.listen();                                      // Call .listen function just to be sure to listen to RFID-reader
  DisplayInitText(0);                                       // Display bogus init text, we are waiting for the RFID Admin tag
  unsigned long start = millis();                           // Set up a timer
  while (millis() - start < TIME_TO_WAIT_FOR_RFID) {        // Loop for TIME_TO_WAIT_FOR_RFID seconds
    if (AdminTag = RFIDreadAdminTag()) break;               // If an admin tag is present then break out of while loop
  } // while
  if (AdminTag) ResetBox();                                 // If it is an reset tag, reset the box, otherwise just continue
  GPS.begin(4800);                                          // Initialize port at 4800 baud for the EM-411 module
  GPS.listen();                                             // Start listen to this port instead of the RFID-port
//  if (EEPROM.read(EEPROM_COORDINATES) == 0xFF ) {           // Check if this is a brand new Arduino (or reset by RFID tag), cell is 0xFF if it is.
//   Destination.LATITUDE = DEST_LATITUDE;                    // Store the initial coordinate to eeprom
//   Destination.LONGITUDE = DEST_LONGITUDE;                  // It will be overwritten by the new coordinates set by user
//   EEPROM_writeAnything(EEPROM_COORDINATES, Destination);   // Write it to eeprom
//  }
  EEPROM_readAnything(EEPROM_COORDINATES, Destination);     // Read the coordinates and stuff them into variables

  AttemptCounter = EEPROM.read(EEPROM_ATTEMPT_COUNTER);     // Read the attempt counter from EEPROM
  if (AttemptCounter == 0xFF) AttemptCounter = 0;           // Brand new Arduino? Then set Attempt_Counter to 0
  ++AttemptCounter;                                         // Increment it with each run

  FoundFlag = EEPROM.read(EEPROM_FOUND_TARGET);             // Read the FoundFlag from EEPROM
  if (FoundFlag == 0xFF) {                                  // If it is 0xFF (new Arduino) then set it to false (0)
    EEPROM.write(EEPROM_FOUND_TARGET, false);               // Set the FoundFlag to false in EEPROM
    FoundFlag = false;                                      // Set also FoundFlag to false
  }

  if (!digitalRead(ProgramSwitch)) {                        // Check if ProgramSwitch has been switched to Program Mode
    ProgramMode = true;                                     // Set ProgramMode flag to true
  }

  if (!digitalRead(RadiusSwitch)) {                         // Check the RadiusSwitch, is it in 10 or 25 m mode?
    Radius = 25;                                            // Set radius to 25 meter, for beginners
  }

  DisplayWelcomeText(5000);                                 // Display the welcome screen
  if (AttemptCounter >= DEF_ATTEMPT_MAX && !ProgramMode) {  // Have the user used up all attempts and not in programming mode?
    Text("  *** Oh NO!!! ***", " All attempts used", "You have failed!!", "", 5000);                   // Yes, he/she has eaten up all attempts
    Text("Please return box to", " your name here", "your phone here", "  your email here  ", 10000);  // Explain what to do now
    PowerOff();                                             // Shut down the box
  }

  if (ProgramMode) {                                        // Check if program mode, display text what to do
    Text("Program switch is ON", "Is this new target?", "If not, press start", "switch to shut down", 5000);
  } else if (FoundFlag) {                                   // Is the found flag true? Display what to do then shut down box
    Text("Target already found", "", " Please program new", "    coordinates.", 5000);
    Text("    Instructions", "are provided inside", "      the box.", "", 5000);
    servo.write(SERVO_OPEN);                                // Open the lock
    Text(" Unlocking box and", " shutting it down.", "", "", 5000);
    ShutDown();                                             // Turn off the Travelling GeoBug
  } else {                                                  // Normal mode, display attempts used
    String OLEDstring;                                      // Placeholder for concatenated string
    char charBuf[21];                                       // temp buffer for concatenated string
    OLEDstring += "      ";                                 // crude center of text
    OLEDstring += AttemptCounter;                           // Add the attempt counter
    OLEDstring += " of ";                                   // Add text " of "
    OLEDstring += DEF_ATTEMPT_MAX;                          // and add the max attempt number
    OLEDstring.toCharArray(charBuf, 21);                    // Copy it to a char array to be able to print it on the OLED
    Text("   This is attempt", charBuf, "", "", 5000);
  }

  Text(" (((( Seeking )))))", "(((((( Signal ))))))", " This might take up", "    to 45 seconds", 0);  // Display seeking text on the OLED
} // End of setup-routine

void loop()  // The Arduino loop() function
{
  if (GPS.available() && EM411gps.encode(GPS.read())) {               // Has a valid NMEA sentence been parsed?
    float lat, lon;                                                   // Local variables for latitude and longitude
    unsigned long fix_age;                                            // Variable for storing time received from GPS.

    EM411gps.f_get_position(&lat, &lon, &fix_age);                    // Have we established a location?
    if (fix_age != TinyGPS::GPS_INVALID_AGE) {                        // Is it a valid GPS fix?
      if (ProgramMode) {                                              // Have switch for programming been set?
        Text("Programming mode", "Resetting attempts", "", "", 2000);
        Text("New coordinates", "programmed to", "", "",0);           // No delay, delay is in next function
        Print_DDDMMMMM(lat, lon);                                     // Print the Degree Minutes on the OLED
        Destination.LATITUDE = lat;                                   // Store lat into eeprom variable
        Destination.LONGITUDE = lon;                                  // Store lon into eeprom variable
        EEPROM_writeAnything(EEPROM_COORDINATES, Destination);        // Store the coordinates to EEPROM
        servo.write(SERVO_OPEN);                                      // Open the lock so that the switch can be set to normal mode
        if (EEPROM.read(EEPROM_ATTEMPT_COUNTER)) {                    // Is AttemptCounter greater than 0?
          EEPROM.write(EEPROM_ATTEMPT_COUNTER, 0);                    // Then set it to 0
        }
        if (EEPROM.read(EEPROM_FOUND_TARGET)) {
          EEPROM.write(EEPROM_FOUND_TARGET, false);                   // Set the FoundFlag to false in EEPROM, but only if it is true
        }
        Text("Now turn programming", "switch to Off and", "close the lid.", "Unlocking........", 5000);
        ShutDown();                                                   // Turn off the Travelling GeoBug
      }
      else {                                                          // Programming mode not set. Let's play
        EEPROM.write(EEPROM_ATTEMPT_COUNTER, AttemptCounter);         // Save the new attempt in EEPROM, do it only if we get a fix
        // Calculate the distance to the destination
        float distance_meters = TinyGPS::distance_between(lat, lon, Destination.LATITUDE, Destination.LONGITUDE);
        oled.clear();                                                 // Clear display
        oled.setCursor(0, 0);                                         // Set cursor at row one
        oled.print(F("You are at position:"));
        Print_DDDMMMMM(lat, lon);                                     // Print the Degree Minutes on the OLED
        if (distance_meters <= Radius) {                              // Are we close enough?
          FoundThePlace();                                            // Yes, the user found the place
        } else {
          StillNotFound(lat, lon, distance_meters);                   // No, user has not found the target (yet)
        } // StillNotFound
        PowerOff();                                                   // Shutdown the box
      } // ProgramMode
    } // if fix_age
  } // if GPS.available
  if (millis() >= 300000)                                             // If the box have been in use for 5 minutes
    PowerOff();                                                       // Then shut it down, no GPS fix established
} // End of Arduino loop

/***************************************************************************************************** 
*  Function name: Text()                                                                             *
*  Purpose: Display text on the OLED display                                                         *
*  Parameters: row one, row two, row three, row four, delay in ms                                    *
*  First, second, third, forth parameters:                                                           *
*  Text string to be displayed at the row 1 to 4 on the OLED display                                 *
*  Fifth parameter: Delay in ms. Useful if you have multiple messages to be displayed                *
*  Returns: Nothing                                                                                  *
******************************************************************************************************/
void Text(const char *rowone, const char *rowtwo, const char *rowthree, const char *rowfour, unsigned int del)
{
  oled.clear();           // Clear display
  oled.setCursor(0, 0);   // Set cursor at row one
  oled.print(rowone);     // Print row one
  oled.setCursor(0, 1);   // Set cursor at row two
  oled.print(rowtwo);     // Print row two
  oled.setCursor(0, 2);   // Set cursor at row three
  oled.print(rowthree);   // Print row three
  oled.setCursor(0, 3);   // Set cursor at row four
  oled.print(rowfour);    // Print row four
  delay(del);             // Wait the number of ms before returning
}

/**************************************************************
* Function name: RFIDreadAdminTag()                           *
* Purpose: Check if an Admin tag is present during startup    *
* Parameters: None                                            *
* Returns: True, False or shutdown box, depending on the tag  *
* Calls ReadRFID                                              *
***************************************************************/
boolean RFIDreadAdminTag()
{
  ReadRFID(RFIDTAG);                                          // Read the tag number from the reader. Should return a 12 digit serial number
  if(RFIDTAG.equals(RFID_ADMIN_TAG)) return true;             // If tag is an Admin tag, go to Admin mode
  if(RFIDTAG.equals(RFID_RESET_TAG)) ResetAttemptCounter();   // If tag is an reset tag, reset attempt counter
  if(RFIDTAG.equals(RFID_OPEN_TAG)) UnlockBox();              // Is it the Open Box Tag, then unlock box
  if(RFIDTAG.equals(RFID_COORD_TAG)) {                        // Is it the Display Coordinates Tag?
    DisplayDestinationCoordinates();                          // Yes, then display the programmed coordinates
    }
  return false;                                               // All other tags returns false
}

/***********************************************************
* Function name: ResetAttemptCounter()                     *
* Purpose: Reset the attempt counter to 0 in EEPROM        *
* Parameters: None                                         *
* Returns: nothing                                         *
************************************************************/
void ResetAttemptCounter(void)
{
  EEPROM.write(EEPROM_ATTEMPT_COUNTER, 0); // Set AttemptCounter to 0
  oled.clear();                            // Clear display
  oled.setCursor(0, 0);                    // Set cursor at row one
  oled.print(F("  Resetting attempt"));    // Display whats done
  oled.setCursor(0, 1);                    // Set cursor at row two
  oled.print(F("    counter to 0"));       // Display whats done
  delay(2000);                             // Wait two seconds so that the servo is able to open
  ShutDown();                              // Turn off the Travelling GeoBug
}

/***********************************************************
* Function name: UnlockBox()                               *
* Purpose: Unlock the box without changing anything        *
* Parameters: None                                         *
* Returns: nothing                                         *
************************************************************/
void UnlockBox(void)
{
  servo.write(SERVO_OPEN);                 // Just open the box
  oled.clear();                            // Clear display
  oled.setCursor(0, 0);                    // Set cursor at row one
  oled.print(F("Unlocking box"));          // Display whats done
  delay(2000);                             // Wait two seconds so that the servo is able to open
  ShutDown();                              // Turn off the Travelling GeoBug
}

/***********************************************************
* Function name: DisplayDestinationCoordinates()           *
* Purpose: Display the programmed coordinates stored       *
*          in EEPROM                                       *
* Parameters: None                                         *
* Returns: nothing                                         *
************************************************************/
void DisplayDestinationCoordinates(void)
{
  EEPROM_readAnything(EEPROM_COORDINATES, Destination);         // Read the coordinates from EEPROM
  oled.clear();
  oled.setCursor(0, 0);
  oled.print(F("The programmed"));
  oled.setCursor(0, 1);
  oled.print(F("coordinates are:"));
  Print_DDDMMMMM(Destination.LATITUDE, Destination.LONGITUDE);  // Print the Degree Minutes on the OLED
  delay(5000);                                                  // Wait five seconds
  ShutDown();                                                   // Turn off the Travelling GeoBug
}

/***********************************************************
* Function name: ReadRFID()                                *
* Purpose: Read a tag from the RFID reader                 *
* Parameters: Pointer to the string to hold the tag ID     *
* Returns: Nothing                                         *
************************************************************/
void ReadRFID(String &ReadTagString)
{
// ToDo: Make it return a valid string without header or checksum.
// It works as it is now, but could be improved
  int bytesread = 0;
  int  val = 0; 
  char code[20];
  String TagCode="";

  if(RFIDReader.available() > 0) {          // If data available from reader 
    if((val = RFIDReader.read()) == 02) {   // Check for header, it is 0x02
      bytesread = 0;                        // Reset bytesread
      while(bytesread<12) {                 // Read 12 digit code from tag
        if( RFIDReader.available() > 0) { 
           val = RFIDReader.read();         // Read digit from tag
           if((val == 0x0D)||(val == 0x0A)||(val == 0x03)||(val == 0x02)) { // If header or stop bytes before the 10 digit reading 
              break;                        // Stop reading 
           } 
          if(val != 02) {                   // If it is anything else than a header
            code[bytesread] = val;          // add the digit
            bytesread++;                    // ready to read next digit
            code[bytesread] = '\0';         // add the NULL
          } // if val != 02
        } // if reader available
      } // while
      if(bytesread == 12) {                 // If 12 digit read is complete 
        for(int i=0; code[i]!='\0' ; i++)   // Copy the Chars to a String
        {
          TagCode += code[i];               // Add it to TagCode
        }
        ReadTagString = TagCode;            // Update the caller
        while(RFIDReader.available() > 0)   // Burn off any characters still in the buffer
        {
          RFIDReader.read();
        } 
      } // if bytesread
      bytesread = 0;                        // Reset bytesread
      TagCode="";                           // And empty TagCode
    } // if val == 02
  } // if RFIDReader available
}

/**********************************************************
* Function name: PowerOff()                               *
* Purpose: Shut down the system using the Pololu switch   *
*          Display text about it                          *
* Parameters: None                                        *
* Returns: Nothing                                        *
***********************************************************/
void PowerOff()
{
  oled.clear();                              // Clear screen
  oled.setCursor(0, 0);                      // First row
  oled.print(F("   Shutting down"));
  oled.setCursor(0, 1);                      // Second row
  oled.print(F("     GeoBug by"));
  oled.setCursor(0, 2);                      // Third row
  oled.print(F(" turning off power"));
  delay(2000);                               // Delay 2 seconds
  oled.clear();                              // Clear screen
  servo.write(SERVO_CLOSE);                  // Close the lock
  ShutDown();                                // Turn off the Travelling GeoBug
} 

/**********************************************************
* Function name: ShutDown()                               *
* Purpose: Shut down the system without text              *
* Parameters: None                                        *
* Returns: Nothing                                        *
***********************************************************/
void ShutDown()
{
  digitalWrite(PololuSwitch, HIGH);          // Turn off the Travelling GeoBug
} 

/**********************************************************
* Function name: PlayMusic()                              *
* Purpose: Play a tune when the destination is reached    *
* Parameters: None                                        *
* Returns: Nothing                                        *
***********************************************************/
void PlayMusic()
{
  tone(Speaker, 659, 200);
  delay(400);
  tone(Speaker, 659, 100);
  delay(200);
  tone(Speaker, 659, 100);
  delay(200);
  tone(Speaker, 1319, 200);
  delay(400);
  tone(Speaker, 1319, 200);
  delay(400);
  tone(Speaker, 1760, 750);
  delay(1500);
}

/**********************************************************
* Function name: ResetBox()                               *
* Purpose: Opens box, reset attempt counter               *
* Parameters: None                                        *
* Returns: Nothing                                        *
***********************************************************/
void ResetBox()
{
  Text("     Reset Box", "Clear Attempt Count", "Reset coordinates", "", 0);
  oled.setCursor(0, 3);                                     // Set cursor at row four
  oled.print(F("Unlocking for 20 sec"));
  servo.write(SERVO_OPEN);                                  // Open the lock
//  EEPROM.write(EEPROM_COORDINATES, 0xFF);                 // Reset the coordinates in EEPROM
   Destination.LATITUDE = DEST_LATITUDE;                    // Store the initial coordinate to eeprom
   Destination.LONGITUDE = DEST_LONGITUDE;                  // It will be overwritten by the new coordinates set by user
   EEPROM_writeAnything(EEPROM_COORDINATES, Destination);   // Write it to eeprom
  if (EEPROM.read(EEPROM_ATTEMPT_COUNTER)) EEPROM.write(EEPROM_ATTEMPT_COUNTER, 0); // Set AttemptCounter to 0, but only if it is not 0
  if (EEPROM.read(EEPROM_FOUND_TARGET)) EEPROM.write(EEPROM_FOUND_TARGET, false);   // Set the FoundFlag to false in EEPROM, but only if it is true
  delay(20000);                                             // Wait 20 seconds
  oled.setCursor(0, 3);                                     // Set cursor at row four
  oled.print(F("Locking the box     "));
  servo.write(SERVO_CLOSE);                                 // Close the lock
  ShutDown();                                               // Turn off the Travelling GeoBug
}

/**************************************************************************
* Function name: Print_DDDMMMMM()                                         *
* Purpose: Print DDD MM.MMM coordinates on the OLED on row three and four *
* Parameters: Pointer to lat and lon                                      *
* Returns: Nothing                                                        *
**************************************************************************/
void Print_DDDMMMMM(float &lat, float &lon)
{
  float dec_minutes_lat, dec_minutes_lon, minutes_lat, minutes_lon;
  int dec_degrees_lat, dec_degrees_lon;

  dec_degrees_lat = (int) lat;             // Take out the degrees for latitude
  dec_degrees_lon = (int) lon;             // Do the same for longitude
  dec_minutes_lat = lat - dec_degrees_lat; // Now, save the decimal minutes for latitude
  dec_minutes_lon = lon - dec_degrees_lon; // and the same for longitude
  minutes_lat = dec_minutes_lat * 60;      // Convert the decimal minutes to MM.MMM for latitude
  minutes_lon = dec_minutes_lon * 60;      // and for longitude

  oled.setCursor(0, 2);
  oled.print(F("Lat: "));
  oled.print(dec_degrees_lat);
  oled.print((char)223);                   // Degree character
  oled.print(F(" "));
  oled.print(minutes_lat,3);
  oled.setCursor(0, 3);
  oled.print(F("Lon: "));
  oled.print(dec_degrees_lon);
  oled.print((char)223);                   // Degree character
  oled.print(F(" "));
  if((int) minutes_lon <= 9) oled.print(F("0"));
  oled.print(minutes_lon,3);
  delay(7000);                             // 7 second delay, plenty of time so that user can take a picture of the display
}
/*****************************************************************
* Function name: FoundThePlace()                                 *
* Purpose: Display congratulations and instructions on the OLED, *
*          opens the box and plays a tune                        * 
* Parameters: None                                               *
* Returns: Nothing                                               *
******************************************************************/
void FoundThePlace()
{
  GPS.end();                                               // Shut down communication with the GPS, needs to be done when using tone() function
  if(!EEPROM.read(EEPROM_FOUND_TARGET)) {                  // Is FoundFlag false, then set the FoundFlag to true in EEPROM
    EEPROM.write(EEPROM_FOUND_TARGET, true);
  }
  Text("       ACCESS", "      GRANTED!", "", "", 0);      // Tell the user the good news <big grin>
  PlayMusic();                                             // Play a tune
  Text("  Congratulations", "You found the place", "Please read further", "instructions inside", 5000); // Display congratulations and instructions
  Text("   Unlocking box", "so that you can log", "the TB hidden inside", "", 5000);
  servo.write(SERVO_OPEN);                                 // Open box
  delay(2000);                                             // Pause for 2 seconds
  ShutDown();                                              // Turn off the Travelling GeoBug
}
/***************************************************************
* Function name: StillNotFound                                 *
* Purpose: Print distance on the OLED and the cardinals        *
* Parameters: Pointers to lat, lon and distance_meters         *
* Returns: Nothing                                             *
****************************************************************/
void StillNotFound(float &lat, float &lon, float &distance_meters)
{
  oled.clear();                                             // Clear display
  oled.setCursor(0, 0);
  oled.print(F(" Distance to target:"));
  oled.setCursor(5, 1);                                    // Center text (crude, no check made)
  if (distance_meters < 1000) {                            // Are we under 1000 meters from target?
    oled.print((int)distance_meters);                      // Yes, so print it in meters
    oled.print(F(" meter"));
  } else {
    oled.print((int)(round(distance_meters / 1000)));      // Otherwise print it as kilometers
    oled.print(F(" kilometer"));
    #ifdef DEBUGCODE
    oled.setCursor(0, 2);                                  // Third row on display
    oled.print(distance_meters);
    #endif
  }
  if (BeginnerMode) {                                      // If the user is a beginner, tell him the direction to go
    oled.setCursor(0, 2);                                  // Third row on display
    oled.print(F("You need to go "));
    oled.print(TinyGPS::cardinal(TinyGPS::course_to (lat, lon, Destination.LATITUDE, Destination.LONGITUDE))); // Print the cardinal
  }
    delay(5000);                                           // Wait 5 seconds
    Text("   ACCESS DENIED!", " You are not close", "      enough!", "  Keep on walking", 5000);   // Tell the user the sad news
}

// Text definitions here
void DisplayInitText(unsigned int del)
{
//             "                    " this is the max string length for the display
  oled.clear();                          // Clear display
  oled.setCursor(0, 0);                  // Set cursor at row one
  oled.print(F("  Travelling GeoBug"));  // Print row one
  oled.setCursor(0, 1);                  // Set cursor at row two
  oled.print(F("     TBXXXXXX"));        // Print row two
  oled.setCursor(0, 2);                  // Set cursor at row three
  oled.print(F("    Initializing"));     // Print row three
  oled.setCursor(0, 3);                  // Set cursor at row four
  oled.print(F("   please wait..."));    // Print row four
  delay(del);                            // Wait before returning
}

void DisplayWelcomeText(unsigned int del)
{
//            "                    " this is the max string length for the display
  oled.clear();                          // Clear display
  oled.setCursor(0, 0);                  // Set cursor at row one
  oled.print(F("   Welcome to the   ")); // Print row one
  oled.setCursor(0, 1);                  // Set cursor at row two
  oled.print(F("  Traveling GeoBug  ")); // Print row two
  oled.setCursor(0, 2);                  // Set cursor at row three
  oled.print(F("      TBXXXXX"));        // Print row three
  oled.setCursor(0, 3);                  // Set cursor at row four
  oled.print(F("Built by mickecamino")); // Print row four
  delay(del);                            // Wait before returning
}

