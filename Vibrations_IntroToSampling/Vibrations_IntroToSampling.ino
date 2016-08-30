/* LED Blink, Teensyduino Tutorial #1
   http://www.pjrc.com/teensy/tutorial.html
 
   This example code is in the public domain.
*/

// Teensy 2.0 has the LED on pin 11
// Teensy++ 2.0 has the LED on pin 6
// Teensy 3.0 has the LED on pin 13

#include  <i2c_t3.h> // the I2C library that replaces Wire.h for the Teensy 3.2

const double gyroScaleFactor = 0.003814697265625; //Set to 125deg/s / 2^15
const double gyroOffset = -0.122513335; //Used to zero out the rate gyro when it is still. Uses a longterm average.

#include "BNO055.h"


const int ledPin = LED_BUILTIN;

elapsedMillis ledTimer;
elapsedMillis ledDuration;

boolean ledState = false;

// the setup() method runs once, when the sketch starts

void setup() {
  // initialize the digital pin as an output.
  pinMode(ledPin, OUTPUT);
}

// the loop() methor runs over and over again,
// as long as the board has power

void loop() {
  if ( ledTimer >= 50){
    ledTimer = 0;
    ledState = !ledState;
    digitalWrite(ledPin,ledState);
    Serial.print(BNOgetAccelX());
    Serial.print("\t");
    Serial.println();
  }
  
 
}

