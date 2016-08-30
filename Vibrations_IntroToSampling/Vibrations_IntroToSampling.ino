/* LED Blink, Teensyduino Tutorial #1
   http://www.pjrc.com/teensy/tutorial.html
 
   This example code is in the public domain.
*/

// Teensy 2.0 has the LED on pin 11
// Teensy++ 2.0 has the LED on pin 6
// Teensy 3.0 has the LED on pin 13

#include  <i2c_t3.h> // the I2C library that replaces Wire.h for the Teensy 3.2

#define PS0Pin      2
#define resetPin    22
#define IMUIntPin   6
#define STPin       17

#define gyroScaleFactor  0.003814697265625  //Set to 125deg/s / 2^1
#define gyroOffset      -0.122513335        //Used to zero out the rate gyro when it is still. Uses a longterm average.

#include "BNO055.h"


#define ledPin LED_BUILTIN

elapsedMillis ledTimer;
elapsedMillis ledDuration;

boolean ledState = false;

// the setup() method runs once, when the sketch starts

void setup() {
  // initialize the digital pin as an output.
  pinMode(ledPin, OUTPUT);
 
  pinMode(PS0Pin,OUTPUT);
  digitalWrite(PS0Pin,LOW);
  pinMode(resetPin,OUTPUT);
  digitalWrite(resetPin,LOW);
  pinMode(STPin,OUTPUT);
  digitalWrite(STPin,LOW);

  
  pinMode(IMUIntPin,INPUT);
  
  delay(20);
  digitalWrite(resetPin,HIGH);
  
      Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);


  Serial.begin(115200); //debug console
  delay(1000);

  Serial.println("Starting Bosch BNO055 IMU Sensor... ");

  /*
   System Status:
   0 System idle,
  1 System Error,
  2 Init ializing peripherals
  3 System Init ializat ion
  4 Execut ing selftest ,
  5 Sensor fusion algorithm running,
  6 System running without fusion algorithm
   */
  while (getBNOSystemStatus() < 5 ){
    
    BNOwrite(BNO055_OPR_MODE_ADDR, OPERATION_MODE_CONFIG); //Set to configuration Mode
    delay(20);
    BNOwrite(BNO055_OPR_MODE_ADDR,OPERATION_MODE_AMG);  
    delay(200);
  }
  Serial.println("Verifying BNO055 Settings...");
  getBNO055Status();
  
//
//    BNOwrite(BNO055_OPR_MODE_ADDR, OPERATION_MODE_CONFIG); //Set to configuration Mode
//    delay(20);
//    BNOwrite(BNO055_OPR_MODE_ADDR, OPERATION_MODE_CONFIG); //Set to configuration Mode
//    delay(20);
//    BNOwrite(BNO055_OPR_MODE_ADDR, OPERATION_MODE_CONFIG); //Set to configuration Mode
//    delay(20);
//    Serial.println("Done.");
//
//    // Set to use external oscillator
//    Serial.print("Setting up for External oscillator... ");
//    if (BNOread(BNO055_SYS_CLK_STAT_ADDR) == 0) {
//      BNOwrite(BNO055_SYS_TRIGGER_ADDR, 0b10000000);
//      Serial.println("BNO055 set to use external oscillator");
//    }
//    else
//    {
//      BNOwrite(BNO055_SYS_TRIGGER_ADDR, 0b10000000);
//      Serial.println("Problem setting BNO055 to use external oscillator.");
//    }
//    delay(10);
//
//    Serial.print("Setting up Rate Gyro... ");
//    BNOwrite(BNO055_PAGE_ID_ADDR, 1); // Set to page 1
//    delay(10);
//    BNOwrite(GYRO_CONF_0, 0b00101100); // Set gyroscope to 125deg/s at 23Hz (see pg 28 of datasheet)
//    delay(10);
//    BNOwrite(GYRO_CONF_1, 0); // Set gyroscope  (see pg 28 of datasheet)
//    delay(10);
//
//
//    Serial.println("Done.");
//    Serial.print("Setting up Magnetometer... ");
//    BNOwrite(MAG_CONF, 0b00011001); //
//    delay(10);
//    Serial.println("Done.");
//
//    Serial.print("Setting up Accelerometer... ");
//    BNOwrite(ACC_CONF, 0); //
//    delay(10);
//    Serial.println("Done.");
//
//    Serial.print("Setting up Units... ");
//    BNOwrite(BNO055_PAGE_ID_ADDR, 0); // Set to page 0
//    delay(10);
//    BNOwrite(BNO055_UNIT_SEL_ADDR, 0b00010000); // Set units to M/s^2 Deg/sec, Deg, Deg F
//    delay(10);
//    Serial.println("Done.");
//
//    Serial.print("Turning on Sensors... ");
//    //BNOwrite(BNO055_OPR_MODE_ADDR, OPERATION_MODE_AMG); //
//    BNOwrite(BNO055_OPR_MODE_ADDR,OPERATION_MODE_NDOF);//
//    delay(10);
//    
//    delay(10);
//    Serial.println("Done.");
//
//    Serial.println("Verifying BNO055 Settings...");
//    opMode = getBNO055Status();
//  }
//  Serial.println("Done.");


  
}

// the loop() method runs over and over again,
// as long as the board has power

void loop() {
  if ( ledTimer >= 50){
    ledTimer = 0;
    ledState = !ledState;
    digitalWrite(ledPin,ledState);
    Serial.print(BNOgetAccelX());
    Serial.print("\t");
    Serial.println(BNOgetYawRate());
  }
  
 
}

