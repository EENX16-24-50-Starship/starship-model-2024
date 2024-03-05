// BLDC motor test (3.3 V esp8266)
// -----------------------------
// Created by Gunnar Edman

// This program tests the motors at a pre programemd 
// speed for a preprogrammed time duration
// (or controls speed with potentiometer if enabled)

// Program safety features
// -----------------------
// This program has some built in safety. Upon startup of the ESCs,
// one must press and hold the connected button for 2 seconds in order
// to enter calibration mode. This is to prevent that the microcontroller
// enters calibration by misstake and sends full throttle for 5 seconds
// to the ESC, when the ESC might be in armed flight mode.
// (This happened once during a motor test, which resultad in minor injurious)

#include <Arduino.h>
#include <Servo.h>

// Adjustable program parameters
int sHigh = 1150;         // Top speed of the motor tests
int duration = 2000;      // Duration: how long the top speed will be maintained
int speedLimit = 1170;    // Adjusts the maximum allowed speed during testing

// Declare ESC
Servo dc_motor_1;
Servo dc_motor_2;

// Pin assignment
int dc_motor_1_pin = 4;
int dc_motor_2_pin = 5;
int buttonPin = 14;
int potPin = PIN_A0;

// Global variable declaration
int t0 = 0;
int tPress = 0;
bool calibrated = false;
bool pot = true;          // <<<<-------- Change this depending on usage of potentiometer or not
int potVal = 0;


// Functions---------------------------
void motorsWrite(int s) {
  // For safty, set speed to zero and shut down if button is pressed
  if(digitalRead(buttonPin)) {
    dc_motor_1.write(1100);
    dc_motor_2.write(1100);  
    delay(10000);
    exit(0);
  }

  // If not, change motor speed
  else {
  dc_motor_1.write(s);
  dc_motor_2.write(s);  
  }
}

void abortCheck() {
  if(digitalRead(buttonPin)) {
    dc_motor_1.write(1100);
    dc_motor_2.write(1100);  
    delay(10000);
    exit(0);
  }
}

void rampUp(int lo, int high, int rampTime) {
  int rampDelay = int(rampTime/(high - lo));

  // Up ramp
  for (int i=lo; i<= high; i++) {
    motorsWrite(i);
    delay(rampDelay);
  } 
}

void rampDown(int lo, int high, int rampTime) {
  int rampDelay = int(rampTime/(high - lo));

  // Down ramp
  for (int i=high; i<=high; i--) {
    motorsWrite(i);
    delay(rampDelay);
  }
}

// ESC throttle calibration sequence
void escCalibration() {

  // Max throttle for 5 seconds
  dc_motor_1.write(1940);
  dc_motor_2.write(1940);
  delay(5000);                                    

  // Zero throttle for 3 seconds
  dc_motor_1.write(1100); 
  dc_motor_2.write(1100);                           
  delay(3000);

  calibrated = true;

  // Limit thrust range for safety
  dc_motor_1.attach(dc_motor_1_pin, 1100, speedLimit);
  dc_motor_2.attach(dc_motor_2_pin, 1100, speedLimit);
}

void motorTest(int lo, int high, int duration, int tUp, int tDown) {
  // Thrust range 0 to 100 [%] = 1100 to 1940 [microseconds]
  // -------------------------------------------------------
  rampUp(lo, high, tUp);

  // Maintain max-speed for 2 seconds while checking abort button
  int t1 = millis(); 
  int t2 = millis();
  while(t2-t1 < duration) {
    abortCheck();
    t2 = millis();
  }

  // Down ramp
  rampDown(lo, high, tDown);

  // Set motor speed to zero
  motorsWrite(1100);
  delay (2000);
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  // Configure pins
  dc_motor_1.attach(dc_motor_1_pin, 1100, 1940);
  dc_motor_2.attach(dc_motor_2_pin, 1100, 1940);
  pinMode(buttonPin, INPUT_PULLUP);
}

void loop() {
  // Only execute this code if the ESC is in calibration mode (never in armed mode)
  if (!calibrated) {
    // Check for button press and duration off press
    t0 = millis();
    while(digitalRead(buttonPin)) { 
      tPress = millis();

      if(tPress-t0 >= 1000) {
        escCalibration();
      }
    }
  }

  // Test code that runs after throttle signal calibration  
  else {
    // Run pre-programmed speed test
    if(!pot) {
      // Set motor speed to zero
      dc_motor_1.write(1100);
      dc_motor_2.write(1100);

      // 0-speed, top speed, duration time, tUp, tDown
      motorTest(1100, sHigh, duration, 1000, 1000);
      delay(20000);
      exit(0);  
    }

    // Copuple motor speed to potentiometer
    else {
      potVal = map(analogRead(potPin), 0, 1023, 1100, 1940);
      motorsWrite(potVal);
    }
  }
}
