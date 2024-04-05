// BLDC motor test with gimbal(3.3 V esp8266)
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
int duration = 3000;      // Duration: how long the top speed will be maintained
int speedLimit = 1150;    // Adjusts the maximum allowed speed during testing
int gimb1 = 1000;
int gimb2 = 2000;
int gimb3 = 2800;
int gimb4 = 4000;
int gimb5 = 5000;

// Declare ESC
Servo dc_motor_1;
Servo dc_motor_2;
Servo servo1;
Servo servo2;

// Pin assignment
int dc_motor_1_pin = 14;   // D5
int dc_motor_2_pin = 12;   // D6
int buttonPin = 13;        // D7
int potPin = PIN_A0;       // A0
int ledPin = 2;

// Servos
int servoPin1 = 5;    // D1
int servoPin2 = 4;    // D2
int servo1Home = 82;      // degrees
int servo2Home = 107; // 88;      // degrees

// Global variable declaration
int t0 = 0;
int tPress = 0;
bool calibrated = false;
bool pot = false;          // <<<<-------- Change this depending on usage of potentiometer or not
int potVal = 0;

// Servo variables
int pos = 0;    // variable to store the servo position

// Functions---------------------------
// Servo functions
void setServoPos(int p) {
  servo1.write(servo1Home + p*1.5);
  servo2.write(servo2Home + p*1.5);
}

void motorsWrite(int s) {
  // For safty, set speed to zero and shut down if button is pressed
  if(!digitalRead(buttonPin)) {
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
  if(!digitalRead(buttonPin)) {
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
  delay(6000);                                    

  // Zero throttle for 3 seconds
  dc_motor_1.write(1100); 
  dc_motor_2.write(1100);                           
  delay(3000);

  calibrated = true;

  // Limit thrust range for safety
  dc_motor_1.attach(dc_motor_1_pin, 1100, speedLimit);
  dc_motor_2.attach(dc_motor_2_pin, 1100, speedLimit);
  dc_motor_1.write(1100); 
  dc_motor_2.write(1100);                           
}

void motorTest(int lo, int high, int duration, int tUp, int tDown) {
  // Thrust range 0 to 100 [%] = 1100 to 1940 [microseconds]
  // -------------------------------------------------------
  rampUp(lo, high, tUp);

  // Maintain max-speed for duration seconds while checking abort button
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

void motorGimbalTest(int lo, int high, int duration, int tUp, int tDown) {
  // Thrust range 0 to 100 [%] = 1100 to 1940 [microseconds]
  // -------------------------------------------------------
  rampUp(lo, high, tUp);

  // Maintain max-speed for "duration" seconds while checking abort button
  int t1 = millis(); 
  int t2 = millis();
  while (t2-t1 < duration) {
    abortCheck();
    int tStamp = t2-t1;

    // Perform gimbal at predefined time stamps
    if (tStamp >= gimb1-20 && tStamp <= gimb1+20) {
      setServoPos(-40);
    }

    else if (tStamp >= gimb2-20 && tStamp <= gimb2+20) {
      setServoPos(40);
    }

    else if (tStamp >= gimb3-20 && tStamp <= gimb3+20) {
      setServoPos(0);
    }

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
  dc_motor_1.write(1100); 
  dc_motor_2.write(1100);                           

  // Servo setup
  servo1.attach(servoPin1, 900, 2100);
  servo2.attach(servoPin2, 900, 2100);
  setServoPos(20);

  pinMode(buttonPin, INPUT_PULLUP);
}

void loop() {
  // Only execute this code if the ESC is in calibration mode (never in armed mode)
  if (!calibrated) {
    // Check for button press and duration off press
    t0 = millis();
    while (!digitalRead(buttonPin)) { 
      tPress = millis();

      if (tPress-t0 >= 1000) {
        escCalibration();
      }
    }
    setServoPos(45);
    delay(1000);
    setServoPos(-45);
    delay(1000);
    setServoPos(0);
  }

  // Test code that runs after throttle signal calibration  
  else {
    // setServoPos(45);
    // delay(1000); 
    // setServoPos(-45);
    // delay(1000);

    // Run pre-programmed speed test
    if(!pot) {
      // Set motor speed to zero
      dc_motor_1.write(1100);
      dc_motor_2.write(1100);

      // 0-speed, top speed, duration time, tUp, tDown
      motorGimbalTest(1100, sHigh, duration, 1000, 1000);
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
