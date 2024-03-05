#include <Arduino.h>
#include <Servo.h>

// Global declerations--------------------
Servo servo1;  // create servo object to control a servo
int pos = 0;    // variable to store the servo position
//int ledPin = 2;
int servoPin = 9; 

// Function declerations---------------------------
void sweep(int, int, int);
void manualSweep();

void setup() {
  servo1.attach(servoPin);
  //pinMode(ledPin, OUTPUT);
}

void loop() {
//  sweep(0,180,15);
  manualSweep();
}

// Functions---------------------------------------
void sweep(int p1, int p2, int d) {

  //digitalWrite(ledPin, HIGH);
  // Move to p2 
  for (pos = p1; pos <= p2; pos += 1) {
    // In steps of 1 degree
    servo1.write(pos);
    delay(d);
  }

  delay(400);

  //digitalWrite(ledPin, LOW);
  // Move to p1
  for (pos = p2; pos >= p1; pos -= 1) {
    servo1.write(pos);
    delay(d);
  }
}

void manualSweep() {
  servo1.writeMicroseconds(500);
  delay(1000);
  
  servo1.writeMicroseconds(1500);
  delay(1500);

  servo1.writeMicroseconds(2500);
  delay(1500);

  // for (int t = 500; t <= 2500; t ++) {
  //   servo1.writeMicroseconds(t);
  //   delay(1);
  // }

  // delay(1000);
  
  // for (int t = 2500; t >= 500; t --) {
  //   servo1.writeMicroseconds(t);
  //   delay(1);
  // }

  servo1.writeMicroseconds(500);
  delay (1000);
}