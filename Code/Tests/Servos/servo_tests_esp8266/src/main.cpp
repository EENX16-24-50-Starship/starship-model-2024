// PWM servo tests (3.3V esp8266)
// -----------------------------
// Created by Gunnar Edman

// Script to test PWM control of different servos with a 3.3 V amplitude 

#include <Arduino.h>
#include <Servo.h>

// Pin declarations-----------------------
int ledPin = 2;
int servoPin = 0; 
int potPin = 5;

// Global declerations--------------------
Servo servo1;
int pos = 0;
int potVal = 0;

// Function declerations---------------------------
void sweep(int, int, int);
void manualSweep();
void fastTest();
void potControl();

void setup() {
  servo1.attach(servoPin, 500, 2500);
  pinMode(ledPin, OUTPUT);
  pinMode(potPin, INPUT);

  // Servo position zero
  // servo1.writeMicroseconds(500);
  // delay(1000);
}

void loop() {
  // sweep(0,180,1);
  // manualSweep();
  // fastTest();
  servo1.write(75);
  delay(200);
  
  servo1.write(120);
  delay(200);
}

// Functions---------------------------------------
void sweep(int p1, int p2, int d) {

  digitalWrite(ledPin, HIGH);
  // Move to p2 
  for (pos = p1; pos <= p2; pos += 1) {
    // In steps of 1 degree
    servo1.write(pos);
    delay(d);
  }

  delay(400);

  digitalWrite(ledPin, LOW);
  // Move to p1
  for (pos = p2; pos >= p1; pos -= 1) {
    servo1.write(pos);
    delay(d);
  }
}

void manualSweep() {
  servo1.writeMicroseconds(500);
  delay(4000);
  
  servo1.writeMicroseconds(1500);
  delay(4000);

  servo1.writeMicroseconds(2500);
  delay(4000);

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

void fastTest() {
  servo1.write(0);
  delay(1500);
  
  servo1.write(90);
  delay(1500);

  servo1.write(180);
  delay(1500);

  servo1.write(0);
  delay(3000);
}

void potControl() {
  potVal = analogRead(potPin);
  servo1.writeMicroseconds(map(potVal, 0, 1023, 0, 180));
}



