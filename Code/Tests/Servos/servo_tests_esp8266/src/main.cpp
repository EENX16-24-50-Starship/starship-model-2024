// PWM servo tests (3.3V esp8266)
// -----------------------------
// Created by Gunnar Edman

// Script to test PWM control of different servos with a 3.3 V amplitude 

#include <Arduino.h>
#include <Servo.h>

// Global declerations--------------------
Servo servo1;  // create servo object to control a servo
Servo servo2;  // create servo object to control a servo
int pos = 0;    // variable to store the servo position
int servoPin1 = 4;    // D2
int servoPin2 = 5;    // D1
int servo1Home = 82;      // degrees
int servo2Home = 107; // 88;      // degrees
int buttomPin = 12;   // D6
int ledPin = 2;

// Function declerations---------------------------
void sweep(int, int, int);
void manualSweep();
void fastTest();
void potControl();
void hobbexT1();

void setServoPos(int p) {
  servo1.write(servo1Home + p*1.5);
  servo2.write(servo2Home + p*1.5);
}

void setServoOffset(int p) {
  servo1.writeMicroseconds(servo1Home + p*1.5);
  servo2.write(servo2Home + p*1.5);
}

  void setup() {
  // servo1.attach(servoPin, 500, 2500);
  servo1.attach(servoPin1, 900, 2100);
  servo2.attach(servoPin2, 900, 2100);
  servo1.write(servo1Home);
  servo2.write(servo2Home);
  pinMode(buttomPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  // Servo position zero
  // servo1.writeMicroseconds(500);
  // delay(1000);
}

void loop() {
  // hobbexT1();

  // setServoPos(0);

  // while (!buttomPin) {
    // setServoPos(20);
    // delay(800);
    // setServoPos(-20);
    // delay(800);
    // sweep(0, 180, 1);
  // }

  setServoPos(20);
  delay(1000);
  setServoPos(-20);
  delay(1000);

  // setServoPos(0);
  // delay(10000);
  // delay(800);
  // setServoPos(10);
  // delay(1000);
}

// Functions---------------------------------------
void sweep(int p1, int p2, int d) {

  digitalWrite(ledPin, HIGH);
  // Move to p2 
  for (pos = p1; pos <= p2; pos += 1) {
    // In steps of 1 degree
    servo1.write(pos);
    servo2.write(pos);
    delay(d);
  }

  delay(400);

  digitalWrite(ledPin, LOW);
  // Move to p1
  for (pos = p2; pos >= p1; pos -= 1) {
    servo1.write(pos);
    servo2.write(pos);
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

// void potControl() {
//   potVal = analogRead(potPin);
//   servo1.writeMicroseconds(map(potVal, 0, 1023, 0, 180));
// }

void hobbexT1() {
  // Servo specs (duty cycle range): 
  // https://www.banggood.com/JX-Servo-PDI-6208MG-8kg-Servo-120-Degrees-High-Precision-Metal-Gear-Digital-Standard-Servo-p-1070889.html?cur_warehouse=CN
  // https://www.hobbex.se/catalog/product/view/id/1724/s/pdi-6208mg/category/2/

  servo1.write(0);
  servo2.write(0);
  delay(500);
  
  servo1.write(180);
  servo2.write(180);
  delay(1000);
}