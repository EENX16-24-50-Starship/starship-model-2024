#include <Arduino.h>
#include <Servo.h>

// Global declerations--------------------
Servo servo1;  // create servo object to control a servo
Servo servo2;  // create servo object to control a servo
int pos = 0;    // variable to store the servo position
int servoPin1 = 9; 
int servoPin2 = 11;
int servo1Home = 90;      // degrees
int servo2Home = 88;//88;      // degrees

// Function declerations---------------------------
void sweep(int, int, int);
void controlledSweep();
void manualSweep();
void sweep2();
void hobbexT1();
void hobbexControlledSweep(int, int, int);

void setup() {
  // servo1.attach(servoPin1, 900, 2100);
  servo1.attach(servoPin1, 900, 2100);
  servo2.attach(servoPin2, 900, 2100);
  servo1.write(servo1Home);
  servo2.write(servo2Home);
}

void loop() {
  // sweep(0,180,10);
  // sweep2();
  // hobbexT1();
  int d = 2;
  hobbexControlledSweep(0, -33*1.5, d);
  delay(800);
  hobbexControlledSweep(-33*1.5, 33*1.5, d);
  delay(800);
  hobbexControlledSweep(33*1.5, 0, d);
  delay(800);
}

  // 0 = 500
  // 60 = 1500
  // 120 = 2500

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
  servo1.writeMicroseconds(1300);
  servo2.writeMicroseconds(1300);
  delay(1000);
  
  servo1.writeMicroseconds(1500);
  servo2.writeMicroseconds(1500);
  delay(1500);

  servo1.writeMicroseconds(1700);
  servo2.writeMicroseconds(1700);
  delay(1500);


  servo1.writeMicroseconds(500);
  delay (1000);
}

void controlledSweep() {
  for (int t = 500; t <= 2500; t ++) {
    servo1.writeMicroseconds(t);
    delay(1);
  }

  delay(1000);
  
  for (int t = 2500; t >= 500; t --) {
    servo1.writeMicroseconds(t);
    delay(1);
  }
}

void sweep2() {
  int gRot = 34;
  int n1 = 83;
  int n2 = 86;//88;
  int del = 800;

  servo1.write(n1);
  servo2.write(n2);
  delay(1000);
  
  servo1.write(n1 - gRot);
  servo2.write(n2 - gRot*1.5);
  delay(del);

  servo1.write(n1 + gRot);
  servo2.write(n1 + gRot*1.5);
  delay(del); 

  servo1.write(n1);
  servo2.write(n2*1-5);
  delay(1000);
}

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


void hobbexControlledSweep(int p1, int p2, int d) {
  if (p1 < p2) {
    for (pos = p1; pos <= p2; pos += 1) {
      // In steps of 1 degree
      servo1.write(servo1Home + pos);
      servo2.write(servo2Home + pos);
      delay(d);
    }
  }

  else {
    for (pos = p2; pos >= p1; pos -= 1) {
      servo1.write(servo1Home + pos);
      servo2.write(servo2Home + pos);
      delay(d);
    }
  }
}