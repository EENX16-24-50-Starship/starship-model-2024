// PWM servo tests (3.3V esp8266)
// -----------------------------
// Created by Gunnar Edman

// This script programs the ESC. Specificallym it changes the BEC 
// voltage mode to either 5.4, 6 or 7.4 V depending on user input

#include <Arduino.h>
#include <Servo.h>

// Declare ESC
Servo dc_motor_1;
Servo dc_motor_2;

int dc_motor_1_pin = 4;
int dc_motor_2_pin = 5;
int buttomPin = 14;
int ledPin = 2;

void setup() {
  Serial.begin(9600);

  // Attach motors to servo object
  dc_motor_1.attach(dc_motor_1_pin);
  dc_motor_2.attach(dc_motor_2_pin);

// Config 
  pinMode(buttomPin, INPUT_PULLUP);

  // // Send full throttle for 5 seconds
  // dc_motor_1.write(1940);
  // dc_motor_2.write(1940);
}

// Main loop
void loop() {
  // Thrust range 0 to 100 [%] = 1100 to 1940 [microseconds]
  // -------------------------------------------------------
  while(!digitalRead(buttomPin)) {
    digitalWrite(ledPin, HIGH);
    Serial.print("1940 \n");
    dc_motor_1.write(1940);
    dc_motor_2.write(1940);
    // dc_motor_1.write(1100);
    // dc_motor_2.write(1100);
    // delay(1000);
    // dc_motor_1.write(1100);
    // dc_motor_2.write(1100);
    // delay(1000);  
  }
  dc_motor_1.write(1100);
  dc_motor_2.write(1100);

  // for (int i = 1100; i<=1940; i++) {
  //   dc_motor_1.write(i);
  //   dc_motor_2.write(i);
  //   delay(1);
  // }

  // for (int i = 1940; i<=1100; i--) {
  //   dc_motor_1.write(i);
  //   dc_motor_2.write(i);
  //   delay(1);
  // }
}