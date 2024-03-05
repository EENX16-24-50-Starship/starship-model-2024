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


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  dc_motor_1.attach(dc_motor_1_pin);
  dc_motor_2.attach(dc_motor_2_pin);
  pinMode(buttomPin, INPUT_PULLUP);

  // Send full throttle for 5 seconds
  dc_motor_1.write(1940);
  dc_motor_2.write(1940);
}

// Main loop
void loop() {
  // Thrust range 0 to 100 [%] = 1100 to 1940 [microseconds]
  // -------------------------------------------------------
  while(digitalRead(buttomPin)) {
    dc_motor_1.write(1940);
    dc_motor_2.write(1940);
  }
}