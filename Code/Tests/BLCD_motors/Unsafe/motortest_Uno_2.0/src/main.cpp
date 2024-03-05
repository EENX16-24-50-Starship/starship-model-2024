#include <Arduino.h>
#include <Servo.h>
  
Servo dc_motor_1;
Servo dc_motor_2;                                 // declare ESC

int dc_motor_1_pin = A2;
int dc_motor_2_pin = A1;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  dc_motor_1.attach(dc_motor_1_pin);
  dc_motor_2.attach(dc_motor_2_pin);

  dc_motor_1.write(1940);                                   //Initialize main ESC calibration sequence
  dc_motor_2.write(1940);
  delay(5000);                                    
  dc_motor_1.write(1100);
  dc_motor_2.write(1100);                           
  delay(3000);  
}

void loop() {
  // Thrust 1100 to 1940

  // Set motor speed to zero
  dc_motor_1.write(1100);
  dc_motor_2.write(1100);

  delay(4000);

  int lo = 1100;
  int high = 1800;
  int rampTime = 3000;
  int rampDelay = int(rampTime/(high - lo));

  // Up ramp
  for (int i=lo; i<= high; i++) {
    dc_motor_1.write(i);
    dc_motor_2.write(i);
    delay(rampDelay);
  }

  delay(2000);

  // Down ramp
  int rampTimeD = 1000;
  int rampDelayD = int(rampTimeD/(high - lo));

  for (int i=high; i<=high; i--) {
    dc_motor_1.write(i);                           // initialize ESC  
    dc_motor_2.write(i);
    delay(rampDelayD);
  }

  // Set motor speed to zero
  dc_motor_1.write(1100);                           // initialize ESC  
  dc_motor_2.write(1100);
  delay(20000);                                // delay for stabilization                        

/*
  dc_motor_1.write(1500);                           // initialize ESC  
  dc_motor_2.write(1500);
  delay(5000);                                // delay for stabilization                        
*/
  
/*  dc_motor_1.write(1180);                           // initialize ESC  
  dc_motor_2.write(1180);
  delay(4000);
  
  dc_motor_1.write(1100);                           // initialize ESC  
  dc_motor_2.write(1100);
  delay(6000);
  //exit(0);  
  */                       
}