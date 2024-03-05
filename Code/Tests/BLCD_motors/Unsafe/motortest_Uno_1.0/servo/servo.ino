#include <Servo.h>


Servo myservo;  // create servo object to control a servo
Servo myservo2;
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

void setup() {
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  myservo2.attach(6);
}

void loop() {
  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos''
    myservo2.write(pos);
    delay(15);                     // waits 15ms for the servo to reach the position
  }
  myservo.write(0);              // tell servo to go to position in variable 'pos''
  myservo2.write(0);
  delay(1000);
}




