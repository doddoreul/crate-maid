#include <Arduino.h>

const int leftWheel = 11;
const int rightWheel = 10;

char readSerial;

void setup() {
  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
    readSerial = Serial.read();
    move(readSerial);
  }
}

void move(char dir) {
  if (dir == 'left'){
    // Set the left and right motor speeds
    analogWrite(leftWheel, 180);
    analogWrite(rightWheel, 0);
  } else if (dir == 'right') {
    // Set the left and right motor speeds
    analogWrite(leftWheel, 0);
    analogWrite(rightWheel, 180);
  } else if (dir == "foward") {
    // Set the left and right motor speeds to 0
    analogWrite(leftWheel, 180);
    analogWrite(rightWheel, 180);
  } else {
    analogWrite(leftWheel, 0);
    analogWrite(rightWheel, 0);
  }
}