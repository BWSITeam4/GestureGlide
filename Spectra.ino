#include <SoftwareSerial.h>
SoftwareSerial BT(2, 3);  // RX, TX

// Motor pin definitions
#define bottomRightSpeed 4
#define bottomRightCW 5
#define bottomRightCCW 6
#define bottomLeftCW 7
#define bottomLeftCCW 8
#define bottomLeftSpeed 9

#define topLeftSpeed 10
#define topLeftCW 11
#define topLeftCCW 12
#define topRightCW 13
#define topRightCCW 14
#define topRightSpeed 15

// Motor variables
int xAxis = 140;
int yAxis = 140;
int motorSpeedBottomLeft = 0;
int motorSpeedBottomRight = 0;
int motorSpeedTopLeft = 0;
int motorSpeedTopRight = 0;
int speedRange = 255;  // Define a value for speedRange
const int deadZone = 20;  // Increase the dead zone to 30 (adjust as needed)


void setup() {
  // Set motor pins as OUTPUT
  pinMode(bottomRightSpeed, OUTPUT);
  pinMode(bottomLeftSpeed, OUTPUT);
  pinMode(topLeftSpeed, OUTPUT);
  pinMode(topRightSpeed, OUTPUT);
  pinMode(bottomRightCW, OUTPUT);
  pinMode(bottomRightCCW, OUTPUT);
  pinMode(bottomLeftCW, OUTPUT);
  pinMode(bottomLeftCCW, OUTPUT);
  pinMode(topLeftCW, OUTPUT);
  pinMode(topLeftCCW, OUTPUT);
  pinMode(topRightCW, OUTPUT);
  pinMode(topRightCCW, OUTPUT);

  // Start serial communication for debugging
  Serial.begin(9600);

  // Start Bluetooth communication
  BT.begin(9600);  // Default communication rate of the Bluetooth module
}
void loop() {
  // Read the incoming data from the Smartphone Android App
  while (BT.available() >= 2) {
    xAxis = BT.read();
    delay(10);
    yAxis = BT.read();
    Serial.print("xAxis: ");
    Serial.print(xAxis);
    Serial.print(", yAxis: ");
    Serial.println(yAxis);

    if (abs(xAxis - 140) < deadZone && abs(yAxis - 140) < deadZone) {
      Stop();
    } else {
      if (yAxis < 140 - deadZone) {
        // Moving forward
        motorSpeedBottomLeft = map(yAxis, 0, 140 - deadZone, 0, 255);
        motorSpeedBottomRight = map(yAxis, 0, 140 - deadZone, 0, 255);
        motorSpeedTopLeft = map(yAxis, 0, 140 - deadZone, 0, 255);
        motorSpeedTopRight = map(yAxis, 0, 140 - deadZone, 0, 255);
      } else if (yAxis > 140 + deadZone) {
        // Moving backward
        motorSpeedBottomLeft = map(yAxis, 140 + deadZone, 255, 0, 255);
        motorSpeedBottomRight = map(yAxis, 140 + deadZone, 255, 0, 255);
        motorSpeedTopLeft = map(yAxis, 140 + deadZone, 255, 0, 255);
        motorSpeedTopRight = map(yAxis, 140 + deadZone, 255, 0, 255);
      } else {
        motorSpeedBottomLeft = 0;
        motorSpeedBottomRight = 0;
        motorSpeedTopLeft = 0;
        motorSpeedTopRight = 0;
      }

      if (xAxis < 140 - deadZone) {
        // Moving left
        motorSpeedBottomLeft += map(xAxis, 0, 140 - deadZone, 0, 255);
        motorSpeedBottomRight -= map(xAxis, 0, 140 - deadZone, 0, 255);
        motorSpeedTopLeft -= map(xAxis, 0, 140 - deadZone, 0, 255);
        motorSpeedTopRight += map(xAxis, 0, 140 - deadZone, 0, 255);
      } else if (xAxis > 140 + deadZone) {
        // Moving right
        motorSpeedBottomLeft -= map(xAxis, 140 + deadZone, 255, 0, 255);
        motorSpeedBottomRight += map(xAxis, 140 + deadZone, 255, 0, 255);
        motorSpeedTopLeft += map(xAxis, 140 + deadZone, 255, 0, 255);
        motorSpeedTopRight -= map(xAxis, 140 + deadZone, 255, 0, 255);
      }

      // Ensure motor speeds are within the valid range
      motorSpeedBottomLeft = constrain(motorSpeedBottomLeft, 0, 255);
      motorSpeedBottomRight = constrain(motorSpeedBottomRight, 0, 255);
      motorSpeedTopLeft = constrain(motorSpeedTopLeft, 0, 255);
      motorSpeedTopRight = constrain(motorSpeedTopRight, 0, 255);

      // Apply the motor speeds
      analogWrite(bottomRightSpeed, motorSpeedBottomRight);
      analogWrite(topRightSpeed, motorSpeedTopRight);
      analogWrite(bottomLeftSpeed, motorSpeedBottomLeft);
      analogWrite(topLeftSpeed, motorSpeedTopLeft);
    }
  }
}




  // Motor control functions for movement directions
  void forward() {
    digitalWrite(bottomRightCW, HIGH);
    digitalWrite(bottomRightCCW, LOW);
    digitalWrite(bottomLeftCW, HIGH);
    digitalWrite(bottomLeftCCW, LOW);
    digitalWrite(topLeftCW, HIGH);
    digitalWrite(topLeftCCW, LOW);
    digitalWrite(topRightCW, HIGH);
    digitalWrite(topRightCCW, LOW);
  }

  void backward() {
    digitalWrite(bottomRightCW, LOW);
    digitalWrite(bottomRightCCW, HIGH);
    digitalWrite(bottomLeftCW, LOW);
    digitalWrite(bottomLeftCCW, HIGH);
    digitalWrite(topLeftCW, LOW);
    digitalWrite(topLeftCCW, HIGH);
    digitalWrite(topRightCW, LOW);
    digitalWrite(topRightCCW, HIGH);
  }

  void turnLeft() {
    digitalWrite(bottomRightCW, HIGH);
    digitalWrite(bottomRightCCW, LOW);
    digitalWrite(bottomLeftCW, LOW);
    digitalWrite(bottomLeftCCW, HIGH);
    digitalWrite(topLeftCW, LOW);
    digitalWrite(topLeftCCW, HIGH);
    digitalWrite(topRightCW, HIGH);
    digitalWrite(topRightCCW, LOW);
  }

  void turnRight() {
    digitalWrite(bottomRightCW, LOW);
    digitalWrite(bottomRightCCW, HIGH);
    digitalWrite(bottomLeftCW, HIGH);
    digitalWrite(bottomLeftCCW, LOW);
    digitalWrite(topLeftCW, HIGH);
    digitalWrite(topLeftCCW, LOW);
    digitalWrite(topRightCW, LOW);
    digitalWrite(topRightCCW, HIGH);
  }

  void Stop() {
    digitalWrite(bottomRightCW, LOW);
    digitalWrite(bottomRightCCW, LOW);
    digitalWrite(bottomLeftCW, LOW);
    digitalWrite(bottomLeftCCW, LOW);
    digitalWrite(topLeftCW, LOW);
    digitalWrite(topLeftCCW, LOW);
    digitalWrite(topRightCW, LOW);
    digitalWrite(topRightCCW, LOW);
  }
