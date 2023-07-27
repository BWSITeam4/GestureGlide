#include <SoftwareSerial.h>
SoftwareSerial BT(2, 3); // RX, TX

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
int motorSpeedA = 0;
int motorSpeedB = 0;
int motorSpeedC = 0;
int motorSpeedD = 0;
int speedRange = 255; // Define a value for speedRange

const int deadZone = 10; // Define the dead zone around the center position

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
  BT.begin(9600); // Default communication rate of the Bluetooth module
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
  }

  // Calculate the motor speeds based on the joystick position
  // Motor Controller 1 (Bottom motors)
  if (xAxis > 140 - deadZone && xAxis < 140 + deadZone && yAxis > 140 - deadZone && yAxis < 140 + deadZone) {
    // Joystick in the center, stop the motors
    Stop();
    motorSpeedA = 0;
    motorSpeedB = 0;
  } else {
    // Calculate the motor speeds based on the joystick position for Motor Controller 1 (Bottom motors)
    if (yAxis > 140 + deadZone) {
      // Move forward
      forward();
      motorSpeedA = map(yAxis, 140 + deadZone, 255, 0, speedRange);
      motorSpeedB = map(yAxis, 140 + deadZone, 255, 0, speedRange);
      motorSpeedC = map(yAxis, 140 + deadZone, 255, 0, speedRange);
      motorSpeedD = map(yAxis, 140 + deadZone, 255, 0, speedRange);
    } else if (yAxis < 140 - deadZone) {
      // Move backward
      backward();
      motorSpeedA = map(yAxis, 140 - deadZone, 0, -speedRange, 0);
      motorSpeedB = map(yAxis, 140 - deadZone, 0, -speedRange, 0);
      motorSpeedC = map(yAxis, 140 - deadZone, 0, -speedRange, 0);
      motorSpeedD = map(yAxis, 140 - deadZone, 0, -speedRange, 0);
    } else {
      // Joystick in the vertical center position, stop the motors
      Stop();
      motorSpeedA = 0;
      motorSpeedB = 0;
    }
  }

  // Calculate the motor speeds based on the joystick position for Motor Controller 2 (Top motors)
  if (xAxis > 140 + deadZone) {
    // Turn right
    forwardRight();
    motorSpeedC = map(xAxis, 140 + deadZone, 255, 0, speedRange);
    motorSpeedD = map(xAxis, 140 + deadZone, 255, 0, speedRange);
  } else if (xAxis < 140 - deadZone) {
    // Turn left
    forwardLeft();
    motorSpeedC = map(xAxis, 140 - deadZone, 0, -speedRange, 0);
    motorSpeedD = map(xAxis, 140 - deadZone, 0, -speedRange, 0);
  } else {
    // Joystick in the horizontal center position, stop the motors
    Stop();
    motorSpeedC = 0;
    motorSpeedD = 0;
  }

  // Ensure motor speeds are within the valid range
  motorSpeedA = constrain(motorSpeedA, 0, 255);
  motorSpeedB = constrain(motorSpeedB, 0, 255);
  motorSpeedC = constrain(motorSpeedC, 0, 255);
  motorSpeedD = constrain(motorSpeedD, 0, 255);

  // Motor control functions for L298 Motor Controller 1
  setMotorSpeedL298_1(motorSpeedA, motorSpeedB);

  // Motor control functions for L298 Motor Controller 2
  setMotorSpeedL298_2(motorSpeedC, motorSpeedD);
}

// Function to set motor speeds for L298 Motor Controller 1
void setMotorSpeedL298_1(int speedA, int speedB) {
  // Ensure motor speeds are within the valid range
  speedA = constrain(speedA, 0, 255);
  speedB = constrain(speedB, 0, 255);

  // Set motor speeds
  analogWrite(bottomRightSpeed, speedA);
  analogWrite(bottomLeftSpeed, speedB);

  // Set motor directions based on motor speeds
  digitalWrite(bottomRightCW, speedA > 0 ? HIGH : LOW);
  digitalWrite(bottomRightCCW, speedA < 0 ? HIGH : LOW);

  digitalWrite(bottomLeftCW, speedB > 0 ? HIGH : LOW);
  digitalWrite(bottomLeftCCW, speedB < 0 ? HIGH : LOW);
}

// Function to set motor speeds for L298 Motor Controller 2
void setMotorSpeedL298_2(int speedA, int speedB) {
  // Ensure motor speeds are within the valid range
  speedA = constrain(speedA, 0, 255);
  speedB = constrain(speedB, 0, 255);

  // Set motor speeds
  analogWrite(topLeftSpeed, speedA);
  analogWrite(topRightSpeed, speedB);

  // Set motor directions based on motor speeds
  digitalWrite(topLeftCW, speedA > 0 ? HIGH : LOW);
  digitalWrite(topLeftCCW, speedA < 0 ? HIGH : LOW);

  digitalWrite(topRightCW, speedB > 0 ? HIGH : LOW);
  digitalWrite(topRightCCW, speedB < 0 ? HIGH : LOW);
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

void forwardLeft() {
  digitalWrite(bottomRightCW, HIGH);
  digitalWrite(bottomRightCCW, LOW);
  digitalWrite(bottomLeftCW, LOW);
  digitalWrite(bottomLeftCCW, HIGH);
  digitalWrite(topLeftCW, HIGH);
  digitalWrite(topLeftCCW, LOW);
  digitalWrite(topRightCW, LOW);
  digitalWrite(topRightCCW, HIGH);
}

void forwardRight() {
  digitalWrite(bottomRightCW, LOW);
  digitalWrite(bottomRightCCW, HIGH);
  digitalWrite(bottomLeftCW, HIGH);
  digitalWrite(bottomLeftCCW, LOW);
  digitalWrite(topLeftCW, LOW);
  digitalWrite(topLeftCCW, HIGH);
  digitalWrite(topRightCW, HIGH);
  digitalWrite(topRightCCW, LOW);
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
