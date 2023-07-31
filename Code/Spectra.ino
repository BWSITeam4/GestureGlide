#include <SoftwareSerial.h>
SoftwareSerial BT(10, 11);  // RX, TX

// Motor pin definitions
#define bottomRightSpeed 32
#define bottomRightCW 28
#define bottomRightCCW 30
#define bottomLeftCW 24
#define bottomLeftCCW 26
#define bottomLeftSpeed 22

#define topLeftSpeed 23
#define topLeftCW 25
#define topLeftCCW 27
#define topRightCW 31
#define topRightCCW 29
#define topRightSpeed 33

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

    if (xAxis > 130 && xAxis < 150 && yAxis > 130 && yAxis < 150) {
      Stop();
    } else {
      HandleMovement();
    }

    analogWrite(bottomRightSpeed, motorSpeedBottomRight);
    analogWrite(topRightSpeed, motorSpeedTopRight);
    analogWrite(bottomLeftSpeed, motorSpeedBottomLeft);
    analogWrite(topLeftSpeed, motorSpeedTopLeft);
  }
}

void HandleMovement() {
  if (yAxis <= 130) {
    if (xAxis <= 130) {
      forwardLeft();
    } else if (xAxis >= 150) {
      forwardRight();
    } else {
      forward();
    }
  } else if (yAxis >= 150) {
    if (xAxis <= 130) {
      backwardLeft();
    } else if (xAxis >= 150) {
      backwardRight();
    } else {
      backward();
    }
  } else {
    if (xAxis <= 130) {
      turnLeft();
    } else if (xAxis >= 150) {
      turnRight();
    } else {
      Stop();
    }
  }
}

// Motor control functions for movement directions
void forward() {
  int speed = map(yAxis, 130, 60, 0, speedRange);
  motorSpeedBottomLeft = speed;
  motorSpeedBottomRight = speed;
  motorSpeedTopLeft = speed;
  motorSpeedTopRight = speed;
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
  int speed = map(yAxis, 150, 220, 0, speedRange);
  motorSpeedBottomLeft = speed;
  motorSpeedBottomRight = speed;
  motorSpeedTopLeft = speed;
  motorSpeedTopRight = speed;
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
  int speed = map(xAxis, 130, 60, 0, speedRange);
  motorSpeedBottomLeft = speed;
  motorSpeedBottomRight = speed;
  motorSpeedTopLeft = speed;
  motorSpeedTopRight = speed;
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
  int speed = map(xAxis, 150, 220, 0, speedRange);
  motorSpeedBottomLeft = speed;
  motorSpeedBottomRight = speed;
  motorSpeedTopLeft = speed;
  motorSpeedTopRight = speed;
  digitalWrite(bottomRightCW, LOW);
  digitalWrite(bottomRightCCW, HIGH);
  digitalWrite(bottomLeftCW, HIGH);
  digitalWrite(bottomLeftCCW, LOW);
  digitalWrite(topLeftCW, HIGH);
  digitalWrite(topLeftCCW, LOW);
  digitalWrite(topRightCW, LOW);
  digitalWrite(topRightCCW, HIGH);
}

void forwardLeft() {
  int speedY = map(yAxis, 130, 60, 0, speedRange);
  int speedX = map(xAxis, 130, 60, 0, speedRange);
  motorSpeedBottomLeft = speedY;
  motorSpeedTopRight = speedY;
  motorSpeedBottomRight = speedX;
  motorSpeedTopLeft = 0;
  digitalWrite(bottomRightCW, HIGH);
  digitalWrite(bottomRightCCW, LOW);
  digitalWrite(bottomLeftCW, LOW);
  digitalWrite(bottomLeftCCW, LOW);
  digitalWrite(topLeftCW, LOW);
  digitalWrite(topLeftCCW, LOW);
  digitalWrite(topRightCW, HIGH);
  digitalWrite(topRightCCW, LOW);
}

void forwardRight() {
  int speedY = map(yAxis, 130, 60, 0, speedRange);
  int speedX = map(xAxis, 220, 150, 0, speedRange);
  motorSpeedBottomRight = speedY;
  motorSpeedTopLeft = speedY;
  motorSpeedBottomLeft = speedX;
  motorSpeedTopRight = 0;
  digitalWrite(bottomRightCW, LOW);
  digitalWrite(bottomRightCCW, LOW);
  digitalWrite(bottomLeftCW, HIGH);
  digitalWrite(bottomLeftCCW, LOW);
  digitalWrite(topLeftCW, HIGH);
  digitalWrite(topLeftCCW,LOW);
  digitalWrite(topRightCW, LOW);
  digitalWrite(topRightCCW, LOW);
}

void backwardLeft() {
  int speedY = map(yAxis, 220, 150, 0, speedRange);
  int speedX = map(xAxis, 130, 60, 0, speedRange);
  motorSpeedTopLeft = speedY;
  motorSpeedBottomRight = speedY;
  motorSpeedTopRight = speedX;
  motorSpeedBottomLeft = 0;
  digitalWrite(bottomRightCW, LOW);
  digitalWrite(bottomRightCCW, LOW);
  digitalWrite(bottomLeftCW, LOW);
  digitalWrite(bottomLeftCCW, HIGH);
  digitalWrite(topLeftCW, LOW);
  digitalWrite(topLeftCCW, HIGH);
  digitalWrite(topRightCW, LOW);
  digitalWrite(topRightCCW, LOW);
}

void backwardRight() {
  int speedY = map(yAxis, 220, 150, 0, speedRange);
  int speedX = map(xAxis, 220, 150, 0, speedRange);
  motorSpeedTopRight = speedY;
  motorSpeedBottomLeft = speedY;
  motorSpeedTopLeft = speedX;
  motorSpeedBottomRight = 0;
  digitalWrite(bottomRightCW, LOW);
  digitalWrite(bottomRightCCW, HIGH);
  digitalWrite(bottomLeftCW, LOW);
  digitalWrite(bottomLeftCCW, LOW);
  digitalWrite(topLeftCW, LOW);
  digitalWrite(topLeftCCW, LOW);
  digitalWrite(topRightCW, LOW);
  digitalWrite(topRightCCW, HIGH);
}

void Stop() {
  motorSpeedBottomLeft = 0;
  motorSpeedBottomRight = 0;
  motorSpeedTopLeft = 0;
  motorSpeedTopRight = 0;
  digitalWrite(bottomRightCW, LOW);
  digitalWrite(bottomRightCCW, LOW);
  digitalWrite(bottomLeftCW, LOW);
  digitalWrite(bottomLeftCCW, LOW);
  digitalWrite(topLeftCW, LOW);
  digitalWrite(topLeftCCW, LOW);
  digitalWrite(topRightCW, LOW);
  digitalWrite(topRightCCW, LOW);
}
