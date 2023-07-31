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

int xAxis = 140;
int yAxis = 140;
int motorSpeedBottomLeft = 0;
int motorSpeedBottomRight = 0;
int motorSpeedTopLeft = 0;
int motorSpeedTopRight = 0;
int speedRange = 255;  // Define a value for speedRange
const int deadZone = 30; // Adjust dead zone as needed
const int smoothingFactor = 5; // Adjust smoothing factor as needed

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
    // Smooth the joystick inputs
    xAxis = smoothInput(BT.read());
    Serial.println(xAxis);
    delay(10);
    yAxis = smoothInput(BT.read());
    Serial.println(yAxis);
    // ... (same as before)

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
  if (yAxis > 130 && yAxis < 150) {
    HandleHorizontalMovement();
  } else {
    HandleVerticalMovement();
  }
}

void HandleHorizontalMovement() {
  if (xAxis < 130) {
    motorSpeedBottomLeft = map(xAxis, 130, 60, 0, speedRange);
    motorSpeedBottomRight = map(xAxis, 130, 60, 0, speedRange);
    motorSpeedTopLeft = map(xAxis, 130, 60, 0, speedRange);
    motorSpeedTopRight = map(xAxis, 130, 60, 0, speedRange);
    turnLeft();
  } else if (xAxis < 254 && xAxis > 150) {
    motorSpeedBottomLeft = map(xAxis, 150, 220, 0, speedRange);
    motorSpeedBottomRight = map(xAxis, 150, 220, 0, speedRange);
    motorSpeedTopLeft = map(xAxis, 150, 220, 0, speedRange);
    motorSpeedTopRight = map(xAxis, 150, 220, 0, speedRange);
    turnRight();
  }
}

void HandleVerticalMovement() {
  if (xAxis > 130 && xAxis < 150) {
    if (yAxis < 130) {
      forward();
    } else if (yAxis < 254 && yAxis > 150) {
      backward();
    }

    if (yAxis < 130) {
      motorSpeedBottomLeft = map(yAxis, 130, 60, 0, speedRange);
      motorSpeedBottomRight = map(yAxis, 130, 60, 0, speedRange);
      motorSpeedTopLeft = map(yAxis, 130, 60, 0, speedRange);
      motorSpeedTopRight = map(yAxis, 130, 60, 0, speedRange);
    } else if (yAxis > 150) {
      motorSpeedBottomLeft = map(yAxis, 150, 220, 0, speedRange);
      motorSpeedBottomRight = map(yAxis, 150, 220, 0, speedRange);
      motorSpeedTopLeft = map(yAxis, 150, 220, 0, speedRange);
      motorSpeedTopRight = map(yAxis, 150, 220, 0, speedRange);
    }
  } else {
    if (yAxis < 130) {
      forward();
    } else if (yAxis < 254 && yAxis > 150) {
      backward();
    }

    if (xAxis < 130) {
      motorSpeedBottomLeft = map(xAxis, 130, 60, 0, speedRange);
      motorSpeedBottomRight = 255;
      motorSpeedTopLeft = map(xAxis, 130, 60, 0, speedRange);
      motorSpeedTopRight = 255;
    } else if (xAxis < 254 && xAxis > 150) {
      motorSpeedBottomLeft = 255;
      motorSpeedBottomRight = map(xAxis, 150, 220, 0, speedRange);
      motorSpeedTopLeft = 255;
      motorSpeedTopRight = map(xAxis, 150, 220, 0, speedRange);
    }
  }
}
int smoothInput(int inputValue) {
  static int prevValue = 0;
  int smoothedValue = (inputValue + prevValue * (smoothingFactor - 1)) / smoothingFactor;
  prevValue = smoothedValue;
  if (abs(smoothedValue - 128) < deadZone) {
    return 128; // Center position if within dead zone
  } else {
    return smoothedValue;
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
