#include <SoftwareSerial.h>
SoftwareSerial BT(2, 3); // RX, TX

// Motor control pins for L298 Motor Controller 1
#define enAONE 4
#define in1ONE 5
#define in2ONE 6
#define in3ONE 7
#define in4ONE 8
#define enBONE 9  // Assign a pin for motor controller 1, Motor B

// Motor control pins for L298 Motor Controller 2
#define enATWO 10 // Assign a pin for motor controller 2, Motor A
#define in1TWO 11
#define in2TWO 12
#define in3TWO 13
#define in4TWO 14
#define enBTWO 15 // Assign a pin for motor controller 2, Motor B

int xAxis = 140, yAxis = 140;
int motorSpeedA = 0;
int motorSpeedB = 0;

const int deadZone = 10; // Define the dead zone around the center position

void setup() {
  pinMode(enAONE, OUTPUT);
  pinMode(enBONE, OUTPUT); 
  pinMode(enATWO, OUTPUT);
  pinMode(enBTWO, OUTPUT); 
  pinMode(in1ONE, OUTPUT);
  pinMode(in2ONE, OUTPUT);
  pinMode(in3ONE, OUTPUT);
  pinMode(in4ONE, OUTPUT);
  pinMode(in1TWO, OUTPUT);
  pinMode(in2TWO, OUTPUT);
  pinMode(in3TWO, OUTPUT);
  pinMode(in4TWO, OUTPUT);

  Serial.begin(9600);
  BT.begin(9600); // Default communication rate of the Bluetooth module
}

// ... (rest of the code remains unchanged)

void loop() {
  // Read the incoming data from the Smartphone Android App
  while (BT.available() >= 2) {
    xAxis = BT.read();
    delay(10);
    yAxis = BT.read();
    Serial.print(xAxis);
    Serial.print(",");
    Serial.println(yAxis);
  }

  // Motor control logic...
  int speedRange = 255; // Maximum speed range for the motors

  if (xAxis > 140 - deadZone && xAxis < 140 + deadZone && yAxis > 140 - deadZone && yAxis < 140 + deadZone) {
    // Joystick in the center, stop the motors
    Stop();
    motorSpeedA = 0;
    motorSpeedB = 0;
  } else {
    // Calculate the motor speeds based on the joystick position
    if (yAxis > 140 - deadZone && yAxis < 140 + deadZone) {
      // Move left or right
      if (xAxis < 140 - deadZone) {
        // Turn left
        turnLeft();
        motorSpeedA = map(xAxis, 140 - deadZone, 60, 0, speedRange);
        motorSpeedB = map(xAxis, 140 - deadZone, 60, 0, speedRange);
      } else if (xAxis > 140 + deadZone) {
        // Turn right
        turnRight();
        motorSpeedA = map(xAxis, 140 + deadZone, 220, 0, speedRange);
        motorSpeedB = map(xAxis, 140 + deadZone, 220, 0, speedRange);
      }
    } else if (xAxis > 140 - deadZone && xAxis < 140 + deadZone) {
      // Move forward or backward
      if (yAxis < 140 - deadZone) {
        // Forward
        forward();
        motorSpeedA = map(yAxis, 140 - deadZone, 60, 0, speedRange);
        motorSpeedB = map(yAxis, 140 - deadZone, 60, 0, speedRange);
      } else if (yAxis > 140 + deadZone) {
        // Backward
        backward();
        motorSpeedA = map(yAxis, 140 + deadZone, 220, 0, speedRange);
        motorSpeedB = map(yAxis, 140 + deadZone, 220, 0, speedRange);
      }
    } else {
      // Move diagonally
      if (xAxis < 140 - deadZone && yAxis < 140 - deadZone) {
        // Forward-left
        forwardLeft();
        motorSpeedA = map(xAxis, 140 - deadZone, 60, 0, speedRange);
        motorSpeedB = 255;
      } else if (xAxis > 140 + deadZone && yAxis < 140 - deadZone) {
        // Forward-right
        forwardRight();
        motorSpeedA = 255;
        motorSpeedB = map(xAxis, 140 + deadZone, 220, 0, speedRange);
      } else if (xAxis < 140 - deadZone && yAxis > 140 + deadZone) {
        // Backward-left
        backwardLeft();
        motorSpeedA = map(xAxis, 140 - deadZone, 60, speedRange, 50);
        motorSpeedB = 255;
      } else if (xAxis > 140 + deadZone && yAxis > 140 + deadZone) {
        // Backward-right
        backwardRight();
        motorSpeedA = 255;
        motorSpeedB = map(xAxis, 140 + deadZone, 220, speedRange, 50);
      }
    }
  }

  // Ensure motor speeds are within the valid range
  motorSpeedA = constrain(motorSpeedA, 0, 255);
  motorSpeedB = constrain(motorSpeedB, 0, 255);

  // Motor control functions for L298 Motor Controller 1
  setMotorSpeedL298_1(motorSpeedA, motorSpeedB);

  // Motor control functions for L298 Motor Controller 2
  setMotorSpeedL298_2(motorSpeedA, motorSpeedB);
}

// ... (previous code remains the same)

// Motor control functions for L298 Motor Controller 1
void setMotorSpeedL298_1(int speedA, int speedB) {
  analogWrite(enAONE, speedA);
  analogWrite(enBONE, speedB);
  if (speedA > 0) {
    digitalWrite(in1ONE, HIGH);
    digitalWrite(in2ONE, LOW);
  } else {
    digitalWrite(in1ONE, LOW);
    digitalWrite(in2ONE, LOW);
  }

  if (speedB > 0) {
    digitalWrite(in3ONE, HIGH);
    digitalWrite(in4ONE, LOW);
  } else {
    digitalWrite(in3ONE, LOW);
    digitalWrite(in4ONE, LOW);
  }
}

// Motor control functions for L298 Motor Controller 2
void setMotorSpeedL298_2(int speedA, int speedB) {
  analogWrite(enATWO, speedA);
  analogWrite(enBTWO, speedB);
  if (speedA > 0) {
    digitalWrite(in1TWO, HIGH);
    digitalWrite(in2TWO, LOW);
  } else {
    digitalWrite(in1TWO, LOW);
    digitalWrite(in2TWO, LOW);
  }

  if (speedB > 0) {
    digitalWrite(in3TWO, HIGH);
    digitalWrite(in4TWO, LOW);
  } else {
    digitalWrite(in3TWO, LOW);
    digitalWrite(in4TWO, LOW);
  }
}

// ... (rest of the code remains the same)

// Your motor control functions (forward, backward, etc.) were not defined correctly
// I'll provide the correct implementations below:

void forward() {
  digitalWrite(in1ONE, HIGH);
  digitalWrite(in2ONE, LOW);
  digitalWrite(in3TWO, HIGH);
  digitalWrite(in4TWO, LOW);
}

void backward() {
  digitalWrite(in1ONE, LOW);
  digitalWrite(in2ONE, HIGH);
  digitalWrite(in3TWO, LOW);
  digitalWrite(in4TWO, HIGH);
}

void forwardLeft() {
  digitalWrite(in1ONE, HIGH);
  digitalWrite(in2ONE, LOW);
  digitalWrite(in3TWO, LOW);
  digitalWrite(in4TWO, HIGH);
}

void forwardRight() {
  digitalWrite(in1ONE, LOW);
  digitalWrite(in2ONE, HIGH);
  digitalWrite(in3TWO, HIGH);
  digitalWrite(in4TWO, LOW);
}

void backwardLeft() {
  digitalWrite(in1ONE, LOW);
  digitalWrite(in2ONE, HIGH);
  digitalWrite(in3TWO, LOW);
  digitalWrite(in4TWO, HIGH);
}

void backwardRight() {
  digitalWrite(in1ONE, HIGH);
  digitalWrite(in2ONE, LOW);
  digitalWrite(in3TWO, HIGH);
  digitalWrite(in4TWO, LOW);
}

void Stop() {
  digitalWrite(in1ONE, LOW);
  digitalWrite(in2ONE, LOW);
  digitalWrite(in3TWO, LOW);
  digitalWrite(in4TWO, LOW);
}
