#include <SoftwareSerial.h>
SoftwareSerial BT(2, 3); // RX, TX

// Motor control pins for L298 Motor Controller 1
#define enAONE 4
#define in1ONE 5
#define in2ONE 6
#define in3ONE 7
#define in4ONE 8
#define enBONE 9

// Motor control pins for L298 Motor Controller 2
#define enATWO 10
#define in1TWO 11
#define in2TWO 12
#define in3TWO 13
#define in4TWO 14
#define enBTWO 15

int x = 1;
int xAxis = 140, yAxis = 140;
int readByte;
int motorSpeedA = 0;
int motorSpeedB = 0;

const int RELAY_PIN = A5;

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
  delay(500);
}

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

  // Read sensor values
  int thermistorValue = analogRead(THERMISTOR_PIN);
  int photoresistorValue = analogRead(PHOTORESISTOR_PIN);

  // Convert sensor values to meaningful data (you need to calibrate this based on your sensors)
  float temperature = map(thermistorValue, 0, 1023, 0, 100); // Example calibration for temperature
  int lightLevel = map(photoresistorValue, 0, 1023, 0, 100); // Example calibration for light level

  // Send sensor data to the MIT App Inventor app
  BT.print("T:"); // Use a custom delimiter, "T:" for temperature
  BT.print(temperature);
  BT.print(",");
  BT.print("L:"); // Use a custom delimiter, "L:" for light level
  BT.print(lightLevel);
  BT.print(",");

  // Motor control logic...
  int speedRange = 255; // Maximum speed range for the motors
  int deadZone = 10; // Define the dead zone around the center position

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
        forword();
        motorSpeedA = map(yAxis, 140 - deadZone, 60, 0, speedRange);
        motorSpeedB = map(yAxis, 140 - deadZone, 60, 0, speedRange);
      } else if (yAxis > 140 + deadZone) {
        // Backward
        backword();
        motorSpeedA = map(yAxis, 140 + deadZone, 220, 0, speedRange);
        motorSpeedB = map(yAxis, 140 + deadZone, 220, 0, speedRange);
      }
    } else {
      // Move diagonally
      if (xAxis < 140 - deadZone && yAxis < 140 - deadZone) {
        // Forward-left
        forword();
        turnLeft();
        motorSpeedA = map(xAxis, 140 - deadZone, 60, 0, speedRange);
        motorSpeedB = 255;
      } else if (xAxis > 140 + deadZone && yAxis < 140 - deadZone) {
        // Forward-right
        forword();
        turnRight();
        motorSpeedA = 255;
        motorSpeedB = map(xAxis, 140 + deadZone, 220, 0, speedRange);
      } else if (xAxis < 140 - deadZone && yAxis > 140 + deadZone) {
        // Backward-left
        backword();
        turnLeft();
        motorSpeedA = map(xAxis, 140 - deadZone, 60, speedRange, 50);
        motorSpeedB = 255;
      } else if (xAxis > 140 + deadZone && yAxis > 140 + deadZone) {
        // Backward-right
        backword();
        turnRight();
        motorSpeedA = 255;
        motorSpeedB = map(xAxis, 140 + deadZone, 220, speedRange, 50);
      }
    }
  }

  // Motor control functions for L298 Motor Controller 1
  setMotorSpeedL298_1(motorSpeedA, motorSpeedB);

  // Motor control functions for L298 Motor Controller 2
  setMotorSpeedL298_2(motorSpeedA, motorSpeedB);

  analogWrite(enAONE, motorSpeedA); // Send PWM signal to motor A of L298 motor controller 1
  analogWrite(enBONE, motorSpeedB); // Send PWM signal to motor B of L298 motor controller 1
  analogWrite(enATWO, motorSpeedA); // Send PWM signal to motor A of L298 motor controller 2
  analogWrite(enBTWO, motorSpeedB); // Send PWM signal to motor B of L298 motor controller 2
}

// Motor control functions for L298 Motor Controller 1
void setMotorSpeedL298_1(int speedA, int speedB) {
  if (speedA > 0) {
    digitalWrite(in1ONE, HIGH);
    digitalWrite(in2ONE, LOW);
    analogWrite(enAONE, speedA);
  } else {
    digitalWrite(in1ONE, LOW);
    digitalWrite(in2ONE, LOW);
  }

  if (speedB > 0) {
    digitalWrite(in3ONE, HIGH);
    digitalWrite(in4ONE, LOW);
    analogWrite(enBONE, speedB);
  } else {
    digitalWrite(in3ONE, LOW);
    digitalWrite(in4ONE, LOW);
  }
}

// Motor control functions for L298 Motor Controller 2
void setMotorSpeedL298_2(int speedA, int speedB) {
  if (speedA > 0) {
    digitalWrite(in1TWO, HIGH);
    digitalWrite(in2TWO, LOW);
    analogWrite(enATWO, speedA);
  } else {
    digitalWrite(in1TWO, LOW);
    digitalWrite(in2TWO, LOW);
  }

  if (speedB > 0) {
    digitalWrite(in3TWO, HIGH);
    digitalWrite(in4TWO, LOW);
    analogWrite(enBTWO, speedB);
  } else {
    digitalWrite(in3TWO, LOW);
    digitalWrite(in4TWO, LOW);
  }
}


