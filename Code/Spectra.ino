#include <SoftwareSerial.h>
#include <Adafruit_VL53L0X.h>
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

// State handling manual = 0 vs autonomous = 1
int state_int = 0;
int state_button;

// Initializing lidar sensors
#define LOX1_ADDRESS 0x30  // front sensor
#define LOX2_ADDRESS 0x31  // left sensor
#define LOX3_ADDRESS 0x32  // right sensor
int front_sensor, left_sensor, right_sensor;

// setting constant safe distance
const int safeDistance = 50;
int speed = 0;
int mapSpeedX, mapSpeedY = 0;

// set the pins to shutdown
#define SHT_LOX1 2  // shutdown pin
#define SHT_LOX2 3
#define SHT_LOX3 4

// objects for the vl53l0x
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();

// this holds the measurement
VL53L0X_RangingMeasurementData_t front;
VL53L0X_RangingMeasurementData_t left;
VL53L0X_RangingMeasurementData_t right;

// Motor variables
int xAxis = 140;
int yAxis = 140;
int motorSpeedBottomLeft = 0;
int motorSpeedBottomRight = 0;
int motorSpeedTopLeft = 0;
int motorSpeedTopRight = 0;
int speedRange = 255;     // Define a value for speedRange
const int deadZone = 20;  // Increase the dead zone to 20 (adjust as needed)

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

  while (!Serial) { delay(1); }
  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  pinMode(SHT_LOX3, OUTPUT);
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);

  // Start Bluetooth communication
  BT.begin(9600);  // Default communication rate of the Bluetooth module
}

void loop() {
  // Read the incoming data from the Smartphone Android App

  while (BT.available() >= 2) {
    xAxis = BT.read();
    delay(10);
    yAxis = BT.read();
    delay(10);
    if (xAxis == 254 && yAxis == 254) {
      state_int = 1;
      Serial.println("Starting...");
      setID();
    } else if (xAxis == 255 && yAxis == 255) {
      Serial.println("Manual");
      digitalWrite(SHT_LOX1, LOW);
      digitalWrite(SHT_LOX2, LOW);
      digitalWrite(SHT_LOX3, LOW);
      state_int = 0;
    }

    switch (state_int) {
      case 0:
        // Manual mode
        if (BT.available() >= 2) {
          xAxis = BT.read();
          delay(10);
          yAxis = BT.read();
          Serial.print("xAxis: ");
          Serial.print(xAxis);
          Serial.print(", yAxis: ");
          Serial.println(yAxis);
          mapSpeedX = xAxis;
          mapSpeedY = yAxis;
        }

        if (xAxis > 130 && xAxis < 150 && yAxis > 130 && yAxis < 150) {
          Stop();
        } else if (!(xAxis == 254 ||  yAxis == 254 || xAxis == 255 || yAxis == 255)) {
          HandleMovement();
        }

        analogWrite(bottomRightSpeed, motorSpeedBottomRight);
        analogWrite(topRightSpeed, motorSpeedTopRight);
        analogWrite(bottomLeftSpeed, motorSpeedBottomLeft);
        analogWrite(topLeftSpeed, motorSpeedTopLeft);
        break;
      case 1:
        // Automatic mode
        Serial.println("Automatic mode");
        mapSpeedX = 255;
        mapSpeedY = 255;
        read_sensors();

        if (front_sensor < safeDistance) {
          Stop();
          delay(500);  //allowing the rover to stop

          // Refine the decision based on the distance from the side obstacles
          if (left_sensor > right_sensor) {
            if (left_sensor - right_sensor > safeDistance) {
              turnLeft();
            } else {
              forwardLeft();
            }
          } else {
            if (right_sensor - left_sensor > safeDistance) {
              turnRight();
            } else {
              forwardRight();
            }
          }
          delay(500);  //allowing the rover to change its direction
        } else {
          if (left_sensor < safeDistance) {
            turnRight();
          } else if (right_sensor < safeDistance) {
            turnLeft();
          } else {
            forward();
          }
        }

        analogWrite(bottomRightSpeed, motorSpeedBottomRight);
        analogWrite(topRightSpeed, motorSpeedTopRight);
        analogWrite(bottomLeftSpeed, motorSpeedBottomLeft);
        analogWrite(topLeftSpeed, motorSpeedTopLeft);

        delay(100);
        break;
    }
  }
}

void HandleMovement() {
  if (yAxis <= 130) {
    if (xAxis <= 120) {
      forwardLeft();
    } else if (xAxis >= 160) {
      forwardRight();
    } else {
      forward();
    }
  } else if (yAxis >= 150) {
    if (xAxis <= 120) {
      backwardLeft();
    } else if (xAxis >= 160) {
      backwardRight();
    } else {
      backward();
    }
  } else {
    if (xAxis <= 120) {
      turnLeft();
    } else if (xAxis >= 160) {
      turnRight();
    } else {
      Stop();
    }
  }
}

// Motor control functions for movement directions
void forward() {
  int speed = map(yAxis, 110, 50, 0, speedRange);
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
  int speed = map(yAxis, 160, 200, 0, speedRange);
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
  int speed = map(xAxis, 110, 50, 0, speedRange);
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
  int speed = map(xAxis, 160, 200, 0, speedRange);
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
  motorSpeedTopRight = 255;
  motorSpeedBottomRight = 255;
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
  motorSpeedTopLeft = 255;
  motorSpeedBottomLeft = 255;
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
  motorSpeedTopRight = 255;
  motorSpeedBottomRight = 255;
  digitalWrite(bottomRightCW, LOW);
  digitalWrite(bottomRightCCW, HIGH);
  digitalWrite(bottomLeftCW, LOW);
  digitalWrite(bottomLeftCCW, LOW);
  digitalWrite(topLeftCW, LOW);
  digitalWrite(topLeftCCW, LOW);
  digitalWrite(topRightCW, LOW);
  digitalWrite(topRightCCW, HIGH);
}

void backwardRight() {
  motorSpeedTopLeft = 255;
  motorSpeedBottomLeft = 255;
  digitalWrite(bottomRightCW, LOW);
  digitalWrite(bottomRightCCW, LOW);
  digitalWrite(bottomLeftCW, LOW);
  digitalWrite(bottomLeftCCW, HIGH);
  digitalWrite(topLeftCW, LOW);
  digitalWrite(topLeftCCW, HIGH);
  digitalWrite(topRightCW, LOW);
  digitalWrite(topRightCCW, LOW);
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

void setID() {
  // all reset
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  delay(10);
  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  digitalWrite(SHT_LOX3, HIGH);
  delay(10);

  // activating LOX1 and reseting LOX2
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);

  // initing LOX1
  if (!lox1.begin(LOX1_ADDRESS)) {
    Serial.println(F("Failed to boot first VL53L0X"));
    while (1)
      ;
  }
  delay(10);

  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  //initing LOX2
  if (!lox2.begin(LOX2_ADDRESS)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    while (1)
      ;
  }

  // activating LOX2
  digitalWrite(SHT_LOX3, HIGH);
  delay(10);

  //initing LOX2
  if (!lox3.begin(LOX3_ADDRESS)) {
    Serial.println(F("Failed to boot third VL53L0X"));
    while (1)
      ;
  }
}

void read_sensors() {

  lox1.rangingTest(&front, false);  // pass in 'true' to get debug data printout!
  lox2.rangingTest(&left, false);   // pass in 'true' to get debug data printout!
  lox3.rangingTest(&right, false);  // pass in 'true' to get debug data printout!

  // print sensor one reading
  Serial.print(F("F: "));
  if (front.RangeStatus != 4) {  // if not out of range
    front_sensor = front.RangeMilliMeter;
    Serial.print(front_sensor);
    Serial.print(F("mm"));
  } else {
    Serial.print(F("N.A."));
  }

  Serial.print(" ");

  // print sensor two reading
  Serial.print(F("L: "));
  if (left.RangeStatus != 4) {
    left_sensor = left.RangeMilliMeter;
    Serial.print(left_sensor);
    Serial.print(F("mm"));
  } else {
    Serial.print(F("N.A."));
  }

  Serial.print(" ");

  // print sensor two reading
  Serial.print(F("R: "));
  if (right.RangeStatus != 4) {
    right_sensor = right.RangeMilliMeter;
    Serial.print(right_sensor);
    Serial.print(F("mm"));
  } else {
    Serial.print(F("N.A."));
  }

  Serial.println();
}
