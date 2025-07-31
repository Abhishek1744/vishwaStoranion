#include <Servo.h> // Include the Servo library

// --- Motor Driver 1 (Controls Motor A and Motor B) ---
// Motor A connections
int enA = 9;
int in1 = 8;
int in2 = 7;
// Motor B connections
int enB = 3;
int in3 = 5;
int in4 = 4;

// --- Motor Driver 2 (Controls Motor C and Motor D) ---
// You will need a second L298N motor driver module for these motors.
// Using dedicated digital pins on Arduino Mega.
// Motor C connections
int enC = 10;  // PWM pin
int in5 = 11;
int in6 = 12;
// Motor D connections
int enD = 6;   // PWM pin
int in7 = 2;
int in8 = 13;  // Using another dedicated digital pin

// --- MQ Gas Sensor Pins ---
int mq1Pin = A0; // Connect MQ1 analog output to Arduino Analog Pin A0
int mq2Pin = A1; // Connect MQ2 analog output to Arduino Analog Pin A1
int mq3Pin = A2; // Connect MQ3 analog output to Arduino Analog Pin A2
int mq4Pin = A3; // Connect MQ4 analog output to Arduino Analog Pin A3

// --- Servo Motor Setup ---
Servo myServo;      // Create a servo object
int servoPin = 44;  // Choose a digital pin for the servo motor (Mega has many!)
int servoGasDetectedPos = 90; // Angle for servo when gas is detected (e.g., 90 degrees)
int servoNoGasPos = 0;      // Angle for servo when no gas (e.g., 0 degrees)

int motorSpeed = 150; // A good starting speed (0-255). Adjust as needed.

// --- Thresholds for each MQ sensor ---
// *** IMPORTANT: Calibrate these values for your specific sensors and environment ***
// These values are typically 0-1023 for analogRead.
int mq1Threshold = 400; // For MQ1 (controls Motor A)
int mq2Threshold = 400; // For MQ2 (controls Motor B)
int mq3Threshold = 400; // For MQ3 (controls Motor C)
int mq4Threshold = 400; // For MQ4 (controls Motor D and Servo)

void setup() {
  // --- Set all motor control pins as outputs ---
  // Motor Driver 1
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Motor Driver 2
  pinMode(enC, OUTPUT);
  pinMode(in5, OUTPUT);
  pinMode(in6, OUTPUT);
  pinMode(enD, OUTPUT);
  pinMode(in7, OUTPUT);
  pinMode(in8, OUTPUT);

  // --- Initialize all motors to be off ---
  stopMotorA();
  stopMotorB();
  stopMotorC();
  stopMotorD();

  // --- Servo Setup ---
  myServo.attach(servoPin); // Attach the servo object to the servo pin
  myServo.write(servoNoGasPos); // Initialize servo to the "no gas" position

  Serial.begin(9600); // For debugging
  Serial.println("4 Motor, 4 MQ Sensor, and Servo control initialized on Arduino Mega.");
}

void loop() {
  // --- MQ1 Sensor and Motor A Control ---
  int mq1SensorValue = analogRead(mq1Pin);
  Serial.print("MQ1 Sensor Value (for Motor A): ");
  Serial.println(mq1SensorValue);
  if (mq1SensorValue > mq1Threshold) {
    Serial.println("MQ1: Gas detected! Motor A Forward.");
    forwardMotorA();
  } else {
    Serial.println("MQ1: Gas level low. Motor A Backward.");
    backwardMotorA();
  }

  // --- MQ2 Sensor and Motor B Control ---
  int mq2SensorValue = analogRead(mq2Pin);
  Serial.print("MQ2 Sensor Value (for Motor B): ");
  Serial.println(mq2SensorValue);
  if (mq2SensorValue > mq2Threshold) {
    Serial.println("MQ2: Gas detected! Motor B Forward.");
    forwardMotorB();
  } else {
    Serial.println("MQ2: Gas level low. Motor B Backward.");
    backwardMotorB();
  }

  // --- MQ3 Sensor and Motor C Control ---
  int mq3SensorValue = analogRead(mq3Pin);
  Serial.print("MQ3 Sensor Value (for Motor C): ");
  Serial.println(mq3SensorValue);
  if (mq3SensorValue > mq3Threshold) {
    Serial.println("MQ3: Gas detected! Motor C Forward.");
    forwardMotorC();
  } else {
    Serial.println("MQ3: Gas level low. Motor C Backward.");
    backwardMotorC();
  }

  // --- MQ4 Sensor, Motor D, AND Servo Control ---
  int mq4SensorValue = analogRead(mq4Pin);
  Serial.print("MQ4 Sensor Value (for Motor D and Servo): ");
  Serial.println(mq4SensorValue);
  if (mq4SensorValue > mq4Threshold) {
    Serial.println("MQ4: Gas detected! Motor D Forward & Servo to Gas Detected Pos.");
    forwardMotorD();
    myServo.write(servoGasDetectedPos); // Move servo to gas detected position
  } else {
    Serial.println("MQ4: Gas level low. Motor D Backward & Servo to No Gas Pos.");
    backwardMotorD();
    myServo.write(servoNoGasPos);      // Move servo back to no gas position
  }

  delay(500); // Small delay to avoid rapid motor/servo changes and allow sensors to stabilize
}

// --- Individual Motor Control Functions (Motor A, B, C, D) ---

// Motor A Functions (Driver 1, Motor 1)
void forwardMotorA() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enA, motorSpeed);
}
void backwardMotorA() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enA, motorSpeed);
}
void stopMotorA() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(enA, 0);
}

// Motor B Functions (Driver 1, Motor 2)
void forwardMotorB() {
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enB, motorSpeed);
}
void backwardMotorB() {
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enB, motorSpeed);
}
void stopMotorB() {
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enB, 0);
}

// Motor C Functions (Driver 2, Motor 1)
void forwardMotorC() {
  digitalWrite(in5, HIGH);
  digitalWrite(in6, LOW);
  analogWrite(enC, motorSpeed);
}
void backwardMotorC() {
  digitalWrite(in5, LOW);
  digitalWrite(in6, HIGH);
  analogWrite(enC, motorSpeed);
}
void stopMotorC() {
  digitalWrite(in5, LOW);
  digitalWrite(in6, LOW);
  analogWrite(enC, 0);
}

// Motor D Functions (Driver 2, Motor 2)
void forwardMotorD() {
  digitalWrite(in7, HIGH);
  digitalWrite(in8, LOW);
  analogWrite(enD, motorSpeed);
}
void backwardMotorD() {
  digitalWrite(in7, LOW);
  digitalWrite(in8, HIGH);
  analogWrite(enD, motorSpeed);
}
void stopMotorD() {
  digitalWrite(in7, LOW);
  digitalWrite(in8, LOW);
  analogWrite(enD, 0);
}

// Optional: Function to set a new speed for all motors (can be modified for individual control)
void setMotorSpeed(int newSpeed) {
  if (newSpeed >= 0 && newSpeed <= 255) {
    motorSpeed = newSpeed;
    Serial.print("Motor speed set to: ");
    Serial.println(motorSpeed);
  } else {
    Serial.println("Invalid speed. Speed must be between 0 and 255.");
  }
}