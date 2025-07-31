#include <Servo.h>         // Include the Servo library
#include <DHT.h>           // Include the DHT sensor library by Adafruit
#include <Adafruit_Sensor.h> // Required for DHT sensor library
#include <Wire.h>          // Required for I2C communication
#include <LiquidCrystal_I2C.h> // Include the LiquidCrystal_I2C library

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

// --- DHT11 Sensor Setup ---
#define DHTPIN 46     // Digital pin connected to the DHT sensor data pin (e.g., D46 on Mega)
#define DHTTYPE DHT11   // DHT 11 (Other options: DHT22, DHT21)
DHT dht(DHTPIN, DHTTYPE); // Initialize DHT sensor object

// --- LCD I2C Setup ---
LiquidCrystal_I2C lcd(0x27, 20, 4); // Assuming a 20x4 LCD at address 0x27 (adjust as needed)

// --- Bluetooth HC-05 Setup ---
// Using Hardware Serial1 on Arduino Mega
// HC-05 TXD (transmit) -> Arduino Mega RX1 (Pin 19)
// HC-05 RXD (receive)  -> Arduino Mega TX1 (Pin 18)
const long bluetoothBaudRate = 9600; // Common baud rate for HC-05. Change to 38400 if needed.

int motorSpeed = 150; // A good starting speed (0-255). Adjust as needed.

// --- Thresholds for each MQ sensor ---
int mq1Threshold = 400; // For MQ1 (controls Motor A)
int mq2Threshold = 400; // For MQ2 (controls Motor B)
int mq3Threshold = 400; // For MQ3 (controls Motor C)
int mq4Threshold = 400; // For MQ4 (controls Motor D and Servo)

// --- GLOBAL VARIABLES for DHT readings ---
float dhtHumidity = NAN;
float dhtTemperature = NAN;

unsigned long lastDHTReadTime = 0;
const long dhtReadInterval = 2000; // Read DHT11 every 2 seconds (min interval for DHT11)
unsigned long lastDisplayUpdateTime = 0;
const long displayUpdateInterval = 500; // Update LCD every 0.5 seconds for responsiveness

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

  // --- DHT Sensor Setup ---
  dht.begin(); // Start the DHT sensor

  // --- LCD Setup ---
  lcd.init();      // Initialize the LCD
  lcd.backlight(); // Turn on the backlight
  lcd.print("System Booting...");
  lcd.setCursor(0, 1);
  lcd.print("Loading Sensors...");
  delay(2000); // Give time to read messages

  // --- Serial (USB) Setup (for initial setup messages and backup debugging) ---
  Serial.begin(9600);
  Serial.println("System Initialized.");

  // --- Bluetooth (HC-05) Setup ---
  Serial1.begin(bluetoothBaudRate); // Initialize Serial1 for HC-05
  Serial.print("Bluetooth (HC-05) Initialized on Serial1 @ ");
  Serial.print(bluetoothBaudRate);
  Serial.println(" bps");
  Serial1.println("Bluetooth connected!"); // Send a test message
}

void loop() {
  unsigned long currentMillis = millis();

  // --- DHT11 Temperature and Humidity Reading (update global variables) ---
  if (currentMillis - lastDHTReadTime >= dhtReadInterval) {
    lastDHTReadTime = currentMillis;

    dhtHumidity = dht.readHumidity();
    dhtTemperature = dht.readTemperature(); // Reads temperature in Celsius (default)

    if (isnan(dhtHumidity) || isnan(dhtTemperature)) {
      // It's good to know if DHT fails, so we can keep a commented Serial.println for a quick test
      // Serial.println(F("Failed to read from DHT sensor!"));
    }
  }

  // --- MQ Gas Sensor Readings ---
  int mq1SensorValue = analogRead(mq1Pin);
  int mq2SensorValue = analogRead(mq2Pin);
  int mq3SensorValue = analogRead(mq3Pin);
  int mq4SensorValue = analogRead(mq4Pin);

  // --- Motor Control Logic (remains unchanged) ---
  if (mq1SensorValue > mq1Threshold) {
    forwardMotorA();
  } else {
    backwardMotorA();
  }

  if (mq2SensorValue > mq2Threshold) {
    forwardMotorB();
  } else {
    backwardMotorB();
  }

  if (mq3SensorValue > mq3Threshold) {
    forwardMotorC();
  } else {
    backwardMotorC();
  }

  // --- MQ4 Sensor, Motor D, AND Servo Control ---
  if (mq4SensorValue > mq4Threshold) {
    forwardMotorD();
    myServo.write(servoGasDetectedPos); // Move servo to gas detected position
  } else {
    backwardMotorD();
    myServo.write(servoNoGasPos);      // Move servo back to no gas position
  }

  // --- Prepare Data String for Serial Monitor and Bluetooth ---
  // Using a String object to build the output. Be mindful of memory usage in long-running apps.
  // For this kind of small, repeated string, it's generally fine on a Mega.
  String dataString = "";
  dataString += "Temp: ";
  if (!isnan(dhtTemperature)) dataString += String(dhtTemperature, 1); else dataString += "--";
  dataString += "C | Hum: ";
  if (!isnan(dhtHumidity)) dataString += String(dhtHumidity, 1); else dataString += "--";
  dataString += "% | MQ1: ";
  dataString += String(mq1SensorValue);
  dataString += " | MQ2: ";
  dataString += String(mq2SensorValue);
  dataString += " | MQ3: ";
  dataString += String(mq3SensorValue);
  dataString += " | MQ4: ";
  dataString += String(mq4SensorValue);

  // --- Display on Serial Monitor (USB) ---
  Serial.println(dataString);

  // --- Send Data via Bluetooth (HC-05) ---
  // Ensure the app can parse this exact string format
  Serial1.println(dataString);


  // --- LCD Display Update ---
  if (currentMillis - lastDisplayUpdateTime >= displayUpdateInterval) {
    lastDisplayUpdateTime = currentMillis;

    lcd.clear(); // Clear the display before writing new data

    // Line 1: Temperature and Humidity
    lcd.setCursor(0, 0);
    lcd.print("Temp:");
    if (isnan(dhtTemperature)) lcd.print("--"); else lcd.print(dhtTemperature, 1);
    lcd.print("C Hum:");
    if (isnan(dhtHumidity)) lcd.print("--"); else lcd.print(dhtHumidity, 1);
    lcd.print("%");

    // Line 2: MQ1 and MQ2
    lcd.setCursor(0, 1);
    lcd.print("MQ1:");
    lcd.print(mq1SensorValue);
    lcd.print(" MQ2:");
    lcd.print(mq2SensorValue);

    // Line 3: MQ3 and MQ4
    lcd.setCursor(0, 2);
    lcd.print("MQ3:");
    lcd.print(mq3SensorValue);
    lcd.print(" MQ4:");
    lcd.print(mq4SensorValue);

    // Optional Line 4: System Status / Motor States
    lcd.setCursor(0, 3);
    lcd.print("System Ready");
  }

  delay(50); // Small delay for the overall loop
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
  }
}