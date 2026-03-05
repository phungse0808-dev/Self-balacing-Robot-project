#include <MPU6050_tockn.h>
#include <Wire.h>
#include <IRremote.h>

// --- Motor Pin Definitions ---
#define IN1 13
#define IN2 12
#define ENA 5
#define ENB 6
#define IN3 9
#define IN4 8

// --- IR Remote Pin ---
#define IR_PIN 3

MPU6050 mpu6050(Wire);
IRrecv irrecv(IR_PIN);
decode_results results;

// --- PID Parameters ---
float kp = 34, ki = 0.0052, kd = 22;
float basesetpoint = -2.8;
float setpoint = basesetpoint;
int manualSpeed = 90;

float error, lastError, integral, derivative, output;

// --- IR Direction Control ---
bool rotateLeft = false;
bool rotateRight = false;
bool isManualSetpoint = false;

// --- Serial Input Variables ---
String cmdstring;
String cmdcode;
String parmstring;
bool newcmd = false;
int sepIndex;
bool noparm = false;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu6050.begin();
  mpu6050.setGyroOffsets(-2.83, -1.95, 0.14);
  irrecv.enableIRIn();

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void loop() {
  mpu6050.update();
  float pitch = mpu6050.getAngleY();

  // --- IR Control ---
  if (rotateLeft) {
    motorcontrolManual(manualSpeed, manualSpeed - 30);
  }
  else if (rotateRight) {
    motorcontrolManual(manualSpeed - 30, manualSpeed);
  }
  else {
    // ปกติ → ให้ PID บาลานซ์
    pidControl(pitch);
  }

  Serial.print("Rotate Left: "); Serial.print(rotateLeft);
  Serial.print(" | Rotate Right: "); Serial.println(rotateRight);

  // --- Serial Monitor Output ---
  Serial.print("Error: "); Serial.print(error);
  Serial.print("\tOutput: "); Serial.print(output);
  Serial.print("\tSetpoint: "); Serial.println(setpoint);

  // --- Serial Input ---
  if (Serial.available() > 0) {
    cmdstring = Serial.readString();
    newcmd = true;
  }
  if (newcmd) {
    callcmd();
    newcmd = false;
  }

  // --- IR Decode ---
  if (irrecv.decode(&results)) {
    handleIR(results.value);
    irrecv.resume(); // Ready for next IR command
  }

  delay(10);
}

void pidControl(float pitch) {
  error = setpoint - pitch;
  integral += error;
  integral = constrain(integral, -300, 300);
  derivative = error - lastError;
  output = kp * error + ki * integral + kd * derivative;

  // Dead zone + reset I term
  if (abs(error) < 0.3 && abs(output) < 20) {
    output = 0;
    integral = 0;
  }

  lastError = error;
  motorcontrol(output);
}

void motorcontrol(float output) {
  int motorSpeed = constrain(output, -153, 153);

  // Motor A
  analogWrite(ENA, abs(motorSpeed));
  digitalWrite(IN1, motorSpeed > 0 ? HIGH : LOW);
  digitalWrite(IN2, motorSpeed > 0 ? LOW : HIGH);

  // Motor B
  analogWrite(ENB, abs(motorSpeed));
  digitalWrite(IN3, motorSpeed > 0 ? HIGH : LOW);
  digitalWrite(IN4, motorSpeed > 0 ? LOW : HIGH);
}

void motorcontrolManual(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  analogWrite(ENA, abs(leftSpeed));
  digitalWrite(IN1, leftSpeed > 0 ? HIGH : LOW);
  digitalWrite(IN2, leftSpeed > 0 ? LOW : HIGH);

  analogWrite(ENB, abs(rightSpeed));
  digitalWrite(IN3, rightSpeed > 0 ? HIGH : LOW);
  digitalWrite(IN4, rightSpeed > 0 ? LOW : HIGH);
}

void handleIR(unsigned long code) {
  Serial.print("IR Code: "); Serial.println(code, HEX);

  if (code == 0xFF10EF) {      // Rotate Left
    rotateLeft = true;
    rotateRight = false;
  }
  else if (code == 0xFF5AA5) { // Rotate Right
    rotateRight = true;
    rotateLeft = false;
  }
  else if (code == 0xFF38C7) { // Stop
    rotateLeft = false;
    rotateRight = false;
    manualSpeed = 100;
  }
  else if (code == 0xFF18E7) { // Speed Up
    manualSpeed = min(manualSpeed + 10, 255);
    Serial.println("Speed++");
  }
  else if (code == 0xFF4AB5) { // Speed Down
    manualSpeed = max(manualSpeed - 10, 0);
    Serial.println("Speed--");
  }
}

void callcmd() {
  cmdstring.trim();
  sepIndex = cmdstring.indexOf('=');

  if (sepIndex == -1) {
    cmdcode = cmdstring;
    noparm = true;
  } else {
    cmdcode = cmdstring.substring(0, sepIndex);
    cmdcode.trim();
    parmstring = cmdstring.substring(sepIndex + 1);
    parmstring.trim();
    noparm = false;
  }

  if (cmdcode.equalsIgnoreCase("kp") && !noparm) kp = parmstring.toFloat();
  else if (cmdcode.equalsIgnoreCase("ki") && !noparm) ki = parmstring.toFloat();
  else if (cmdcode.equalsIgnoreCase("kd") && !noparm) kd = parmstring.toFloat();
}