
#include <Arduino.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// ======================================================
// 1. HARDWARE CONFIGURATION
// ======================================================

const int ENA = 6;
const int IN1 = 8;
const int IN2 = 9;

const int ENCODER_A = 2;
const int ENCODER_B = 3;

// ======================================================
// 2. MOTOR AND CONTROL PARAMETERS
// ======================================================

const float PULSES_PER_REV = 496.0; // Empirically measured
const unsigned long SAMPLE_TIME_MS = 50; // 20 Hz

// Adjust this limit to your motor's safe operating range
const float MAX_TARGET_RPM = 170.0;

// Original PID gains
float Kp = 2.0;
float Ki = 1.1;
float Kd = 0.05;

// ======================================================
// 3. ENCODER AND PID STATE
// ======================================================

volatile long encoderTicks = 0;
long lastEncoderTicks = 0;

unsigned long lastTime = 0;

float targetRPM = 0.0;
float actualRPM = 0.0;

float error = 0.0;
float integralSum = 0.0;
float derivative = 0.0;
float lastRPM = 0.0;

float controlSignal = 0.0;
int appliedPWM = 0;

bool motorEnabled = false;

// Serial command buffer
char commandBuffer[32];
byte commandIndex = 0;

// ======================================================
// 4. ENCODER INTERRUPT
// ======================================================

void readEncoder() {
  if (digitalRead(ENCODER_B) == HIGH) {
    encoderTicks++;
  } else {
    encoderTicks--;
  }
}

// ======================================================
// 5. MOTOR DRIVER
// ======================================================

void driveMotor(float pwmCommand) {

  // This controller regulates forward speed only.
  int pwm = constrain((int)pwmCommand, 0, 255);

  if (!motorEnabled || targetRPM <= 0.0) {
    pwm = 0;
  }

  appliedPWM = pwm;

  if (pwm > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }

  analogWrite(ENA, pwm);
}

// ======================================================
// 6. MOTOR STOP / RESET
// ======================================================

void stopMotor() {

  motorEnabled = false;
  targetRPM = 0.0;

  integralSum = 0.0;
  error = 0.0;
  derivative = 0.0;
  controlSignal = 0.0;

  driveMotor(0);
}

// ======================================================
// 7. SERIAL COMMAND PROCESSING
// ======================================================

void processCommand(char* command) {

  // SET,<RPM>  example: SET,100
  if (strncmp(command, "SET,", 4) == 0) {

    char* endPointer;
    double requestedRPM = strtod(command + 4, &endPointer);

    // Ignore trailing spaces
    while (*endPointer == ' ') {
      endPointer++;
    }

    if (endPointer == command + 4 ||
        *endPointer != '\0' ||
        isnan(requestedRPM) ||
        isinf(requestedRPM) ||
        requestedRPM < 0.0 ||
        requestedRPM > MAX_TARGET_RPM) {

      Serial.println("ERR,INVALID_RPM");
      return;
    }

    if (requestedRPM == 0.0) {
      stopMotor();
    } else {
      targetRPM = (float)requestedRPM;
      motorEnabled = true;
    }

    Serial.print("ACK,SET,");
    Serial.println(targetRPM, 2);
  }

  // STOP
  else if (strcmp(command, "STOP") == 0) {

    stopMotor();
    Serial.println("ACK,STOP");
  }

  // Unknown command
  else {
    Serial.println("ERR,UNKNOWN_COMMAND");
  }
}

void readSerialCommands() {

  while (Serial.available() > 0) {

    char incoming = Serial.read();

    if (incoming == '\n') {

      commandBuffer[commandIndex] = '\0';

      if (commandIndex > 0) {
        processCommand(commandBuffer);
      }

      commandIndex = 0;
    }

    else if (incoming != '\r') {

      if (commandIndex < sizeof(commandBuffer) - 1) {
        commandBuffer[commandIndex++] = incoming;
      } else {
        // Discard oversized command
        commandIndex = 0;
        Serial.println("ERR,COMMAND_TOO_LONG");

        // Flush remaining serial input
        while (Serial.available() > 0) {
          Serial.read();
        }
      }
    }
  }
}

// ======================================================
// 8. PID CONTROLLER
// ======================================================

void updatePID(float dtSeconds) {

  error = targetRPM - actualRPM;

  if (!motorEnabled) {
    controlSignal = 0.0;
    integralSum = 0.0;
    derivative = 0.0;

    driveMotor(0);
    lastRPM = actualRPM;
    return;
  }

  // Derivative on measurement prevents setpoint kick
  derivative = -(actualRPM - lastRPM) / dtSeconds;

  // Candidate integral value
  float proposedIntegral = integralSum + error * dtSeconds;

  // Compute preliminary unsaturated output
  float proposedOutput =
      Kp * error +
      Ki * proposedIntegral +
      Kd * derivative;

  // Conditional integration anti-windup:
  // Reject integration if it drives output further
  // into upper or lower PWM saturation.
  bool upperSaturation =
      (proposedOutput > 255.0 && error > 0.0);

  bool lowerSaturation =
      (proposedOutput < 0.0 && error < 0.0);

  if (!upperSaturation && !lowerSaturation) {
    integralSum = proposedIntegral;
  }

  // Final PID calculation (unclamped)
  controlSignal =
      Kp * error +
      Ki * integralSum +
      Kd * derivative;

  // driveMotor applies PWM saturation: 0 to 255
  driveMotor(controlSignal);

  lastRPM = actualRPM;
}

// ======================================================
// 9. TELEMETRY OUTPUT
// ======================================================

void sendTelemetry(unsigned long timestamp) {

  // Format:
  // DATA,time_ms,target_rpm,actual_rpm,pwm,error,pid_output

  Serial.print("DATA,");
  Serial.print(timestamp);
  Serial.print(",");

  Serial.print(targetRPM, 2);
  Serial.print(",");

  Serial.print(actualRPM, 2);
  Serial.print(",");

  Serial.print(appliedPWM);
  Serial.print(",");

  Serial.print(error, 2);
  Serial.print(",");

  Serial.println(controlSignal, 2);
}

// ======================================================
// 10. SETUP
// ======================================================

void setup() {

  Serial.begin(115200);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);

  attachInterrupt(
    digitalPinToInterrupt(ENCODER_A),
    readEncoder,
    RISING
  );

  stopMotor();

  lastTime = millis();

  Serial.println("READY,MOTOR_PID_V2");

  Serial.println(
    "HEADER,time_ms,target_rpm,actual_rpm,pwm,error,pid_output"
  );
}

// ======================================================
// 11. MAIN CONTROL LOOP
// ======================================================

void loop() {

  // Process commands sent by Python
  readSerialCommands();

  unsigned long currentTime = millis();
  unsigned long timeChange = currentTime - lastTime;

  if (timeChange >= SAMPLE_TIME_MS) {

    float dtSeconds = timeChange / 1000.0;

    // Atomic encoder read (32-bit counter on 8-bit AVR)
    noInterrupts();
    long currentTicks = encoderTicks;
    interrupts();

    long deltaTicks = currentTicks - lastEncoderTicks;
    lastEncoderTicks = currentTicks;

    // Convert encoder pulses to RPM
    actualRPM =
        (deltaTicks / PULSES_PER_REV) *
        (60.0 / dtSeconds);

    // Run PID update
    updatePID(dtSeconds);

    // Send measurement data to Python
    sendTelemetry(currentTime);

    lastTime = currentTime;
  }
}
