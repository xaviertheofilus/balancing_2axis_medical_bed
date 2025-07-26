#include "motor_control.h"
#include <Arduino.h>

// For ESP32 - include the LEDC library for PWM functions
#ifdef ESP32
  #include "driver/ledc.h"
  #include "esp32-hal-ledc.h"
#endif

// Motor control variables
bool motorsInitialized = false;

// Motor speed tracking
int currentMotorXSpeed = 0;
int currentMotorYSpeed = 0;

// Use PWM settings from config.h
// PWM channels and settings are defined in config.h

void setupMotors() {
    if (motorsInitialized) return;
    
    // Initialize PWM channels for ESP32 using config.h definitions
    #ifdef ESP32
        // Setup PWM for Motor X - NEW ESP32 3.x syntax
        ledcAttach(MOTOR_X_PWM, pwmFreq, pwmResolution);
        
        // Setup PWM for Motor Y - NEW ESP32 3.x syntax
        ledcAttach(MOTOR_Y_PWM, pwmFreq, pwmResolution);
    #else
        // For other Arduino boards, use analogWrite
        pinMode(MOTOR_X_PWM, OUTPUT);
        pinMode(MOTOR_Y_PWM, OUTPUT);
    #endif
    
    // Setup direction pins
    pinMode(MOTOR_X_DIR1, OUTPUT);
    pinMode(MOTOR_X_DIR2, OUTPUT);
    pinMode(MOTOR_Y_DIR1, OUTPUT);
    pinMode(MOTOR_Y_DIR2, OUTPUT);

    pinMode(LIMIT_X_PIN, INPUT_PULLUP);
    pinMode(LIMIT_Y_PIN, INPUT_PULLUP);
    
    // Initialize motors to stopped state
    stopMotors();
    motorsInitialized = true;
    
    Serial.println("Motors initialized successfully");
}

void moveMotorX(int speed) {
    if (!motorsInitialized) {
        setupMotors();
    }
    
    // Set direction
    setMotorXDirection(speed >= 0);
    
    // Set PWM value
    int pwmValue = constrainPWM(abs(speed));
    setPWMX(pwmValue);
    
    currentMotorXSpeed = speed;
}

void moveMotorY(int speed) {
    if (!motorsInitialized) {
        setupMotors();
    }
    
    // Set direction
    setMotorYDirection(speed >= 0);
    
    // Set PWM value
    int pwmValue = constrainPWM(abs(speed));
    setPWMY(pwmValue);
    
    currentMotorYSpeed = speed;
}

void stopMotors() {
    stopMotorX();
    stopMotorY();
}

void stopMotorX() {
    setPWMX(0);
    currentMotorXSpeed = 0;
}

void stopMotorY() {
    setPWMY(0);
    currentMotorYSpeed = 0;
}

void executeMotorControl() {
    // Convert roll and pitch PWM values to motor speeds
    int motorXSpeed = (int)rollPWM;
    int motorYSpeed = (int)pitchPWM;
    
    setMotorSpeeds(motorXSpeed, motorYSpeed);
}

void setMotorSpeeds(int speedX, int speedY) {
    moveMotorX(speedX);
    moveMotorY(speedY);
}

int getMotorXSpeed() {
    return currentMotorXSpeed;
}

int getMotorYSpeed() {
    return currentMotorYSpeed;
}

bool areMotorsRunning() {
    return (currentMotorXSpeed != 0 || currentMotorYSpeed != 0);
}

void setPWMX(int pwmValue) {
    pwmValue = constrainPWM(pwmValue);
    
    #ifdef ESP32
        // NEW ESP32 3.x syntax - write directly to pin
        ledcWrite(MOTOR_X_PWM, pwmValue);
    #else
        analogWrite(MOTOR_X_PWM, pwmValue);
    #endif
}

void setPWMY(int pwmValue) {
    pwmValue = constrainPWM(pwmValue);
    
    #ifdef ESP32
        // NEW ESP32 3.x syntax - write directly to pin
        ledcWrite(MOTOR_Y_PWM, pwmValue);
    #else
        analogWrite(MOTOR_Y_PWM, pwmValue);
    #endif
}

int constrainPWM(int pwmValue) {
    return constrain(pwmValue, (int)MIN_PWM, (int)MAX_PWM);
}

void setMotorXDirection(bool forward) {
    // Using H-bridge with DIR1 and DIR2 pins
    if (forward) {
        digitalWrite(MOTOR_X_DIR1, HIGH);
        digitalWrite(MOTOR_X_DIR2, LOW);
    } else {
        digitalWrite(MOTOR_X_DIR1, LOW);
        digitalWrite(MOTOR_X_DIR2, HIGH);
    }
}

void setMotorYDirection(bool forward) {
    // Using H-bridge with DIR1 and DIR2 pins
    if (forward) {
        digitalWrite(MOTOR_Y_DIR1, HIGH);
        digitalWrite(MOTOR_Y_DIR2, LOW);
    } else {
        digitalWrite(MOTOR_Y_DIR1, LOW);
        digitalWrite(MOTOR_Y_DIR2, HIGH);
    }
}

void printMotorStatus() {
    Serial.println("=== Motor Status ===");
    Serial.print("Motors Initialized: ");
    Serial.println(motorsInitialized ? "Yes" : "No");
    Serial.print("Motor X Speed: ");
    Serial.println(currentMotorXSpeed);
    Serial.print("Motor Y Speed: ");
    Serial.println(currentMotorYSpeed);
    Serial.print("Motors Running: ");
    Serial.println(areMotorsRunning() ? "Yes" : "No");
    Serial.print("Roll PWM: ");
    Serial.println(rollPWM);
    Serial.print("Pitch PWM: ");
    Serial.println(pitchPWM);
    Serial.println("==================");
}

void resetMotorControllers() {
    stopMotors();
    motorsInitialized = false;
    rollPWM = 0;
    pitchPWM = 0;
    currentMotorXSpeed = 0;
    currentMotorYSpeed = 0;
    Serial.println("Motor controllers reset");
}