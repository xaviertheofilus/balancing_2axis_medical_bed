#include "homing.h"
#include "config.h"
#include "motor_control.h"
#include "encoders.h"
#include "imu_handler.h"
#include "safety.h"

bool motorX_homed = false;
bool motorY_homed = false;

void startHoming() {
  homingInProgress = true;
  motorX_homed = false;
  motorY_homed = false;
  
  Serial.println(F("Starting homing process..."));
  
  // Start moving backward to find home position
  if (!isLimitXPressed() || motorX_homed) {
    moveMotorX(-110); // Move backward slowly
  } else {
    motorX_homed = true;
    resetEncoderX();
    moveMotorX(0);
  }
  
  if (!isLimitYPressed() || motorY_homed) {
    moveMotorY(-110); // Move backward slowly
  } else {
    motorY_homed = true;
    resetEncoderY();
    moveMotorY(0);
  }
}

void processHoming() {
  // Check motor X
  if (isLimitXPressed() || motorX_homed) {
    moveMotorX(0);
    resetEncoderX();
    motorX_homed = true;
    Serial.println(F("Motor X homed"));
  }
  
  // Check motor Y
  if (isLimitYPressed() || motorY_homed) {
    moveMotorY(0);
    resetEncoderY();
    motorY_homed = true;
    Serial.println(F("Motor Y homed"));
  }
  
  // If both motors are homed
  if (motorX_homed && motorY_homed) {
    homingInProgress = false;
    stopMotors();
    
    // Move to center position
    Serial.println(F("Moving to center position..."));
    moveToCenter();
    
    delay(2000); // Wait for movement to complete
    
    // Calibrate IMU offsets
    calibrateOffsets();
    
    systemReady = true;
    Serial.println(F("System Ready!"));
    Serial.println(F("Commands: 's'=start, 't'=stop, 'c'=calibrate, 'h'=home"));
  }
}

void moveToCenter() {
  Serial.println(F("Moving to center position..."));
  
  // Move both motors to center (9 revolutions from home)
  long targetX = (long)(10 * PULSES_PER_REV);
  long targetY = (long)(10 * PULSES_PER_REV);
  
  const int MIN_SPEED = 115;  // Minimum effective speed
  const int MAX_SPEED = 255; // Maximum safe speed for positioning
  
  while (abs(getEncoderX() - targetX) > 50 || abs(getEncoderY() - targetY) > 50) {
    long currentX = getEncoderX();
    long currentY = getEncoderY();
    long errorX = targetX - currentX;
    long errorY = targetY - currentY;
    
    // Calculate speeds with proper scaling
    int speedX = constrain(errorX / 25, -MAX_SPEED, MAX_SPEED);
    int speedY = constrain(errorY / 25, -MAX_SPEED, MAX_SPEED);
    
    // Apply minimum speed threshold
    if (speedX > 0 && speedX < MIN_SPEED) speedX = MIN_SPEED;
    else if (speedX < 0 && speedX > -MIN_SPEED) speedX = -MIN_SPEED;
    
    if (speedY > 0 && speedY < MIN_SPEED) speedY = MIN_SPEED;
    else if (speedY < 0 && speedY > -MIN_SPEED) speedY = -MIN_SPEED;
    
    // Stop if very close to target
    if (abs(errorX) < 50) speedX = 0;
    if (abs(errorY) < 50) speedY = 0;
    
    moveMotorX(speedX);
    moveMotorY(speedY);
    
    // Debug output (reduced frequency)
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 100) {
      Serial.print(F("Target: ")); Serial.print(targetX);
      Serial.print(F(" Current X: ")); Serial.print(currentX);
      Serial.print(F(" Y: ")); Serial.print(currentY);
      Serial.print(F(" Error X: ")); Serial.print(errorX);
      Serial.print(F(" Y: ")); Serial.print(errorY);
      Serial.print(F(" Speed X: ")); Serial.print(speedX);
      Serial.print(F(" Y: ")); Serial.println(speedY);
      lastPrint = millis();
    }
    
    delay(20);
    
    // // Safety timeout
    // static unsigned long startTime = millis();
    // if (millis() - startTime > 30000) { // 30 second timeout
    //   Serial.println(F("Center movement timeout!"));
    //   break;
    // }
  }
  
  stopMotors();
  Serial.println(F("Center position reached"));
}