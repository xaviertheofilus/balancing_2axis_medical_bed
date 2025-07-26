#include "safety.h"
#include "config.h"
#include "motor_control.h"

// SINGLE CORE SAFETY CHECK FUNCTION - No mutex or critical sections needed
int safetyCheck() {
  if (!systemReady) return SAFETY_OK;
  
  long posX = getEncoderX();
  long posY = getEncoderY();
  bool limitXPressed = isLimitXPressed();
  bool limitYPressed = isLimitYPressed();
  
  // Safety flags
  bool xOutOfRange = (posX < 0 || posX > MAX_POSITION_PULSES);
  bool yOutOfRange = (posY < 0 || posY > MAX_POSITION_PULSES);
  
  if (!balancingActive) {
    return SAFETY_OK; // No safety action needed when not balancing
  }
  
  // Handle X-axis safety
  if (limitXPressed) {
    // Only allow movement away from limit
    if (rollPWM > 0) { // Moving away from limit is OK
      moveMotorX(rollPWM);
    } else { // Don't move toward limit
      moveMotorX(0);
    }
  } else if (xOutOfRange) {
    if (posX <= 0 && rollPWM < 0) {
      // At minimum position, don't move further negative
      moveMotorX(0);
    } else if (posX >= MAX_POSITION_PULSES && rollPWM > 0) {
      // At maximum position, don't move further positive
      moveMotorX(0);
    } else {
      // Normal operation
      moveMotorX(rollPWM);
    }
  }
  
  // Handle Y-axis safety  
  if (limitYPressed) {
    // Only allow movement away from limit
    if (pitchPWM > 0) { // Moving away from limit is OK
      moveMotorY(pitchPWM);
    } else { // Don't move toward limit
      moveMotorY(0);
    }
  } else if (yOutOfRange) {
    if (posY <= 0 && pitchPWM < 0) {
      // At minimum position, don't move further negative
      moveMotorY(0);
    } else if (posY >= MAX_POSITION_PULSES && pitchPWM > 0) {
      // At maximum position, don't move further positive  
      moveMotorY(0);
    } else {
      // Normal operation
      moveMotorY(pitchPWM);
    }
  }
  
  // Return safety status
  if (limitXPressed || limitYPressed) {
    return SAFETY_LIMIT_PRESSED; // Limit switch pressed
  } else if (xOutOfRange || yOutOfRange) {
    return SAFETY_OUT_OF_RANGE; // Motor out of range
  } else {
    return SAFETY_OK; // All OK
  }
}

// Emergency stop function - immediately stops all motors
void emergencyStop() {
  moveMotorX(0);
  moveMotorY(0);
  setBalancingActive(false);
  Serial.println(">>> EMERGENCY STOP ACTIVATED <<<");
}

// Check if system is in safe state for operation
bool isSystemSafe() {
  int status = safetyCheck();
  return status;
}

// Helper functions for limit switches
bool isLimitXPressed() {
  return digitalRead(LIMIT_X_PIN) == LOW;
}

bool isLimitYPressed() {
  return digitalRead(LIMIT_Y_PIN) == LOW;
}

// Get safety status as string for debugging
const char* getSafetyStatusString(int status) {
  switch (status) {
    case SAFETY_OK:
      return "OK";
    case SAFETY_LIMIT_PRESSED:
      return "LIMIT_PRESSED";
    case SAFETY_OUT_OF_RANGE:
      return "OUT_OF_RANGE";
    default:
      return "UNKNOWN";
  }
}