#include "balance_control.h"
#include "config.h"
#include "motor_control.h"

// Motor speed
int rollPWM = 0;
int pitchPWM = 0;

float rollKp = DEFAULT_ROLL_KP, rollKi = DEFAULT_ROLL_KI, rollKd = DEFAULT_ROLL_KD;
float pitchKp = DEFAULT_PITCH_KP, pitchKi = DEFAULT_PITCH_KI, pitchKd = DEFAULT_PITCH_KD;

// PID variables
float rollError = 0.0, rollLastError = 0.0, rollIntegral = 0.0;
float pitchError = 0.0, pitchLastError = 0.0, pitchIntegral = 0.0;

void initBalanceControl() {
  // Initialize all PID variables to zero
  resetPIDVariables();
  
  // Set default target angles (level platform)
  targetRoll = 0.0;
  targetPitch = 0.0;
  
  Serial.println(F("Balance control initialized"));
  Serial.print(F("Roll PID: Kp="));
  Serial.print(rollKp);
  Serial.print(F(" Ki="));
  Serial.print(rollKi);
  Serial.print(F(" Kd="));
  Serial.println(rollKd);
  
  Serial.print(F("Pitch PID: Kp="));
  Serial.print(pitchKp);
  Serial.print(F(" Ki="));
  Serial.print(pitchKi);
  Serial.print(F(" Kd="));
  Serial.println(pitchKd);
}

void calculateBalanceControl() {
  // PID calculation for Roll axis
  rollError = targetRoll - currentRoll;
  
  // Integral term with windup protection
  rollIntegral += rollError;
  rollIntegral = constrain(rollIntegral, -20.0, 20.0);
  
  // Derivative term
  float rollDerivative = rollError - rollLastError;
  
  // Calculate PID output
  rollPWM = -(rollKp * rollError + rollKi * rollIntegral + rollKd * rollDerivative);
  rollPWM = constrain(rollPWM, -MAX_PWM, MAX_PWM);
  
  // Store error for next iteration
  rollLastError = rollError;
  
  // PID calculation for Pitch axis
  pitchError = targetPitch - currentPitch;
  
  // Integral term with windup protection
  pitchIntegral += pitchError;
  pitchIntegral = constrain(pitchIntegral, -20.0, 20.0);
  
  // Derivative term
  float pitchDerivative = pitchError - pitchLastError;
  
  // Calculate PID output
  pitchPWM = -(pitchKp * pitchError + pitchKi * pitchIntegral + pitchKd * pitchDerivative);
  pitchPWM = constrain(pitchPWM, -MAX_PWM, MAX_PWM);
  
  // Store error for next iteration
  pitchLastError = pitchError;
}

void startBalancing() {
  if (!systemReady) {
    Serial.println(F("System not ready for balancing"));
    return;
  }
  
  Serial.println(F("Starting balancing..."));
  
  // Reset all PID variables for clean start
  resetPIDVariables();
  
  // Enable balancing
  balancingActive = true;
  
  Serial.println(F("Balancing active!"));
  Serial.print(F("Target Roll: "));
  Serial.print(targetRoll, 2);
  Serial.print(F("° Target Pitch: "));
  Serial.print(targetPitch, 2);
  Serial.println(F("°"));
}

void stopBalancing() {
  Serial.println(F("Stopping balancing..."));
  
  // Disable balancing
  balancingActive = false;
  
  // Stop all motors immediately
  stopMotors();
  
  // Reset PID variables
  resetPIDVariables();
  
  Serial.println(F("Balancing stopped"));
}

void resetPIDVariables() {
  // Reset Roll PID variables
  rollError = 0.0;
  rollLastError = 0.0;
  rollIntegral = 0.0;
  rollPWM = 0.0;
  
  // Reset Pitch PID variables
  pitchError = 0.0;
  pitchLastError = 0.0;
  pitchIntegral = 0.0;
  pitchPWM = 0.0;
  
  Serial.println(F("PID variables reset"));
}

void setTargetAngles(float roll, float pitch) {
  // Constrain target angles to safe limits
  targetRoll = constrain(roll, -30.0, 30.0);
  targetPitch = constrain(pitch, -30.0, 30.0);
  
  Serial.print(F("Target angles set - Roll: "));
  Serial.print(targetRoll, 2);
  Serial.print(F("° Pitch: "));
  Serial.print(targetPitch, 2);
  Serial.println(F("°"));
  
  // If balancing is active, reset integral terms to prevent windup
  if (balancingActive) {
    rollIntegral = 0.0;
    pitchIntegral = 0.0;
  }
}

void setPIDParameters(float kp_roll, float ki_roll, float kd_roll, 
                     float kp_pitch, float ki_pitch, float kd_pitch) {
  // Update Roll PID parameters
  rollKp = constrain(kp_roll, 0.0, 100.0);
  rollKi = constrain(ki_roll, 0.0, 10.0);
  rollKd = constrain(kd_roll, 0.0, 50.0);
  
  // Update Pitch PID parameters
  pitchKp = constrain(kp_pitch, 0.0, 100.0);
  pitchKi = constrain(ki_pitch, 0.0, 10.0);
  pitchKd = constrain(kd_pitch, 0.0, 50.0);
  
  Serial.println(F("PID parameters updated"));
  Serial.print(F("Roll PID: Kp="));
  Serial.print(rollKp, 2);
  Serial.print(F(" Ki="));
  Serial.print(rollKi, 2);
  Serial.print(F(" Kd="));
  Serial.println(rollKd, 2);
  
  Serial.print(F("Pitch PID: Kp="));
  Serial.print(pitchKp, 2);
  Serial.print(F(" Ki="));
  Serial.print(pitchKi, 2);
  Serial.print(F(" Kd="));
  Serial.println(pitchKd, 2);
  
  // If balancing is active, reset integral terms to prevent issues
  if (balancingActive) {
    rollIntegral = 0.0;
    pitchIntegral = 0.0;
    Serial.println(F("Integral terms reset due to parameter change"));
  }
}