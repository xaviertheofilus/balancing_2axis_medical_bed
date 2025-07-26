#ifndef BALANCE_CONTROL_H
#define BALANCE_CONTROL_H

#include <Arduino.h>

// External dependencies
extern bool balancingActive;
extern bool systemReady;

// PID parameters - can be tuned externally
extern float rollKp, rollKi, rollKd;
extern float pitchKp, pitchKi, pitchKd;

// PID variables
extern float rollError, rollLastError, rollIntegral;
extern float pitchError, pitchLastError, pitchIntegral;

// Function declarations
void initBalanceControl();
void calculateBalanceControl();
void startBalancing();
void stopBalancing();
void resetPIDVariables();
void setTargetAngles(float roll, float pitch);
void setPIDParameters(float kp_roll, float ki_roll, float kd_roll, 
                     float kp_pitch, float ki_pitch, float kd_pitch);

#endif // BALANCE_CONTROL_H