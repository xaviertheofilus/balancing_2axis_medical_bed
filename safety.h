#ifndef SAFETY_H
#define SAFETY_H

#include "Arduino.h"

// Safety status codes
#define SAFETY_OK 0
#define SAFETY_LIMIT_PRESSED 1
#define SAFETY_OUT_OF_RANGE 2

// Define mutex for thread-safe printing
static portMUX_TYPE printMutex = portMUX_INITIALIZER_UNLOCKED;

// External variable declarations
extern bool systemReady;
extern bool balancingActive;
extern unsigned long lastPrint;

// Function declarations
int safetyCheck();
const char* getSafetyStatusString(int status);;
bool isLimitXPressed();
bool isLimitYPressed();

// Helper functions for encoder access (declared elsewhere but used here)
extern long getEncoderX();
extern long getEncoderY();

// Motor control functions (declared elsewhere but used here)
extern void moveMotorX(int speed);
extern void moveMotorY(int speed);

extern float getCurrentPitch();
extern float getCurrentRoll();

extern void setBalancingActive(bool active);

#endif // SAFETY_H