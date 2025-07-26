#ifndef HOMING_H
#define HOMING_H

#include <Arduino.h>

// External dependencies
extern bool homingInProgress;
extern bool systemReady;

// Function declarations
void startHoming();
void processHoming();
void moveToCenter();
bool isLimitXPressed();
bool isLimitYPressed();

#endif // HOMING_H