#ifndef ENCODERS_H
#define ENCODERS_H

#include <Arduino.h>
#include "config.h"

// External variables (defined in encoders.cpp)
extern volatile long encoderX_count;
extern volatile long encoderY_count;
extern portMUX_TYPE encoderMux;

// Function declarations
void setupEncoders();
void IRAM_ATTR encoderX_ISR();
void IRAM_ATTR encoderY_ISR();
long getEncoderX();
long getEncoderY();
void resetEncoderX();
void resetEncoderY();

// Utility functions
float getEncoderXRevolutions();
float getEncoderYRevolutions();
bool isEncoderXInRange();
bool isEncoderYInRange();

#endif // ENCODERS_H