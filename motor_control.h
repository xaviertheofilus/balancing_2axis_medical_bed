#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include "config.h"


// Motor state tracking
extern bool motorsInitialized;

// Function declarations for motor setup
void setupMotors();

// Basic motor movement functions
void moveMotorX(int speed);
void moveMotorY(int speed);
void stopMotors();
void stopMotorX();
void stopMotorY();

// Advanced motor control functions
void executeMotorControl();
void setMotorSpeeds(int speedX, int speedY);

// Motor status functions
int getMotorXSpeed();
int getMotorYSpeed();
bool areMotorsRunning();

// PWM control functions
void setPWMX(int pwmValue);
void setPWMY(int pwmValue);
int constrainPWM(int pwmValue);

// Motor direction control
void setMotorXDirection(bool forward);
void setMotorYDirection(bool forward);

// Utility functions
void printMotorStatus();
void resetMotorControllers();

#endif // MOTOR_CONTROL_H