#ifndef IMU_HANDLER_H
#define IMU_HANDLER_H

#include <Wire.h>
#include <Arduino.h>
#include <config.h>

// Alamat I2C MPU6050
#define MPU6050_ADDR 0x68

// Register MPU6050
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43
#define ACCEL_CONFIG 0x1C
#define GYRO_CONFIG 0x1B
#define SMPLRT_DIV 0x19
#define CONFIG 0x1A

// Constants
#define PI 3.14159265359

const float rollO = 0.0; 
const float pitchO = 0.0;

// Error handling constants
const float NOISE_THRESHOLD = 0.15;
const float GYRO_DEADBAND = 0.03;
const int MAX_I2C_RETRIES = 3;
const unsigned long I2C_RECOVERY_INTERVAL = 1500;
const float MOTION_THRESHOLD = 30.0;
const unsigned long MOTION_TIMEOUT = 800;

// Filter constants
const int FILTER_SIZE = 3;
const float ALPHA = 0.96;

// Function declarations
bool initIMU();
bool readIMUData();
void calibrateOffsets();
// void calibrateActualOffset();
void resetIMUState();

// Getter functions
float getCurrentRoll();
float getCurrentPitch();
bool isIMUReady();
bool isIMUCalibrated();
bool isI2CHealthy();
bool isInFastMotionState();
float getCurrentRollVelocity();
float getCurrentPitchVelocity();

// Utility functions
void handleI2CError();
void addToMovingAverage(float roll, float pitch);
bool safeI2CRead(uint8_t address, uint8_t regAddress, uint8_t* data, uint8_t length);
bool safeI2CWrite(uint8_t address, uint8_t regAddress, uint8_t data);

#endif