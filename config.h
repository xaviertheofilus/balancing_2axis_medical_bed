#ifndef CONFIG_H
#define CONFIG_H

#include "Arduino.h"

// ====================================================================
// HARDWARE CONFIGURATION - ESP32-S3-N16R8
// ====================================================================

// ===== MOTOR CONTROL PINS =====
// Motor X (Roll control) - Using safe GPIO pins
#define MOTOR_X_PWM  3     // Changed from 1 (strapping pin)
#define MOTOR_X_DIR1 5     // Changed from 2 
#define MOTOR_X_DIR2 4     // Changed from 42

// Motor Y (Pitch control) - Using safe GPIO pins  
#define MOTOR_Y_PWM  6     // Changed from 3 (strapping pin)
#define MOTOR_Y_DIR1 15    // Changed from 46 (strapping pin)
#define MOTOR_Y_DIR2 7    // Changed from 9

// ===== ENCODER PINS =====
// Using interrupt-capable pins (all ESP32-S3 GPIO pins support interrupts)
#define ENCODER_X_A  38    // Changed from 4
#define ENCODER_X_B  39    // Changed from 5
#define ENCODER_Y_A   16   // Changed from 6
#define ENCODER_Y_B   17   // Changed from 7

// ===== LIMIT SWITCH PINS =====
#define LIMIT_X_PIN  12    // Changed from 10
#define LIMIT_Y_PIN  13    // Changed from 11

// ===== I2C PINS =====
// Using default I2C pins for ESP32-S3 (better hardware support)
#define SDA_PIN  8         // Changed from 8 (standard I2C SDA)
#define SCL_PIN  9         // Changed from 18 (standard I2C SCL)

// Control loop intervals (in milliseconds)
//const unsigned long BALANCE_INTERVAL =10;    // 50Hz for balance control //freq perhitungan PID 100Hz
//const unsigned long STATUS_INTERVAL = 100;    // 5Hz for status printing //sampling data //s 10Hz
//const unsigned long SAFETY_INTERVAL = 100;    // 10Hz for safety checks

// ====================================================================
// PWM CONFIGURATION
// ====================================================================
const int pwmFreq = 1000;      // PWM frequency (Hz)
const int pwmResolution = 8;   // PWM resolution (8-bit = 0-255)
const int pwmChannelX = 0;     // PWM channel for motor X
const int pwmChannelY = 1;     // PWM channel for motor Y

// Motor control limits
const float MAX_PWM = 250.0;   // Maximum safe PWM value
const float MIN_PWM = 0.0;    // Minimum effective PWM value

// ====================================================================
// ENCODER CONFIGURATION
// ====================================================================
const int PULSES_PER_REV = 396;                    // Encoder pulses per revolution
const int MAX_POSITION_REV = 20;                   // Maximum 20 revolutions from home
const long MAX_POSITION_PULSES = MAX_POSITION_REV * PULSES_PER_REV;  // Maximum position in pulses

// ====================================================================
// PID CONTROL PARAMETERS
// ====================================================================
// PID gains for Roll control (X-axis)
const float DEFAULT_ROLL_KP = 42.5;//10;// 45;//42.5; //31.5; //25;   //31.5;        //30;//35;//70.0;//65.0; //53.5  // respon awal 10, 25, 45 FINAL: Roll: kp 42,5; ki 0,72; kd 45,5 
const float DEFAULT_ROLL_KI = 0.72;//0.72; //1.05; // 0;   //1.05;        //1.0;//1.2;//2.2; //1.65                                          Pitch:kp 40,5; ki 0,055;kd 42,5  
const float DEFAULT_ROLL_KD = 45.5;//45.5; //22.0; // 0;   //22;          //20.0;//25;//55.5; //48.0
//////
// PID gains for Pitch control//// (//Y-axis)
const float DEFAULT_PITCH_KP = 40.5;//10;//45;//40.5;  //29.5; // 25;  //29.5;       //30.0;//32;//65.0; //53.5
const float DEFAULT_PITCH_KI = 0.055;//0.098; //0.055;//  0;  //0.055;      // 0.056;//0;//2.2; //1.65
const float DEFAULT_PITCH_KD = 42.5;//42.5; //24.5; //  0;  //24.5;       //25.0;//30;//55.5; //48.0

// PID integral limits
const float PID_INTEGRAL_LIMIT = 50.0;

// ====================================================================
// IMU/DMP CONFIGURATION
// ====================================================================
// I2C communication settings
const unsigned long I2C_CLOCK_SPEED = 400000;     // 400kHz I2C clock

// IMU data validation parameters
const int MAX_CONSECUTIVE_ERRORS = 5;              // Maximum consecutive read errors before reinit
const float MAX_ANGLE_CHANGE = 20.0;              // Maximum allowed angle change per reading (degrees)
const float MAX_ANGLE_LIMIT = 45.0;               // Maximum allowed tilt angle (degrees)

// Calibration parameters
const int MIN_CALIBRATION_READINGS = 80;           // Minimum readings for calibration
const int MAX_CALIBRATION_ATTEMPTS = 200;         // Maximum attempts for calibration

// ====================================================================
// SYSTEM TIMING CONFIGURATION
// ====================================================================
const unsigned long BALANCE_UPDATE_INTERVAL = 50;    // Balance control loop interval (ms) - 20Hz
const unsigned long PRINT_STATUS_INTERVAL = 200;     // Status print interval (ms) - 5Hz
const unsigned long CALIBRATION_READING_DELAY = 20;  // Delay between calibration readings (ms)

// Homing and positioning timeouts
const unsigned long HOMING_TIMEOUT = 30000;          // Homing timeout (ms)
const unsigned long CENTER_MOVE_TIMEOUT = 30000;     // Center movement timeout (ms)

// ====================================================================
// HOMING AND POSITIONING CONFIGURATION
// ====================================================================
// Center position (revolutions from home)
const float CENTER_POSITION_REV = 10.5;
const long CENTER_POSITION_PULSES = (long)(CENTER_POSITION_REV * PULSES_PER_REV);

// Homing speeds
const int HOMING_SPEED = 110;                      // Homing movement speed (PWM)

// Center movement speeds
const int CENTER_MIN_SPEED = 80;                   // Minimum effective speed for centering
const int CENTER_MAX_SPEED = 180;                  // Maximum safe speed for centering
const int CENTER_POSITION_TOLERANCE = 50;         // Position tolerance for center (pulses)
const int CENTER_SPEED_SCALING = 25;              // Speed scaling factor for centering

// ====================================================================
// FREERTOS TASK CONFIGURATION
// ====================================================================
const int BALANCE_TASK_STACK_SIZE = 4096;          // Stack size for balance task
const int PRINT_TASK_STACK_SIZE = 4096;           // Stack size for print task
const int BALANCE_TASK_PRIORITY = 2;              // Priority for balance task
const int PRINT_TASK_PRIORITY = 1;                // Priority for print task
const int BALANCE_TASK_CORE = 1;                  // CPU core for balance task
const int PRINT_TASK_CORE = 0;                    // CPU core for print task

// Task delays
const int BALANCE_TASK_DELAY = 10;                 // Balance task delay (ms)
const int PRINT_TASK_DELAY = 10;                 // Print task delay (ms)

// ====================================================================
// SERIAL COMMUNICATION
// ====================================================================
const unsigned long SERIAL_BAUD_RATE = 115200;    // Serial communication baud rate

const int degLimit = 50;

// ====================================================================
// SAFETY CONFIGURATION
// ====================================================================
// Safety check intervals and thresholds are defined in the timing section above
// Additional safety parameters can be added here as needed


// Global variables
extern float targetRoll;
extern float targetPitch;
extern float currentRoll;
extern float currentPitch;
extern int rollPWM;
extern int pitchPWM;
extern bool motorX_homed;
extern bool motorY_homed;

#endif // CONFIG_H