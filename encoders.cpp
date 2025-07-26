#include "encoders.h"

// Encoder count variables
volatile long encoderX_count = 0;
volatile long encoderY_count = 0;

// Mutex for encoder access
portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;

// Static variables for quadrature decoding
static int lastStateA_X = 0;
static int lastStateB_X = 0;
static int lastStateA_Y = 0;
static int lastStateB_Y = 0;

void setupEncoders() {
  // Configure encoder pins as inputs with internal pullup
  pinMode(ENCODER_X_A, INPUT_PULLUP);
  pinMode(ENCODER_X_B, INPUT_PULLUP);
  pinMode(ENCODER_Y_A, INPUT_PULLUP);
  pinMode(ENCODER_Y_B, INPUT_PULLUP);
  
  // Initialize static variables with current pin states
  lastStateA_X = digitalRead(ENCODER_X_A);
  lastStateB_X = digitalRead(ENCODER_X_B);
  lastStateA_Y = digitalRead(ENCODER_Y_A);
  lastStateB_Y = digitalRead(ENCODER_Y_B);
  
  // Attach interrupts - ESP32 supports interrupts on all GPIO pins
  // Using CHANGE to detect both rising and falling edges for better resolution
  attachInterrupt(digitalPinToInterrupt(ENCODER_X_A), encoderX_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_Y_A), encoderY_ISR, CHANGE);
  
  Serial.println(F("Encoders initialized"));
  Serial.print(F("Encoder X pins: A="));
  Serial.print(ENCODER_X_A);
  Serial.print(F(", B="));
  Serial.println(ENCODER_X_B);
  Serial.print(F("Encoder Y pins: A="));
  Serial.print(ENCODER_Y_A);
  Serial.print(F(", B="));
  Serial.println(ENCODER_Y_B);
}

// OPTIMIZED ENCODER ISR FUNCTION FOR X-AXIS
void IRAM_ATTR encoderX_ISR() {
  portENTER_CRITICAL_ISR(&encoderMux);
  
  int stateA = digitalRead(ENCODER_X_A);
  int stateB = digitalRead(ENCODER_X_B);
  
  // Improved quadrature decoding with edge detection
  if (stateA != lastStateA_X) {
    if (stateA == HIGH) {
      // Rising edge on A
      if (stateB == LOW) {
        encoderX_count++;  // Forward (clockwise)
      } else {
        encoderX_count--;  // Reverse (counter-clockwise)
      }
    } else {
      // Falling edge on A
      if (stateB == HIGH) {
        encoderX_count++;  // Forward (clockwise)
      } else {
        encoderX_count--;  // Reverse (counter-clockwise)
      }
    }
    lastStateA_X = stateA;
  }
  
  // Update state B for potential future use
  lastStateB_X = stateB;
  
  portEXIT_CRITICAL_ISR(&encoderMux);
}

// OPTIMIZED ENCODER ISR FUNCTION FOR Y-AXIS
void IRAM_ATTR encoderY_ISR() {
  portENTER_CRITICAL_ISR(&encoderMux);
  
  int stateA = digitalRead(ENCODER_Y_A);
  int stateB = digitalRead(ENCODER_Y_B);
  
  // Improved quadrature decoding with edge detection
  if (stateA != lastStateA_Y) {
    if (stateA == HIGH) {
      // Rising edge on A
      if (stateB == LOW) {
        encoderY_count++;  // Forward (clockwise)
      } else {
        encoderY_count--;  // Reverse (counter-clockwise)
      }
    } else {
      // Falling edge on A
      if (stateB == HIGH) {
        encoderY_count++;  // Forward (clockwise)
      } else {
        encoderY_count--;  // Reverse (counter-clockwise)
      }
    }
    lastStateA_Y = stateA;
  }
  
  // Update state B for potential future use
  lastStateB_Y = stateB;
  
  portEXIT_CRITICAL_ISR(&encoderMux);
}

// Get current encoder X count (thread-safe)
long getEncoderX() {
  portENTER_CRITICAL(&encoderMux);
  long count = encoderX_count;
  portEXIT_CRITICAL(&encoderMux);
  return count;
}

// Get current encoder Y count (thread-safe)
long getEncoderY() {
  portENTER_CRITICAL(&encoderMux);
  long count = encoderY_count;
  portEXIT_CRITICAL(&encoderMux);
  return count;
}

// Reset encoder X count to zero (thread-safe)
void resetEncoderX() {
  portENTER_CRITICAL(&encoderMux);
  encoderX_count = 0;
  portEXIT_CRITICAL(&encoderMux);
  Serial.println(F("Encoder X reset"));
}

// Reset encoder Y count to zero (thread-safe)
void resetEncoderY() {
  portENTER_CRITICAL(&encoderMux);
  encoderY_count = 0;
  portEXIT_CRITICAL(&encoderMux);
  Serial.println(F("Encoder Y reset"));
}

// Get encoder X position in revolutions
float getEncoderXRevolutions() {
  return (float)getEncoderX() / PULSES_PER_REV;
}

// Get encoder Y position in revolutions
float getEncoderYRevolutions() {
  return (float)getEncoderY() / PULSES_PER_REV;
}

// Check if encoder X is within safe operating range
bool isEncoderXInRange() {
  long count = getEncoderX();
  return (count >= 0 && count <= MAX_POSITION_PULSES);
}

// Check if encoder Y is within safe operating range
bool isEncoderYInRange() {
  long count = getEncoderY();
  return (count >= 0 && count <= MAX_POSITION_PULSES);
}