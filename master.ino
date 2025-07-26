// Enhanced 2-Axis Balancing Platform - Single Core Implementation
// ESP32-S3-N16R8 Implementation - All tasks on Core 1
// With Ramp and Sinusoidal Test Functions

#include "config.h"
#include "imu_handler.h"
#include "motor_control.h"
#include "encoders.h"
#include "homing.h"
#include "balance_control.h"
#include "safety.h"

// Global system state variables
bool systemReady = false;
bool balancingActive = false;
bool homingInProgress = true;

// Target angles (level platform)
float targetRoll = 0.0;
float targetPitch = 0.0;

// Timing variables
unsigned long lastUpdate = 0;
unsigned long lastPrint = 0;
unsigned long lastStatusPrint = 0;

// Task handles for FreeRTOS
TaskHandle_t mainTaskHandle = NULL;

// Safety status
uint8_t safetyStatus = 0;
float angle;

// Test mode variables
typedef enum {
  TEST_MODE_NONE,
  TEST_MODE_STEP,
  TEST_MODE_RAMP,
  TEST_MODE_SINE
} TestMode;

TestMode currentTestMode = TEST_MODE_NONE;
unsigned long testDelay = 0;
unsigned long testStartTime = 0;
float elapsedTime = 0;
float testDuration = 10.0; // Default test duration in seconds
float stepTarget = 0;
float rampRate = 1.0; // degrees per second
float sineAmplitude = 5.0; // degrees
float sineFrequency = 0.1; // Hz (0.1 Hz = 10 second period)
int DELAY = 2000;

// Test parameters
float initialRoll = 0.0;
float initialPitch = 0.0;

void setup() {
  Serial.begin(115200);
  
  // Initialize I2C with custom pins
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  
  Serial.println(F("Enhanced Balancing Platform Initializing on ESP32-S3 (Single Core)..."));
  
  // Setup hardware components
  setupMotors();
  setupEncoders();
  
  // Initialize MPU6050 DMP
  if (initIMU()) {
    Serial.println(F("CMP Ready! Starting homing process..."));
    startHoming();
  } else {
    Serial.println(F("CMP Initialization Failed!"));
    return;
  }
  
  // Create single main task on Core 1
  createMainTask();
  
  Serial.println(F("Single core system initialization complete"));
}

void loop() {
  // Main loop is now empty - everything runs in the main task
  // This prevents any conflicts between loop() and tasks
  vTaskDelete(NULL); // Delete the default loop task
}

// Single main task running on Core 1 - handles everything
void mainTask(void *parameter) {
  unsigned long lastBalanceUpdate = 0;
  unsigned long lastStatusUpdate = 0;
  unsigned long lastCommandCheck = 0;
  unsigned long lastSafetyCheck = 0;
  unsigned long lastTestUpdate = 0; // Add this for test updates
  
  const unsigned long BALANCE_INTERVAL = 5;  // 20Hz balance control
  const unsigned long STATUS_INTERVAL = 200; // 5Hz status printing
  const unsigned long COMMAND_INTERVAL = 10;  // 100Hz command checking
  const unsigned long SAFETY_INTERVAL = 10;   // 40Hz safety checks
  const unsigned long TEST_INTERVAL = 20;     // 50Hz test updates
  
  for (;;) {
    unsigned long currentTime = millis();
    
    // Handle homing process (highest priority)
    if (homingInProgress) {
      processHoming();
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    
    // Wait for system to be ready
    if (!systemReady) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }
    
    // Update test targets if test is running
    if (currentTime - lastTestUpdate >= TEST_INTERVAL) {
      updateTestTargets();
      lastTestUpdate = currentTime;
    }
    
    // Handle serial commands (high frequency for responsiveness)
    if (currentTime - lastCommandCheck >= COMMAND_INTERVAL) {
      handleCommands();
      lastCommandCheck = currentTime;
    }
    
    // Balance control loop
    if (currentTime - lastBalanceUpdate >= BALANCE_INTERVAL) {
      if (readIMUData()) {
        if (balancingActive) {
          calculateBalanceControl();
          executeMotorControl();
        }
      }
      lastBalanceUpdate = currentTime;
    }
    
    // Safety checks
    safetyStatus = safetyCheck();
    
    // Status printing (lowest priority)
    if (currentTime - lastStatusUpdate >= STATUS_INTERVAL) {
      if (systemReady && !homingInProgress) {
        printStatusWithTest();
      }
      lastStatusUpdate = currentTime;
    }
    
    // Small delay to prevent watchdog issues and allow other system tasks
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void createMainTask() {
  // Create single main task on Core 1
  xTaskCreatePinnedToCore(
    mainTask,          // Task function
    "MainTask",        // Task name
    8192,              // Increased stack size for all functionality
    NULL,              // Parameters
    2,                 // Priority
    &mainTaskHandle,   // Task handle
    1                  // Core 1 (Core 0 handles WiFi/Bluetooth)
  );
  
  Serial.println(F("Single core main task created successfully"));
}

// Test Functions
void startRampTest(float duration = 10.0, float rate = 1.0) {
  currentTestMode = TEST_MODE_RAMP; 
  testDuration = duration;
  rampRate = rate;
  
  // Store initial positions
  initialRoll = currentRoll;
  initialPitch = currentPitch;
  
  Serial.print(">>> RAMP TEST STARTED - Duration: ");
  Serial.print(duration);
  Serial.print("s, Rate: ");
  Serial.print(rate);
  Serial.println("°/s <<<");
  Serial.println(">>> Waiting 2 seconds before starting test... <<<");

  // Set delay start time and actual test start time
  testDelay = millis();
  testStartTime = millis() + DELAY; // Test starts 2 seconds from now
}

void startSineTest(float duration = 20.0, float amplitude = 5.0, float frequency = 0.1) {
  currentTestMode = TEST_MODE_SINE;
  testDuration = duration;
  sineAmplitude = amplitude;
  sineFrequency = frequency;
  
  // Store initial positions
  initialRoll = currentRoll;
  initialPitch = currentPitch;
  
  Serial.print(">>> SINE TEST STARTED - Duration: ");
  Serial.print(duration);
  Serial.print("s, Amplitude: ");
  Serial.print(amplitude);
  Serial.print("°, Frequency: ");
  Serial.print(frequency);
  Serial.println("Hz <<<");
  Serial.println(">>> Waiting 2 seconds before starting test... <<<");

  // Set delay start time and actual test start time
  testDelay = millis();
  testStartTime = millis() + DELAY; // Test starts 2 seconds from now
}

void updateTestTargets() {
  // If no test is running, do nothing
  if (currentTestMode == TEST_MODE_NONE) {
    return;
  }
  
  unsigned long currentTime = millis();
  
  // Check if we're still in the 2-second delay period
  if (currentTime < testStartTime) {
    // During delay period, keep targets at initial values
    targetRoll = initialRoll;
    targetPitch = initialPitch;
    return;
  }
  
  // Calculate elapsed time since test actually started (after delay)
  elapsedTime = (currentTime - testStartTime) / 1000.0; // Convert to seconds
  
  // Check if test duration has elapsed
  if (elapsedTime >= testDuration) {
    stopTest();
    resetTargets();
    return;
  }
  
  switch (currentTestMode) {
    case TEST_MODE_STEP:
      targetRoll = initialRoll + stepTarget;
      targetPitch = initialPitch + stepTarget;
      break;
      
    case TEST_MODE_RAMP:
      // Linear ramp: target = initial + rate * time
      targetRoll = initialRoll + (rampRate * elapsedTime);
      targetPitch = initialPitch + (rampRate * elapsedTime);
      break;
      
    case TEST_MODE_SINE:
      // Sinusoidal: target = initial + amplitude * sin(2π * frequency * time)
      angle = 2.0 * PI * sineFrequency * elapsedTime;
      targetRoll = initialRoll + (sineAmplitude * sin(angle));
      targetPitch = initialPitch + (sineAmplitude * sin(angle));
      break;
  }
}

void testTarget(int step) {
  currentTestMode = TEST_MODE_STEP;
  stepTarget = step;
  
  // Store initial positions
  initialRoll = currentRoll;
  initialPitch = currentPitch;
  
  Serial.print(">>> Step test +");
  Serial.print(step);
  Serial.println("° <<<");
  Serial.println(">>> Waiting 2 seconds before starting test... <<<");

  // Set delay start time and actual test start time
  testDelay = millis();
  testStartTime = millis() + DELAY; // Test starts 2 seconds from now
}

void stopTest() {
  if (currentTestMode != TEST_MODE_NONE) {
    Serial.println(">>> TEST COMPLETED <<<");
    currentTestMode = TEST_MODE_NONE;
    
    // Reset targets to current position to avoid sudden jumps
    targetRoll = currentRoll;
    targetPitch = currentPitch;
  }
}


void handleCommands() {
  // Handle serial commands
  if (Serial.available()) {
    char cmd = Serial.read();
    
    switch (cmd) {
      case 's':
      case 'S':
        if (!balancingActive && systemReady) {
          startBalancing();
          Serial.println(">>> Balancing STARTED <<<");
        } else if (!systemReady) {
          Serial.println(">>> System NOT READY <<<");
        }
        break;
        
      case 't':
      case 'T':
        if (balancingActive) {
          stopBalancing();
          Serial.println(">>> Balancing STOPPED <<<");
        }
        break;
        
      case 'c':
      case 'C':
        if (systemReady) {
          Serial.println(">>> Calibrating... <<<");
          calibrateOffsets();
        } else {
          Serial.println(">>> System NOT READY <<<");
        }
        break;
        
      case 'h':
      case 'H':
        if (!balancingActive && systemReady) {
          Serial.println(">>> Rehoming system... <<<");
          systemReady = false;
          homingInProgress = true;
          startHoming();
        } else if (balancingActive) {
          Serial.println(">>> Stop balancing first <<<");
        }
        break;
        
      case 'r':
      case 'R':
        resetTargets();
        break;
        
      case 'p': // Step test +5° (existing)
      case 'P':
        testTarget(5);
        break;

      case 'q': // Step test +10° (existing)
      case 'Q':
        testTarget(10);
        break;
        
      // NEW TEST COMMANDS
      case '0': // Start ramp test with default parameters
        startRampTest(10, 1);
        break;
        
      case '1': // Start ramp test with custom rate (2°/s)
        startRampTest(5.0, 2.0);
        break;

      case '2': // Start ramp test with custom rate (2°/s)
        startRampTest(4.0, 3.0);
        break;
        
      case '3': // Start sine test with default parameters 0.6 hz
        startSineTest(60, 4.0, 0.6);
        break;
        
      case '4': // Start sine test with higher frequency (0.4 Hz)
        startSineTest(60.0, 6.0, 0.4);
        break;
        
      case '5': // Start sine test with larger amplitude (5°)
        startSineTest(60.0, 8.0, 0.2);
        break;
        
      case 'x': // Stop current test
      case 'X':
        stopTest();
        break;
        
      case '?':
        printHelp();
        break;
        
      default:
        // Clear any remaining characters silently
        while (Serial.available()) {
          Serial.read();
        }
        break;
    }
  }
}

void resetTargets() {
  stopTest(); // Stop any running test first
  targetRoll = 0.0;
  targetPitch = 0.0;
  Serial.println(F("Target angles reset to 0°"));
}

void printHelp() {
  Serial.println(F("\n=== BALANCING PLATFORM COMMANDS ==="));
  Serial.println(F("s/S - Start balancing"));
  Serial.println(F("t/T - Stop balancing"));
  Serial.println(F("c/C - Calibrate IMU offsets"));
  Serial.println(F("h/H - Home all motors"));
  Serial.println(F("r/R - Reset target angles to 0°"));
  Serial.println(F("p/P - Step test +5°"));
  Serial.println(F("q/Q - Step test +10°"));
  Serial.println(F(""));
  Serial.println(F("=== TEST INPUTS ==="));
  Serial.println(F("1   - Ramp test (10s, 1°/s)"));
  Serial.println(F("2   - Ramp test (10s, 2°/s)"));
  Serial.println(F("3   - Sine test (20s, 3°, 0.1Hz)"));
  Serial.println(F("4   - Sine test (20s, 5°, 0.2Hz)"));
  Serial.println(F("5   - Sine test (20s, 8°, 0.1Hz)"));
  Serial.println(F("x/X - Stop current test"));
  Serial.println(F("?   - Show this help"));
  Serial.println(F("====================================\n"));
}

// Enhanced status printing with test information
void printStatusWithTest() {
  // Get current values
  float roll = currentRoll;
  float pitch = currentPitch;
  long posX = getEncoderX();
  long posY = getEncoderY();
  
  // Convert positions to mm
  float posXmm = (float)posX / PULSES_PER_REV * 2.0;
  float posYmm = (float)posY / PULSES_PER_REV * 2.0;
  
  // Print basic status
  Serial.print("Roll:");
  Serial.print(roll, 1);
  Serial.print("° Pitch:");
  Serial.print(pitch, 1);
  Serial.print("° | Target R:");
  Serial.print(targetRoll, 1);
  Serial.print("° P:");
  Serial.print(targetPitch, 1);
  Serial.print("° | X:");
  Serial.print(posXmm, 1);
  Serial.print("mm Y:");
  Serial.print(posYmm, 1);
  Serial.print("mm");
  
  // Add PWM values if balancing is active
  if (balancingActive) {
    Serial.print(" | PWM X:");
    Serial.print((int)rollPWM);
    Serial.print(" Y:");
    Serial.print((int)pitchPWM);
    Serial.print(" | ACTIVE");
  } else {
    Serial.print(" | STOPPED");
  }
  
  // Add test mode information
  // if (currentTestMode != TEST_MODE_NONE) {
  //   elapsedTime = (millis() - testStartTime) / 1000.0;
  //   Serial.print(" | TEST:");
  //   if (currentTestMode == TEST_MODE_RAMP) {
  //     Serial.print("RAMP ");
  //   } else if (currentTestMode == TEST_MODE_SINE) {
  //     Serial.print("SINE ");
  //   }
  //   Serial.print(elapsedTime, 1);
  //   Serial.print("s");
  // }
  
  // Add safety status if there's an issue
  if (safetyStatus == SAFETY_LIMIT_PRESSED) {
    Serial.print(" | LIMIT");
  } else if (safetyStatus == SAFETY_OUT_OF_RANGE) {
    Serial.print(" | RANGE");
  }
  
  Serial.println();
}

// Fallback to simple print if needed
void printStatusSimple() {
  printStatusWithTest();
}

// System state functions
void setSystemReady(bool ready) {
  systemReady = ready;
  if (ready) {
    Serial.println(F("System Ready!"));
    Serial.println(F("Commands: 's'=start, 't'=stop, 'c'=calibrate, 'h'=home, '?'=help"));
  }
}

void setHomingInProgress(bool homing) {
  homingInProgress = homing;
}

void setBalancingActive(bool active) {
  balancingActive = active;
}

// Getter functions for other modules
bool isSystemReady() {
  return systemReady;
}

bool isBalancingActive() {
  return balancingActive;
}

bool isHomingInProgress() {
  return homingInProgress;
}

float getTargetRoll() {
  return targetRoll;
}

float getTargetPitch() {
  return targetPitch;
}

void setCurrentRoll(float roll) {
  currentRoll = roll;
}

void setCurrentPitch(float pitch) {
  currentPitch = pitch;
}

unsigned long getLastUpdate() {
  return lastUpdate;
}