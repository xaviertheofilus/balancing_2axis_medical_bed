#include "imu_handler.h"

// Variabel untuk data sensor
int16_t accelX, accelY, accelZ;
int16_t gyroX, gyroY, gyroZ;

// Variabel untuk perhitungan sudut
float accelAngleX, accelAngleY;
float gyroAngleX, gyroAngleY;

// Primary outputs for the system
float currentRoll = 0.0;
float currentPitch = 0.0;

// Calibration - store the "level" reference angles
float levelReferenceRoll = 2.64;
float levelReferencePitch = 0.7;
bool isCalibrated = false;

// Complementary filter
float compFilterRoll = 0.0;
float compFilterPitch = 0.0;
unsigned long lastFilterTime = 0;

// Gyro offsets
float gyroXOffset = 0;
float gyroYOffset = 0;
float gyroZOffset = 0;

// Accelerometer offsets
float accelXOffset = 0;
float accelYOffset = 0;
float accelZOffset = 0;

// Moving average filter
float rollBuffer[FILTER_SIZE] = {0};
float pitchBuffer[FILTER_SIZE] = {0};
int bufferIndex = 0;

// Motion detection
bool isInFastMotion = false;
unsigned long lastMotionTime = 0;

// Error handling
struct ErrorState {
  int consecutiveErrors = 0;
  int i2cErrorCount = 0;
  unsigned long lastI2CError = 0;
  unsigned long lastSuccessfulRead = 0;
  bool i2cHealthy = true;
  bool sensorReady = false;
};
ErrorState errorState;

// Outlier detection
float lastValidRoll = 0.0;
float lastValidPitch = 0.0;
float velocityRoll = 0.0;
float velocityPitch = 0.0;
unsigned long lastValidTime = 0;

// Timing
unsigned long prevTime = 0;
float dt;

// PERBAIKAN: Tambahan untuk stabilitas kalibrasi
float rawRollHistory[20] = {0};
float rawPitchHistory[20] = {0};
int historyIndex = 0;
bool historyFull = false;

// Safe I2C communication with retry logic
bool safeI2CRead(uint8_t address, uint8_t regAddress, uint8_t* data, uint8_t length) {
  for (int attempt = 0; attempt < MAX_I2C_RETRIES; attempt++) {
    Wire.beginTransmission(address);
    Wire.write(regAddress);
    
    if (Wire.endTransmission(false) == 0) {
      if (Wire.requestFrom(address, length) == length) {
        for (int i = 0; i < length; i++) {
          data[i] = Wire.read();
        }
        return true;
      }
    }
    delayMicroseconds(100);
  }
  return false;
}

bool safeI2CWrite(uint8_t address, uint8_t regAddress, uint8_t data) {
  for (int attempt = 0; attempt < MAX_I2C_RETRIES; attempt++) {
    Wire.beginTransmission(address);
    Wire.write(regAddress);
    Wire.write(data);
    
    if (Wire.endTransmission() == 0) {
      return true;
    }
    delayMicroseconds(100);
  }
  return false;
}

bool initIMU() {
  Serial.println(F("Initializing Enhanced MPU6050..."));
  
  // Conservative I2C initialization
  Wire.end();
  delay(150);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);  // 100kHz for stability
  Wire.setTimeout(100);
  delay(300);
  
  // Test connection with retries
  bool connected = false;
  for (int attempt = 0; attempt < 5; attempt++) {
    Serial.print(F("Connection attempt: ")); Serial.println(attempt + 1);
    
    uint8_t testData;
    if (safeI2CRead(MPU6050_ADDR, 0x75, &testData, 1)) { // WHO_AM_I register
      if (testData == 0x68) {
        connected = true;
        Serial.println(F("MPU6050 connected successfully"));
        break;
      }
    }
    delay(200);
  }
  
  if (!connected) {
    Serial.println(F("MPU6050 connection failed"));
    errorState.sensorReady = false;
    return false;
  }
  
  // Wake up MPU6050
  if (!safeI2CWrite(MPU6050_ADDR, PWR_MGMT_1, 0x00)) {
    Serial.println(F("Failed to wake up MPU6050"));
    return false;
  }
  delay(100);
  
  // Configure accelerometer range (±2g)
  if (!safeI2CWrite(MPU6050_ADDR, ACCEL_CONFIG, 0x00)) {
    Serial.println(F("Failed to configure accelerometer"));
    return false;
  }
  
  // Configure gyroscope range (±250°/s)
  if (!safeI2CWrite(MPU6050_ADDR, GYRO_CONFIG, 0x00)) {
    Serial.println(F("Failed to configure gyroscope"));
    return false;
  }
  
  // Set sample rate divider (1kHz / (1 + 7) = 125Hz)
  if (!safeI2CWrite(MPU6050_ADDR, SMPLRT_DIV, 0x07)) {
    Serial.println(F("Failed to set sample rate"));
    return false;
  }
  
  // Configure Digital Low Pass Filter
  if (!safeI2CWrite(MPU6050_ADDR, CONFIG, 0x06)) {
    Serial.println(F("Failed to configure DLPF"));
    return false;
  }
  
  // Reset error state
  errorState.consecutiveErrors = 0;
  errorState.i2cErrorCount = 0;
  errorState.i2cHealthy = true;
  errorState.sensorReady = true;
  errorState.lastSuccessfulRead = millis();
  
  prevTime = millis();
  lastFilterTime = millis();
  lastValidTime = millis();
  
  Serial.println(F("Enhanced MPU6050 initialization complete"));
  return true;
}

bool readRawMPU6050() {
  uint8_t data[14];
  
  if (!safeI2CRead(MPU6050_ADDR, ACCEL_XOUT_H, data, 14)) {
    handleI2CError();
    return false;
  }
  
  // Parse accelerometer data
  accelX = (data[0] << 8) | data[1];
  accelY = (data[2] << 8) | data[3];
  accelZ = (data[4] << 8) | data[5];
  
  // Skip temperature data (data[6], data[7])
  
  // Parse gyroscope data
  gyroX = (data[8] << 8) | data[9];
  gyroY = (data[10] << 8) | data[11];
  gyroZ = (data[12] << 8) | data[13];
  
  // Apply offsets
  accelX -= accelXOffset;
  accelY -= accelYOffset;
  accelZ -= accelZOffset;
  
  gyroX -= gyroXOffset;
  gyroY -= gyroYOffset;
  gyroZ -= gyroZOffset;
  
  return true;
}

// PERBAIKAN: Fungsi untuk mengecek stabilitas dengan threshold yang realistis
bool checkSensorStability() {
  if (!historyFull) return true; // Assume stable if not enough history
  
  float rollSum = 0, pitchSum = 0;
  float rollMin = rawRollHistory[0], rollMax = rawRollHistory[0];
  float pitchMin = rawPitchHistory[0], pitchMax = rawPitchHistory[0];
  
  for (int i = 0; i < 20; i++) {
    rollSum += rawRollHistory[i];
    pitchSum += rawPitchHistory[i];
    
    if (rawRollHistory[i] < rollMin) rollMin = rawRollHistory[i];
    if (rawRollHistory[i] > rollMax) rollMax = rawRollHistory[i];
    if (rawPitchHistory[i] < pitchMin) pitchMin = rawPitchHistory[i];
    if (rawPitchHistory[i] > pitchMax) pitchMax = rawPitchHistory[i];
  }
  
  float rollRange = rollMax - rollMin;
  float pitchRange = pitchMax - pitchMin;
  
  // PERBAIKAN: Threshold yang lebih realistis (2.0 derajat instead of 0.5)
  return (rollRange < 2.0 && pitchRange < 2.0);
}
// PERBAIKAN: Fungsi untuk menghitung sudut dengan orientasi yang tepat
void calculateAnglesImproved() {
  unsigned long currentTime = millis();
  dt = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;
  
  // Validate timing
  if (dt > 0.1 || dt <= 0) {
    dt = 0.01; // Default to 10ms if timing is invalid
  }
  
  // PERBAIKAN: Perhitungan sudut accelerometer yang lebih akurat
  // Konversi ke g-force
  float ax = accelX / 16384.0;
  float ay = accelY / 16384.0;
  float az = accelZ / 16384.0;
  
  // Validasi magnitude accelerometer
  float accelMagnitude = sqrt(ax*ax + ay*ay + az*az);
  
  // PERBAIKAN: Formula sudut yang konsisten dengan orientasi sensor
  // Roll = rotasi terhadap sumbu X (forward/backward)
  // Pitch = rotasi terhadap sumbu Y (left/right)
  accelAngleX = atan2(ay, sqrt(ax*ax + az*az)) * 180.0 / PI;  // Roll
  accelAngleY = atan2(-ax, sqrt(ay*ay + az*az)) * 180.0 / PI; // Pitch
  
  // Convert gyroscope to degrees per second
  float gyroXrate = gyroX / 131.0;
  float gyroYrate = gyroY / 131.0;
  
  // Apply deadband to reduce noise
  if (abs(gyroXrate) < GYRO_DEADBAND) gyroXrate = 0.0;
  if (abs(gyroYrate) < GYRO_DEADBAND) gyroYrate = 0.0;
  
  // Integrate gyroscope
  gyroAngleX += gyroXrate * dt;
  gyroAngleY += gyroYrate * dt;
  
  // Complementary Filter with adaptive alpha based on accelerometer validity
  float adaptiveAlpha = ALPHA;
  if (accelMagnitude < 0.8 || accelMagnitude > 1.2) {
    adaptiveAlpha = 0.99; // Trust gyro more when accelerometer is unreliable
  }
  
  // PERBAIKAN: Complementary filter yang lebih stabil
  float rawRoll = adaptiveAlpha * (currentRoll + gyroXrate * dt) + (1 - adaptiveAlpha) * accelAngleX;
  float rawPitch = adaptiveAlpha * (currentPitch + gyroYrate * dt) + (1 - adaptiveAlpha) * accelAngleY;
  
  // PERBAIKAN: Simpan history untuk analisis stabilitas
  rawRollHistory[historyIndex] = rawRoll;
  rawPitchHistory[historyIndex] = rawPitch;
  historyIndex = (historyIndex + 1) % 20;
  if (historyIndex == 0) historyFull = true;
  
  // Motion detection
  unsigned long currentTimeMs = millis();
  if (currentTimeMs - lastValidTime > 5 && currentTimeMs - lastValidTime < 200) {
    float rollDiff = abs(rawRoll - lastValidRoll);
    float pitchDiff = abs(rawPitch - lastValidPitch);
    
    if (rollDiff > 8.0 || pitchDiff > 8.0) {
      isInFastMotion = true;
      lastMotionTime = currentTimeMs;
    } else if (currentTimeMs - lastMotionTime > MOTION_TIMEOUT) {
      isInFastMotion = false;
    }
    
    // Outlier rejection
    if (rollDiff > 25.0 || pitchDiff > 25.0) {
      return; // Skip this reading
    }
    
    // Update velocity estimation
    float timeDelta = (currentTimeMs - lastValidTime) / 1000.0;
    velocityRoll = rollDiff / timeDelta;
    velocityPitch = pitchDiff / timeDelta;
    
    lastValidTime = currentTimeMs;
  }
  
  // PERBAIKAN: Terapkan moving average SEBELUM kalibrasi
  addToMovingAverage(rawRoll, rawPitch);
  
  // Apply calibration SETELAH filtering
  if (isCalibrated) {
    currentRoll = currentRoll - levelReferenceRoll;
    currentPitch = currentPitch - levelReferencePitch;
  }
  
  // Update references
  lastValidRoll = rawRoll;
  lastValidPitch = rawPitch;
}

void calculateAngles() {
  calculateAnglesImproved();
}

void addToMovingAverage(float roll, float pitch) {
  // Apply noise threshold
  if (abs(roll) < NOISE_THRESHOLD) roll = 0.0;
  if (abs(pitch) < NOISE_THRESHOLD) pitch = 0.0;
  
  rollBuffer[bufferIndex] = roll;
  pitchBuffer[bufferIndex] = pitch;
  bufferIndex = (bufferIndex + 1) % FILTER_SIZE;
  
  // Calculate moving average
  float rollSum = 0, pitchSum = 0;
  for (int i = 0; i < FILTER_SIZE; i++) {
    rollSum += rollBuffer[i];
    pitchSum += pitchBuffer[i];
  }
  
  currentRoll = rollSum / FILTER_SIZE;
  currentPitch = pitchSum / FILTER_SIZE;
  
  // Apply constraints
  currentRoll = constrain(currentRoll, -45.0, 45.0);
  currentPitch = constrain(currentPitch, -45.0, 45.0);
}

bool readIMUData() {
  if (!errorState.sensorReady || !errorState.i2cHealthy) {
    return false;
  }
  
  bool success = readRawMPU6050();
  
  if (success) {
    calculateAngles();
    
    // Success path
    errorState.consecutiveErrors = 0;
    errorState.i2cErrorCount = 0;
    errorState.i2cHealthy = true;
    errorState.lastSuccessfulRead = millis();
    return true;
  } else {
    // Error handling
    errorState.consecutiveErrors++;
    
    if (errorState.consecutiveErrors < 8) {
      // Use last valid values with small decay for short failures
      currentRoll *= 0.99;
      currentPitch *= 0.99;
      addToMovingAverage(currentRoll, currentPitch);
    }
    
    // Trigger reinit if too many consecutive errors
    if (errorState.consecutiveErrors > 30 || errorState.i2cErrorCount > 8) {
      Serial.println(F("Attempting IMU recovery"));
      if (initIMU()) {
        errorState.consecutiveErrors = 0;
        errorState.i2cErrorCount = 0;
      } else {
        errorState.consecutiveErrors = 15; // Partial reset
      }
    }
    return false;
  }
}

void handleI2CError() {
  errorState.i2cErrorCount++;
  errorState.lastI2CError = millis();
  errorState.i2cHealthy = false;
  
  Serial.print(F("I2C Error #")); Serial.println(errorState.i2cErrorCount);
  
  // Progressive recovery strategy
  if (errorState.i2cErrorCount >= MAX_I2C_RETRIES && 
      millis() - errorState.lastI2CError > I2C_RECOVERY_INTERVAL) {
    
    Serial.println(F("Attempting I2C recovery..."));
    
    // Full I2C reset
    Wire.end();
    delay(100);
    
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(100000);
    Wire.setTimeout(100);
    delay(200);
    
    // Test connection
    uint8_t testData;
    bool recovered = safeI2CRead(MPU6050_ADDR, 0x75, &testData, 1);
    
    if (recovered && testData == 0x68) {
      // Reapply basic settings
      safeI2CWrite(MPU6050_ADDR, PWR_MGMT_1, 0x00);
      delay(50);
      safeI2CWrite(MPU6050_ADDR, ACCEL_CONFIG, 0x00);
      safeI2CWrite(MPU6050_ADDR, GYRO_CONFIG, 0x00);
      safeI2CWrite(MPU6050_ADDR, CONFIG, 0x06);
      
      errorState.i2cErrorCount = 0;
      errorState.i2cHealthy = true;
      Serial.println(F("I2C recovery successful"));
    } else {
      Serial.println(F("I2C recovery failed"));
      errorState.i2cErrorCount = MAX_I2C_RETRIES;
    }
  }
}

void calibrateOffsets() {
  Serial.println(F("=== STARTING ENHANCED CALIBRATION ==="));
  Serial.println(F("Keep IMU as still as possible for 15 seconds!"));
  Serial.println(F("Small vibrations are normal and will be filtered out."));
  delay(3000);
  
  if (!errorState.sensorReady || !errorState.i2cHealthy) {
    Serial.println(F("IMU not ready for calibration"));
    return;
  }
  
  // PERBAIKAN: Threshold yang lebih realistis untuk MPU6050
  Serial.println(F("Checking stability..."));
  float stabilityCheckDuration = 2000; // Kurangi jadi 2 detik
  unsigned long stabilityStart = millis();
  
  float maxGyroReading = 0;
  int motionWarnings = 0;
  
  while (millis() - stabilityStart < stabilityCheckDuration) {
    if (!readRawMPU6050()) {
      Serial.println(F("I2C error during stability check"));
      return;
    }
    
    float gxf = gyroX / 131.0;
    float gyf = gyroY / 131.0;
    float gzf = gyroZ / 131.0;
    
    // Track maximum gyro reading
    float maxCurrentReading = max(abs(gxf), max(abs(gyf), abs(gzf)));
    if (maxCurrentReading > maxGyroReading) {
      maxGyroReading = maxCurrentReading;
    }
    
    // PERBAIKAN: Threshold yang lebih realistis (3°/s instead of 1°/s)
    if (abs(gxf) > 3.5 || abs(gyf) > 3.5 || abs(gzf) > 3.5) {
      motionWarnings++;
      if (motionWarnings > 10) { // Hanya warn jika konsisten
        Serial.print(F("Significant motion detected: gx=")); Serial.print(gxf, 2);
        Serial.print(F(" gy=")); Serial.print(gyf, 2);
        Serial.print(F(" gz=")); Serial.println(gzf, 2);
        Serial.println(F("Try to minimize movement..."));
        
        // Reset counter dan timer
        motionWarnings = 0;
        stabilityStart = millis();
        delay(500);
      }
    }
    delay(20);
  }
  
  Serial.print(F("Stability check complete. Max gyro reading: "));
  Serial.print(maxGyroReading, 2);
  Serial.println(F(" °/s"));
  
  // PERBAIKAN: Lanjutkan kalibrasi meskipun ada noise kecil
  if (maxGyroReading < 5.0) {
    Serial.println(F("Sensor sufficiently stable for calibration"));
  } else {
    Serial.println(F("Warning: High motion detected, but proceeding with calibration"));
  }
  
  // PERBAIKAN: Kalibrasi offset sensor dengan outlier rejection
  long accelXSum = 0, accelYSum = 0, accelZSum = 0;
  long gyroXSum = 0, gyroYSum = 0, gyroZSum = 0;
  int validReadings = 0;
  
  // Arrays untuk outlier detection
  float gyroXReadings[100];
  float gyroYReadings[100];  
  float gyroZReadings[100];
  int readingIndex = 0;
  
  for (int i = 0; i < 1500; i++) { // Sedikit kurangi sample
    if (readRawMPU6050()) {
      // Reset offsets untuk raw calculation
      int16_t rawAccelX = accelX + accelXOffset;
      int16_t rawAccelY = accelY + accelYOffset;
      int16_t rawAccelZ = accelZ + accelZOffset;
      int16_t rawGyroX = gyroX + gyroXOffset;
      int16_t rawGyroY = gyroY + gyroYOffset;
      int16_t rawGyroZ = gyroZ + gyroZOffset;
      
      // Convert gyro to degrees/sec for outlier detection
      float gxf = rawGyroX / 131.0;
      float gyf = rawGyroY / 131.0;
      float gzf = rawGyroZ / 131.0;
      
      // PERBAIKAN: Outlier rejection - skip extreme readings
      if (abs(gxf) > 10.0 || abs(gyf) > 10.0 || abs(gzf) > 10.0) {
        continue; // Skip this reading
      }
      
      accelXSum += rawAccelX;
      accelYSum += rawAccelY;
      accelZSum += rawAccelZ;
      
      gyroXSum += rawGyroX;
      gyroYSum += rawGyroY;
      gyroZSum += rawGyroZ;
      
      // Store readings for noise analysis
      if (validReadings < 100) {
        gyroXReadings[validReadings] = gxf;
        gyroYReadings[validReadings] = gyf;
        gyroZReadings[validReadings] = gzf;
      }
      
      validReadings++;
    }
    
    if (i % 150 == 0) {
      Serial.print(F("Calibration progress: ")); Serial.print(i);
      Serial.print(F("/1500, Valid: ")); Serial.println(validReadings);
    }
    delay(3);
  }
  
  if (validReadings > 1000) { // Minimal 66% valid readings
    // Calculate offsets
    accelXOffset = accelXSum / validReadings;
    accelYOffset = accelYSum / validReadings;
    accelZOffset = (accelZSum / validReadings) - 16384; // Z should be 1g
    
    gyroXOffset = gyroXSum / validReadings;
    gyroYOffset = gyroYSum / validReadings;
    gyroZOffset = gyroZSum / validReadings;
    
    // PERBAIKAN: Analisis noise gyroscope
    if (validReadings >= 100) {
      float gyroXNoise = 0, gyroYNoise = 0, gyroZNoise = 0;
      for (int i = 0; i < 100; i++) {
        gyroXNoise += abs(gyroXReadings[i]);
        gyroYNoise += abs(gyroYReadings[i]);
        gyroZNoise += abs(gyroZReadings[i]);
      }
      gyroXNoise /= 100;
      gyroYNoise /= 100;
      gyroZNoise /= 100;
      
      Serial.println(F("Gyroscope noise analysis:"));
      Serial.print(F("Average noise - X: ")); Serial.print(gyroXNoise, 2);
      Serial.print(F("°/s Y: ")); Serial.print(gyroYNoise, 2);
      Serial.print(F("°/s Z: ")); Serial.print(gyroZNoise, 2);
      Serial.println(F("°/s"));
    }
    
    Serial.println(F("Sensor offsets calculated:"));
    Serial.print(F("Accel (X,Y,Z): ")); 
    Serial.print(accelXOffset); Serial.print(F(", "));
    Serial.print(accelYOffset); Serial.print(F(", "));
    Serial.println(accelZOffset);
    
    Serial.print(F("Gyro (X,Y,Z): "));
    Serial.print(gyroXOffset); Serial.print(F(", "));
    Serial.print(gyroYOffset); Serial.print(F(", "));
    Serial.println(gyroZOffset);
    
    // PERBAIKAN: Level reference calibration dengan stabilitas check yang lebih longgar
    Serial.println(F("Calibrating level reference..."));
    
    // Reset state untuk kalibrasi level
    resetIMUState();
    historyFull = false;
    historyIndex = 0;
    
    // Stabilisasi sensor dulu
    for (int i = 0; i < 50; i++) {
      if (readRawMPU6050()) {
        calculateAngles();
      }
      delay(20);
    }
    
    // PERBAIKAN: Langsung lakukan kalibrasi level tanpa stability check yang ketat
    Serial.println(F("Collecting level reference data..."));
    
    float rollSum = 0, pitchSum = 0;
    int levelReadings = 0;
    float rollReadings[300];
    float pitchReadings[300];
    
    for (int i = 0; i < 300; i++) {
      if (readRawMPU6050()) {
        calculateAngles();
        
        // Store readings for outlier detection
        rollReadings[levelReadings] = currentRoll;
        pitchReadings[levelReadings] = currentPitch;
        levelReadings++;
      }
      delay(10);
    }
    
    if (levelReadings >= 200) {
      // PERBAIKAN: Outlier rejection untuk level reference
      // Sort arrays to find median and remove outliers
      float rollMedian = 0, pitchMedian = 0;
      int validLevelReadings = 0;
      
      // Simple outlier rejection - remove extreme 10%
      int startIndex = levelReadings * 0.05;
      int endIndex = levelReadings * 0.95;
      
      for (int i = startIndex; i < endIndex; i++) {
        // Skip readings that are too extreme
        if (abs(rollReadings[i]) < 20.0 && abs(pitchReadings[i]) < 20.0) {
          rollSum += rollReadings[i];
          pitchSum += pitchReadings[i];
          validLevelReadings++;
        }
      }
      
      if (validLevelReadings >= 100) {
        levelReferenceRoll = rollSum / validLevelReadings;
        levelReferencePitch = pitchSum / validLevelReadings;
        isCalibrated = true;
        
        Serial.println(F("=== CALIBRATION COMPLETE ==="));
        Serial.print(F("Level reference - Roll: ")); Serial.print(levelReferenceRoll, 3);
        Serial.print(F("° Pitch: ")); Serial.print(levelReferencePitch, 3);
        Serial.println(F("°"));
        Serial.print(F("Used ")); Serial.print(validLevelReadings);
        Serial.println(F(" valid readings for level reference"));
        
        // Verifikasi kalibrasi
        Serial.println(F("Verifying calibration..."));
        float testRoll = 0, testPitch = 0;
        int testReadings = 0;
        
        for (int i = 0; i < 50; i++) {
          if (readIMUData()) {
            testRoll += currentRoll;
            testPitch += currentPitch;
            testReadings++;
          }
          delay(20);
        }
        
        if (testReadings > 30) {
          float avgRoll = testRoll / testReadings;
          float avgPitch = testPitch / testReadings;
          
          Serial.print(F("Calibrated output - Roll: ")); Serial.print(avgRoll, 3);
          Serial.print(F("° Pitch: ")); Serial.print(avgPitch, 3);
          Serial.println(F("°"));
          
          if (abs(avgRoll) < 3.0 && abs(avgPitch) < 3.0) {
            Serial.println(F("Calibration verification PASSED"));
          } else {
            Serial.println(F("Calibration verification WARNING - some offset remains"));
            Serial.println(F("This is normal for initial calibration"));
          }
        }
        
        resetIMUState();
      } else {
        Serial.println(F("Level calibration failed - too many outliers"));
        isCalibrated = false;
      }
    } else {
      Serial.println(F("Level calibration failed - insufficient valid readings"));
      isCalibrated = false;
    }
  } else {
    Serial.println(F("Sensor calibration failed - insufficient valid readings"));
  }
}

void resetIMUState() {
  errorState.consecutiveErrors = 0;
  errorState.i2cErrorCount = 0;
  errorState.i2cHealthy = true;
  errorState.lastSuccessfulRead = millis();
  
  currentRoll = 0.0;
  currentPitch = 0.0;
  lastValidRoll = 0.0;
  lastValidPitch = 0.0;
  velocityRoll = 0.0;
  velocityPitch = 0.0;
  isInFastMotion = false;
  
  for (int i = 0; i < FILTER_SIZE; i++) {
    rollBuffer[i] = 0;
    pitchBuffer[i] = 0;
  }
  bufferIndex = 0;
  
  // PERBAIKAN: Reset history buffer
  for (int i = 0; i < 20; i++) {
    rawRollHistory[i] = 0;
    rawPitchHistory[i] = 0;
  }
  historyIndex = 0;
  historyFull = false;
  
  prevTime = millis();
  lastFilterTime = millis();
  lastValidTime = millis();
  
  gyroAngleX = 0;
  gyroAngleY = 0;
}

// Getter functions
float getCurrentRoll() {
  return currentRoll;
}

float getCurrentPitch() {
  return currentPitch;
}

bool isIMUReady() {
  return errorState.sensorReady && errorState.i2cHealthy;
}

bool isIMUCalibrated() {
  return isCalibrated;
}

bool isI2CHealthy() {
  return errorState.i2cHealthy && 
         (millis() - errorState.lastSuccessfulRead < 3000) &&
         errorState.sensorReady;
}

bool isInFastMotionState() {
  return isInFastMotion;
}

float getCurrentRollVelocity() {
  return velocityRoll;
}

float getCurrentPitchVelocity() {
  return velocityPitch;
}