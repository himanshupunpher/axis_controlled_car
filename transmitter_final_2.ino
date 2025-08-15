/*
 * Simple & Clean Gesture Controller for ESP8266
 * Controls a car using hand gestures via MPU6050 sensor
 */

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <MPU6050.h>

// ===========================================
// SETTINGS - Easy to modify
// ===========================================
const char* CAR_NAME = "Reddy";
const char* WIFI_NAME = "MPU_AP";
const char* WIFI_PASSWORD = "12345678";

const int CAR_PORT = 8888;
const int LOCAL_PORT = 8889;

// Gesture sensitivity (degrees)
const float FORWARD_THRESHOLD = 15.0;
const float TURN_THRESHOLD = 10.0;    // Lowered for easier turning
const float MAX_ANGLE = 70.0;

// Motor settings
const int MIN_SPEED = 100;
const int MAX_SPEED = 255;
const int DEAD_ZONE = 50;

// Timing (milliseconds)
const int SEND_DELAY = 50;      // How often to send commands
const int STOP_TIMEOUT = 2000;  // Auto-stop after no gesture
const int DEBUG_DELAY = 500;    // Debug print frequency

// ===========================================
// HARDWARE & NETWORK
// ===========================================
MPU6050 sensor;
WiFiUDP udp;
IPAddress carAddress;

// ===========================================
// SENSOR DATA
// ===========================================
struct SensorOffsets {
  float accelX = 0, accelY = 0, accelZ = 0;
  float gyroX = 0, gyroY = 0, gyroZ = 0;
} offsets;

struct GestureData {
  float roll = 0;      // Left/Right tilt
  float pitch = 0;     // Forward/Back tilt
  bool isValid = false;
} gesture;

struct MotorSpeeds {
  int left = 0;
  int right = 0;
} motors;

// ===========================================
// TIMING VARIABLES
// ===========================================
unsigned long lastCommandTime = 0;
unsigned long lastGestureTime = 0;
unsigned long lastDebugTime = 0;

// Smoothing filter
float smoothRoll = 0;
float smoothPitch = 0;
const float SMOOTH_FACTOR = 0.8;

bool debugMode = true;

// ===========================================
// MAIN PROGRAM
// ===========================================

void setup() {
  Serial.begin(115200);
  delay(500);
  
  // Step 1: Initialize sensor
  if (!setupSensor()) {
    Serial.println("❌ Sensor failed - check wiring!");
    while(1) delay(1000);
  }
  
  // Step 2: Connect to WiFi
  if (!connectWiFi()) {
    Serial.println("❌ WiFi failed - check credentials!");
    while(1) delay(1000);
  }
  
  // Step 3: Find the car
  setupNetwork();
  findCar();
  
  // Step 4: Calibrate sensor
  calibrateSensor();
}

void loop() {
  // Check WiFi connection
  checkConnections();
  
  // Read gestures every 20ms
  if (millis() - lastCommandTime >= 20) {
    
    // Get current gesture
    readGesture();
    
    // Calculate motor speeds
    calculateMotorSpeeds();
    
    // Handle auto-stop
    handleTimeout();
    
    // Send command to car
    if (millis() - lastCommandTime >= SEND_DELAY) {
      sendCommand();
      lastCommandTime = millis();
    }
    
    // Print status
    printStatus();
  }
}

// ===========================================
// SENSOR FUNCTIONS
// ===========================================

bool setupSensor() {
  Serial.print("🔧 Setting up MPU6050... ");
  
  Wire.begin();
  sensor.initialize();
  
  if (!sensor.testConnection()) {
    Serial.println("Failed!");
    return false;
  }
  
  // Configure for best gesture detection
  sensor.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
  sensor.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
  sensor.setDLPFMode(MPU6050_DLPF_BW_20); // Reduce noise
  
  Serial.println("OK!");
  return true;
}

void calibrateSensor() {
  Serial.println("Calibrating sensor...");
  Serial.println("   Keep device FLAT and STILL for 3 seconds!");
  delay(2000);
  
  // Collect samples
  long accelSum[3] = {0};
  long gyroSum[3] = {0};
  const int samples = 500;
  
  Serial.print("   Progress: ");
  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    sensor.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    accelSum[0] += ax;
    accelSum[1] += ay;
    accelSum[2] += az;
    gyroSum[0] += gx;
    gyroSum[1] += gy;
    gyroSum[2] += gz;
    
    if (i % 50 == 0) Serial.print("█");
    delay(5);
  }
  
  // Calculate offsets
  offsets.accelX = accelSum[0] / samples;
  offsets.accelY = accelSum[1] / samples;
  offsets.accelZ = accelSum[2] / samples - 16384; // Remove gravity
  offsets.gyroX = gyroSum[0] / samples;
  offsets.gyroY = gyroSum[1] / samples;
  offsets.gyroZ = gyroSum[2] / samples;
  
  Serial.println("Done!");
}

void readGesture() {
  int16_t ax, ay, az, gx, gy, gz;
  
  // getMotion6 returns void, so we just call it and assume it works
  sensor.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Basic sanity check - if all values are zero, sensor might be disconnected
  if (ax == 0 && ay == 0 && az == 0 && gx == 0 && gy == 0 && gz == 0) {
    gesture.isValid = false;
    return;
  }
  
  // Remove calibration offsets
  float accelX = ax - offsets.accelX;
  float accelY = ay - offsets.accelY;
  float accelZ = az - offsets.accelZ;
  
  // Calculate angles from accelerometer (gravity-based, works for slow tilts)
  float roll = atan2(accelY, accelZ) * 180.0 / PI;
  float pitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / PI;
  
  // Smooth the readings for stability
  smoothRoll = SMOOTH_FACTOR * smoothRoll + (1 - SMOOTH_FACTOR) * roll;
  smoothPitch = SMOOTH_FACTOR * smoothPitch + (1 - SMOOTH_FACTOR) * pitch;
  
  // Get gyro rates (for fast movements)
  float gyroRoll = (gx - offsets.gyroX) / 131.0;
  float gyroPitch = (gy - offsets.gyroY) / 131.0;
  
  // Use accelerometer as primary source, gyro only for quick adjustments
  // This ensures slow tilts are properly detected
  gesture.roll = smoothRoll;
  gesture.pitch = smoothPitch;
  
  // Only add gyro if it's significant AND in same direction as tilt
  if (abs(gyroRoll) > 5.0 && ((gyroRoll > 0 && smoothRoll > 0) || (gyroRoll < 0 && smoothRoll < 0))) {
    gesture.roll += gyroRoll * 0.05; // Much smaller gyro influence
  }
  
  if (abs(gyroPitch) > 5.0 && ((gyroPitch > 0 && smoothPitch > 0) || (gyroPitch < 0 && smoothPitch < 0))) {
    gesture.pitch += gyroPitch * 0.05;
  }
  
  gesture.isValid = true;
}

// ===========================================
// MOTOR CONTROL
// ===========================================

void calculateMotorSpeeds() {
  if (!gesture.isValid) {
    motors.left = 0;
    motors.right = 0;
    return;
  }
  
  // Calculate forward/backward speed
  int forwardSpeed = 0;
  if (gesture.pitch > FORWARD_THRESHOLD) {
    // Tilt forward = go forward
    forwardSpeed = map(constrain(gesture.pitch, FORWARD_THRESHOLD, MAX_ANGLE),
                      FORWARD_THRESHOLD, MAX_ANGLE, MIN_SPEED, MAX_SPEED);
  } 
  else if (gesture.pitch < -FORWARD_THRESHOLD) {
    // Tilt backward = go backward
    forwardSpeed = map(constrain(gesture.pitch, -MAX_ANGLE, -FORWARD_THRESHOLD),
                      -MAX_ANGLE, -FORWARD_THRESHOLD, -MAX_SPEED, -MIN_SPEED);
  }
  
  // Calculate turn speed - more sensitive and responsive
  int turnSpeed = 0;
  if (abs(gesture.roll) > TURN_THRESHOLD) {
    if (gesture.roll > TURN_THRESHOLD) {
      // Tilt right = turn right
      turnSpeed = map(constrain(gesture.roll, TURN_THRESHOLD, MAX_ANGLE),
                     TURN_THRESHOLD, MAX_ANGLE, 40, 120);
    }
    else if (gesture.roll < -TURN_THRESHOLD) {
      // Tilt left = turn left  
      turnSpeed = map(constrain(gesture.roll, -MAX_ANGLE, -TURN_THRESHOLD),
                     -MAX_ANGLE, -TURN_THRESHOLD, -120, -40);
    }
  }
  
  // Combine speeds (differential drive)
  motors.left = forwardSpeed + turnSpeed;
  motors.right = forwardSpeed - turnSpeed;
  
  // Apply limits
  motors.left = constrain(motors.left, -MAX_SPEED, MAX_SPEED);
  motors.right = constrain(motors.right, -MAX_SPEED, MAX_SPEED);
  
  // Remove small values that cause jitter - but allow pure turning
  if (abs(motors.left) < DEAD_ZONE && abs(motors.right) < DEAD_ZONE) {
    motors.left = 0;
    motors.right = 0;
  } else {
    // Allow smaller values for pure turning (when only one motor is active)
    if (abs(motors.left) < 30) motors.left = 0;
    if (abs(motors.right) < 30) motors.right = 0;
  }
}

void handleTimeout() {
  // Track when we last had active gesture
  if (motors.left != 0 || motors.right != 0) {
    lastGestureTime = millis();
  }
  
  // Auto-stop after timeout
  if (millis() - lastGestureTime > STOP_TIMEOUT) {
    motors.left = 0;
    motors.right = 0;
  }
}

// ===========================================
// NETWORK FUNCTIONS
// ===========================================

bool connectWiFi() {
  Serial.printf("Connecting to %s... ", WIFI_NAME);
  
  WiFi.begin(WIFI_NAME, WIFI_PASSWORD);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(" Failed!");
    return false;
  }
  
  Serial.println("Connected!");
  Serial.printf("   IP Address: %s\n", WiFi.localIP().toString().c_str());
  return true;
}

void setupNetwork() {
  // Start UDP
  udp.begin(LOCAL_PORT);
  
  // Start mDNS for auto-discovery
  if (MDNS.begin("gesture-controller")) {
    Serial.println("mDNS started");
  }
}

void findCar() {
  Serial.print("Looking for car... ");
  
  // Try mDNS first
  int services = MDNS.queryService("car-control", "udp");
  if (services > 0) {
    carAddress = MDNS.IP(0);
    Serial.printf("Found via mDNS: %s\n", carAddress.toString().c_str());
    return;
  }
  
  // Try broadcast discovery
  Serial.println(CAR_NAME);
  for (int i = 0; i < 5; i++) {
    udp.beginPacket(IPAddress(255, 255, 255, 255), CAR_PORT);
    udp.write("HELLO_CAR");
    udp.endPacket();
    
    delay(500);
    
    if (udp.parsePacket() > 0) {
      carAddress = udp.remoteIP();
      Serial.printf("Found: %s\n", carAddress.toString().c_str());
      return;
    }
  }
  
  // Default to broadcast
  carAddress = IPAddress(255, 255, 255, 255);
  Serial.println("Using broadcast mode");
}

void checkConnections() {
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck > 5000) {
    lastCheck = millis();
    
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi lost, reconnecting...");
      connectWiFi();
    }
  }
}

// ===========================================
// COMMUNICATION
// ===========================================

void sendCommand() {
  String command = "L" + String(motors.left) + 
                  ",R" + String(motors.right) + ",E1";
  
  udp.beginPacket(carAddress, CAR_PORT);
  udp.write(command.c_str());
  udp.endPacket();
}

// ===========================================
// STATUS & DEBUG
// ===========================================

void printStatus() {
  if (!debugMode) return;
  
  if (millis() - lastDebugTime > DEBUG_DELAY) {
    lastDebugTime = millis();
    
    // Show more detailed info for debugging turns
    Serial.printf("Roll:%5.1f° Pitch:%5.1f° → L:%4d R:%4d", 
                  gesture.roll, gesture.pitch, motors.left, motors.right);
    
    // Add turn direction indicator
    if (abs(gesture.roll) > TURN_THRESHOLD) {
      if (gesture.roll > 0) Serial.print(" [TURN RIGHT]");
      else Serial.print(" [TURN LEFT]");
    }
    if (abs(gesture.pitch) > FORWARD_THRESHOLD) {
      if (gesture.pitch > 0) Serial.print(" [FORWARD]");
      else Serial.print(" [BACKWARD]");
    }
    Serial.println();
  }
}
