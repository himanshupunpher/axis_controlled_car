/*
 * Final Gesture Controller ESP8266 with MPU6050
 * Creates WiFi Access Point and controls car using hand gestures
 * Version: Final Production Ready
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
IPAddress carAddress = IPAddress(192, 168, 4, 2); // Expected car IP when it connects

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
unsigned long lastConnectionCheck = 0;

// Smoothing filter
float smoothRoll = 0;
float smoothPitch = 0;
const float SMOOTH_FACTOR = 0.8;

bool debugMode = true;
bool carConnected = false;

// ===========================================
// MAIN PROGRAM
// ===========================================

void setup() {
  Serial.begin(115200);
  delay(500);
  
  Serial.println("=== GESTURE CONTROLLER STARTING ===");
  
  // Step 1: Initialize sensor
  if (!setupSensor()) {
    Serial.println("Sensor failed - check wiring!");
    while(1) delay(1000);
  }
  
  // Step 2: Start Access Point
  if (!startAccessPoint()) {
    Serial.println("Access Point failed!");
    while(1) delay(1000);
  }
  
  // Step 3: Setup network services
  setupNetwork();
  
  // Step 4: Calibrate sensor
  calibrateSensor();
  
  Serial.println("SYSTEM READY!");
  Serial.println("=== GESTURE CONTROLS ===");
  Serial.println("   Tilt Forward/Back: Forward/Reverse");
  Serial.println("   Tilt Left/Right: Turn Left/Right");
  Serial.println("   Keep Level: Stop");
  Serial.println("   Waiting for car to connect...");
  Serial.println("================================");
}

void loop() {
  // Check car connection status
  checkCarConnection();
  
  // Read gestures every 20ms
  static unsigned long lastLoop = 0;
  if (millis() - lastLoop >= 20) {
    lastLoop = millis();
    
    // Get current gesture
    readGesture();
    
    // Calculate motor speeds
    calculateMotorSpeeds();
    
    // Handle auto-stop
    handleTimeout();
    
    // Send command to car (only if connected)
    if (millis() - lastCommandTime >= SEND_DELAY && carConnected) {
      sendCommand();
      lastCommandTime = millis();
    }
    
    // Print status
    printStatus();
  }
}

// ===========================================
// ACCESS POINT FUNCTIONS
// ===========================================

bool startAccessPoint() {
  Serial.printf("🏠 Starting Access Point '%s'... ", WIFI_NAME);
  
  // Set to Access Point mode
  WiFi.mode(WIFI_AP);
  
  // Start the access point
  bool result = WiFi.softAP(WIFI_NAME, WIFI_PASSWORD);
  
  if (!result) {
    Serial.println("Failed!");
    return false;
  }
  
  IPAddress apIP = WiFi.softAPIP();
  Serial.println("Success!");
  Serial.printf("   Network: %s\n", WIFI_NAME);
  Serial.printf("   Password: %s\n", WIFI_PASSWORD);
  Serial.printf("   Controller IP: %s\n", apIP.toString().c_str());
  Serial.printf("   Expected Car IP: %s\n", carAddress.toString().c_str());
  
  delay(2000); // Give AP time to stabilize
  return true;
}

void checkCarConnection() {
  if (millis() - lastConnectionCheck > 3000) { // Check every 3 seconds
    lastConnectionCheck = millis();
    
    int numConnected = WiFi.softAPgetStationNum();
    
    if (numConnected > 0 && !carConnected) {
      carConnected = true;
      Serial.printf("🚗 CAR CONNECTED! (%d device(s) on network)\n", numConnected);
      Serial.println("✅ Ready to receive gestures!");
    } else if (numConnected == 0 && carConnected) {
      carConnected = false;
      Serial.println("CAR DISCONNECTED!");
      // Stop motors when car disconnects
      motors.left = 0;
      motors.right = 0;
    }
  }
}

// ===========================================
// SENSOR FUNCTIONS
// ===========================================

bool setupSensor() {
  Serial.print("Setting up MPU6050... ");
  
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
  delay(3000);
  
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
  
  Serial.println(" Calibration Complete!");
}

void readGesture() {
  int16_t ax, ay, az, gx, gy, gz;
  
  sensor.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Basic sanity check
  if (ax == 0 && ay == 0 && az == 0 && gx == 0 && gy == 0 && gz == 0) {
    gesture.isValid = false;
    return;
  }
  
  // Remove calibration offsets
  float accelX = ax - offsets.accelX;
  float accelY = ay - offsets.accelY;
  float accelZ = az - offsets.accelZ;
  
  // Calculate angles from accelerometer
  float roll = atan2(accelY, accelZ) * 180.0 / PI;
  float pitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / PI;
  
  // Smooth the readings for stability
  smoothRoll = SMOOTH_FACTOR * smoothRoll + (1 - SMOOTH_FACTOR) * roll;
  smoothPitch = SMOOTH_FACTOR * smoothPitch + (1 - SMOOTH_FACTOR) * pitch;
  
  // Get gyro rates
  float gyroRoll = (gx - offsets.gyroX) / 131.0;
  float gyroPitch = (gy - offsets.gyroY) / 131.0;
  
  // Use accelerometer as primary, gyro for quick adjustments
  gesture.roll = smoothRoll;
  gesture.pitch = smoothPitch;
  
  // Add gyro if significant and in same direction
  if (abs(gyroRoll) > 5.0 && ((gyroRoll > 0 && smoothRoll > 0) || (gyroRoll < 0 && smoothRoll < 0))) {
    gesture.roll += gyroRoll * 0.05;
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
  if (!gesture.isValid || !carConnected) {
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
  
  // Calculate turn speed
  int turnSpeed = 0;
  if (abs(gesture.roll) > TURN_THRESHOLD) {
    if (gesture.roll > TURN_THRESHOLD) {
      // Tilt right = turn right (slow down left motor, speed up right motor)
      turnSpeed = map(constrain(gesture.roll, TURN_THRESHOLD, MAX_ANGLE),
                     TURN_THRESHOLD, MAX_ANGLE, 40, 120);
    }
    else if (gesture.roll < -TURN_THRESHOLD) {
      // Tilt left = turn left (slow down right motor, speed up left motor)
      turnSpeed = map(constrain(gesture.roll, -MAX_ANGLE, -TURN_THRESHOLD),
                     -MAX_ANGLE, -TURN_THRESHOLD, -120, -40);
    }
  }
  
  // FIXED: Combine speeds (differential drive)
  // For left turn: left motor slower, right motor faster
  // For right turn: left motor faster, right motor slower
  motors.left = forwardSpeed - turnSpeed;   // Changed: subtract turnSpeed for left
  motors.right = forwardSpeed + turnSpeed;  // Changed: add turnSpeed for right
  
  // Apply limits
  motors.left = constrain(motors.left, -MAX_SPEED, MAX_SPEED);
  motors.right = constrain(motors.right, -MAX_SPEED, MAX_SPEED);
  
  // Remove small values that cause jitter
  if (abs(motors.left) < DEAD_ZONE && abs(motors.right) < DEAD_ZONE) {
    motors.left = 0;
    motors.right = 0;
  } else {
    // Allow smaller values for pure turning
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

void setupNetwork() {
  // Start UDP
  udp.begin(LOCAL_PORT);
  Serial.printf("📡 UDP started on port %d\n", LOCAL_PORT);
  
  // Start mDNS for service discovery
  if (MDNS.begin("gesture-controller")) {
    Serial.println("🔍 mDNS started as 'gesture-controller.local'");
    MDNS.addService("gesture-control", "udp", LOCAL_PORT);
  }
}

// ===========================================
// COMMUNICATION
// ===========================================

void sendCommand() {
  // Use the format your car expects: L100R-50E1
  String command = "L" + String(motors.left) + 
                  "R" + String(motors.right) + 
                  "E1";
  
  udp.beginPacket(carAddress, CAR_PORT);
  udp.write(command.c_str());
  udp.endPacket();
  
  // Debug output (comment out for production)
  if (motors.left != 0 || motors.right != 0) {
    Serial.print("📤 SENT: ");
    Serial.println(command);
  }
}

// ===========================================
// STATUS & DEBUG
// ===========================================

void printStatus() {
  if (!debugMode) return;
  
  if (millis() - lastDebugTime > DEBUG_DELAY) {
    lastDebugTime = millis();
    
    // Show connection status first
    Serial.printf("[%s] ", carConnected ? "CONNECTED" : "WAITING   ");
    
    // Show gesture data
    Serial.printf("Roll:%5.1f° Pitch:%5.1f° → L:%4d R:%4d", 
                  gesture.roll, gesture.pitch, motors.left, motors.right);
    
    // Add gesture direction indicators
    if (abs(gesture.roll) > TURN_THRESHOLD) {
      if (gesture.roll > 0) Serial.print(" [→ RIGHT]");
      else Serial.print(" [← LEFT]");
    }
    if (abs(gesture.pitch) > FORWARD_THRESHOLD) {
      if (gesture.pitch > 0) Serial.print(" [↑ FORWARD]");
      else Serial.print(" [↓ BACKWARD]");
    }
    
    // Show status
    if (!carConnected) {
      Serial.print(" [NO CAR]");
    } else if (motors.left == 0 && motors.right == 0) {
      Serial.print(" [IDLE]");
    } else {
      Serial.print(" [ACTIVE]");
    }
    
    Serial.println();
  }
}
