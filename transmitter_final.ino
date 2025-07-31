 /*
 * Gesture Controller ESP8266 with MPU6050
 * Reads hand gestures and sends motor commands to car ESP
 */

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <MPU6050.h>

// WiFi credentials
const char* ssid = "MPU_AP";
const char* password = "12345678";

// Network settings
WiFiUDP udp;
IPAddress carIP;
const int carPort = 8888;
const int localPort = 8889;

// MPU6050 instance
MPU6050 mpu;

// Calibration values
float accelXOffset = 0, accelYOffset = 0, accelZOffset = 0;
float gyroXOffset = 0, gyroYOffset = 0, gyroZOffset = 0;

// Filter variables
float filteredRoll = 0, filteredPitch = 0;
const float alpha = 0.8;

// Gesture detection parameters
const float GESTURE_THRESHOLD = 15.0; 
const float TURN_THRESHOLD = 20.0;
unsigned long lastCommandTime = 0;
const unsigned long COMMAND_INTERVAL = 50; // ms

// Inactivity timeout
unsigned long lastGestureTime = 0;
const unsigned long GESTURE_TIMEOUT = 2000; // ms

bool debugEnabled = true;

struct MotorCommand {
  int leftMotorPWM;
  int rightMotorPWM;
  bool enable;
};

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (debugEnabled) Serial.println("=== Gesture Controller Starting ===");

  initializeMPU6050();
  connectToWiFi();

  if (MDNS.begin("gesture-controller")) {
    if (debugEnabled) Serial.println("mDNS responder started as 'gesture-controller.local'");
  }

  udp.begin(localPort);
  if (debugEnabled) Serial.printf("UDP started on port %d\n", localPort);

  findCarESP();
  calibrateMPU6050();

  if (debugEnabled) {
    Serial.println("=== Gesture Controller Ready ===");
    Serial.println("Tilt forward/backward: Forward/Reverse");
    Serial.println("Tilt left/right: Turn left/right");
    Serial.println("Keep level: Stop");
  }
}

void loop() {
  // Reconnect if WiFi drops
  if (WiFi.status() != WL_CONNECTED) {
    connectToWiFi();
  }

  static unsigned long lastLoop = 0;
  if (millis() - lastLoop >= 20) {
    lastLoop = millis();

    MotorCommand command = readGestureAndCalculateMotors();

    // Inactivity timeout
    if (abs(command.leftMotorPWM) > 0 || abs(command.rightMotorPWM) > 0) {
      lastGestureTime = millis();
    } else if (millis() - lastGestureTime > GESTURE_TIMEOUT) {
      command.leftMotorPWM = 0;
      command.rightMotorPWM = 0;
    }

    if (millis() - lastCommandTime > COMMAND_INTERVAL) {
      sendMotorCommand(command);
      lastCommandTime = millis();
    }

    printStatus(command);
  }
}

// -----------------------------
// MPU6050 Setup and Calibration
// -----------------------------

void initializeMPU6050() {
  if (debugEnabled) Serial.println("Initializing MPU6050...");

  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }

  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);

  if (debugEnabled) Serial.println("MPU6050 connected");
}

void calibrateMPU6050() {
  if (debugEnabled) Serial.println("Calibrating MPU6050... Keep device flat and still!");
  delay(2000);

  long accelX = 0, accelY = 0, accelZ = 0;
  long gyroX = 0, gyroY = 0, gyroZ = 0;

  const int samples = 1000;
  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    accelX += ax;
    accelY += ay;
    accelZ += az;
    gyroX += gx;
    gyroY += gy;
    gyroZ += gz;

    delay(2);
  }

  accelXOffset = accelX / samples;
  accelYOffset = accelY / samples;
  accelZOffset = (accelZ / samples) - 16384;
  gyroXOffset = gyroX / samples;
  gyroYOffset = gyroY / samples;
  gyroZOffset = gyroZ / samples;

  if (debugEnabled) Serial.println("Calibration complete!");
}

// -----------------------------
// WiFi + mDNS Connection
// -----------------------------

void connectToWiFi() {
  WiFi.begin(ssid, password);
  if (debugEnabled) Serial.print("Connecting to WiFi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    if (debugEnabled) Serial.print(".");
  }

  if (debugEnabled) {
    Serial.println();
    Serial.print("Connected! IP address: ");
    Serial.println(WiFi.localIP());
  }
}

void findCarESP() {
  if (debugEnabled) Serial.println("Searching for car ESP using mDNS...");

  int n = MDNS.queryService("car-control", "udp");

  if (n == 0) {
    if (debugEnabled) Serial.println("No car ESP found. Trying manual discovery...");
    broadcastDiscovery();
  } else {
    carIP = MDNS.IP(0);
    if (debugEnabled) {
      Serial.printf("Found %d car ESP(s)\n", n);
      Serial.print("Car ESP IP: ");
      Serial.println(carIP);
    }
  }
}

void broadcastDiscovery() {
  if (debugEnabled) Serial.println("Broadcasting discovery message...");

  for (int attempts = 0; attempts < 10; attempts++) {
    udp.beginPacket(IPAddress(255, 255, 255, 255), carPort);
    udp.write("GESTURE_CONTROLLER_DISCOVERY");
    udp.endPacket();
    delay(1000);

    int packetSize = udp.parsePacket();
    if (packetSize) {
      carIP = udp.remoteIP();
      if (debugEnabled) {
        Serial.print("Car ESP found at: ");
        Serial.println(carIP);
      }
      return;
    }
  }

  if (debugEnabled) Serial.println("Car ESP not found. Using broadcast.");
  carIP = IPAddress(255, 255, 255, 255);
}

// -----------------------------
// Gesture Detection Logic
// -----------------------------

MotorCommand readGestureAndCalculateMotors() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float accelX = ax - accelXOffset;
  float accelY = ay - accelYOffset;
  float accelZ = az - accelZOffset;

  float roll = atan2(accelY, accelZ) * 180.0 / PI;
  float pitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / PI;

  filteredRoll = alpha * filteredRoll + (1 - alpha) * roll;
  filteredPitch = alpha * filteredPitch + (1 - alpha) * pitch;

  // Gyro: convert raw gyro values to deg/sec using scale factor 131
  float pitchSpeed = (gy - gyroYOffset) / 131.0;
  float rollSpeed  = (gx - gyroXOffset) / 131.0;

  // Mix tilt angle and angular velocity for more dynamic control
  float mixedPitch = filteredPitch + pitchSpeed * 0.1;
  float mixedRoll  = filteredRoll + rollSpeed * 0.1;

  MotorCommand command;
  command.enable = true;

  int forwardSpeed = 0;
  if (mixedPitch > GESTURE_THRESHOLD) {
    forwardSpeed = map(constrain(mixedPitch, GESTURE_THRESHOLD, 70),
                       GESTURE_THRESHOLD, 70, 100, 255);
  } else if (mixedPitch < -GESTURE_THRESHOLD) {
    forwardSpeed = map(constrain(mixedPitch, -70, -GESTURE_THRESHOLD),
                       -70, -GESTURE_THRESHOLD, -255, -100);
  }

  int turnSpeed = 0;
  if (mixedRoll > TURN_THRESHOLD) {
    turnSpeed = map(constrain(mixedRoll, TURN_THRESHOLD, 70),
                    TURN_THRESHOLD, 70, 50, 150);
  } else if (mixedRoll < -TURN_THRESHOLD) {
    turnSpeed = map(constrain(mixedRoll, -70, -TURN_THRESHOLD),
                    -70, -TURN_THRESHOLD, -150, -50);
  }
  
command.leftMotorPWM = forwardSpeed + turnSpeed;
command.rightMotorPWM = forwardSpeed - turnSpeed;


  command.leftMotorPWM = constrain(command.leftMotorPWM, -255, 255);
  command.rightMotorPWM = constrain(command.rightMotorPWM, -255, 255);

  if (abs(command.leftMotorPWM) < 50) command.leftMotorPWM = 0;
  if (abs(command.rightMotorPWM) < 50) command.rightMotorPWM = 0;

  return command;
}

// -----------------------------
// Communication
// -----------------------------

void sendMotorCommand(MotorCommand command) {
  String packet = "L" + String(command.leftMotorPWM) + 
                  ",R" + String(command.rightMotorPWM) + 
                  ",E" + String(command.enable ? 1 : 0);

  udp.beginPacket(carIP, carPort);
  udp.write(packet.c_str());
  udp.endPacket();
}

void printStatus(MotorCommand command) {
  static unsigned long lastPrint = 0;
  if (debugEnabled && millis() - lastPrint > 500) {
    Serial.printf("Roll: %6.1f° | Pitch: %6.1f° | L: %4d | R: %4d\n", 
                  filteredRoll, filteredPitch, 
                  command.leftMotorPWM, command.rightMotorPWM);
    lastPrint = millis();
  }
}
