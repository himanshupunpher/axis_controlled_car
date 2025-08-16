/*
 * Final Car Receiver for Gesture Controller
 * ESP8266 + L298N Motor Driver
 * Connects to gesture controller's WiFi AP
 * Version: Final Production Ready
 */

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>

// ===========================================
// SETTINGS
// ===========================================
const char* ssid = "MPU_AP";
const char* password = "12345678";

// UDP setup
WiFiUDP udp;
const int localPort = 8888;
char incomingPacket[128];

// ===========================================
// MOTOR PINS - L298N Driver
// ===========================================
const int IN1 = D2;  // Left motor direction 1
const int IN2 = D3;  // Left motor direction 2
const int IN3 = D4;  // Right motor direction 1
const int IN4 = D5;  // Right motor direction 2
const int ENB = D6;  // Right motor PWM
const int ENA = D1;  // Left motor PWM

// ===========================================
// MOTOR SETTINGS
// ===========================================
const int DEAD_ZONE = 30;
const int PWM_LIMIT = 900;
const unsigned long SAFETY_TIMEOUT = 3000; // Stop after 3 seconds of no commands

// ===========================================
// VARIABLES
// ===========================================
bool debugEnabled = true;
unsigned long lastPacketTime = 0;
bool motorsEnabled = false;

// ===========================================
// MAIN PROGRAM
// ===========================================

void setup() {
  Serial.begin(115200);
  delay(500);
  
  Serial.println("=== CAR RECEIVER STARTING ===");
  
  // Initialize motor pins
  setupMotorPins();
  
  // Connect to gesture controller's WiFi
  connectToWiFi();
  
  // Start UDP communication
  startUDP();
  
  // Start mDNS service
  startMDNS();
  
  // Test motors briefly
  testMotors();
  
  Serial.println("✅ CAR RECEIVER READY!");
  Serial.println("Waiting for gesture commands...");
  Serial.println("===============================");
}

void loop() {
  // Check for incoming UDP packets
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(incomingPacket, sizeof(incomingPacket) - 1);
    if (len > 0) {
      incomingPacket[len] = '\0';
      handlePacket(String(incomingPacket));
      lastPacketTime = millis();
    }
  }
  
  // Safety timeout - stop motors if no commands received
  if (millis() - lastPacketTime > SAFETY_TIMEOUT && motorsEnabled) {
    if (debugEnabled) Serial.println("⚠️  Safety timeout - stopping motors");
    stopMotors();
    motorsEnabled = false;
  }
  
  // Check WiFi connection
  static unsigned long lastWiFiCheck = 0;
  if (millis() - lastWiFiCheck > 10000) { // Check every 10 seconds
    lastWiFiCheck = millis();
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi lost, reconnecting...");
      connectToWiFi();
    }
  }
}

// ===========================================
// MOTOR SETUP & TESTING
// ===========================================

void setupMotorPins() {
  Serial.println("🔧 Initializing motor pins...");
  
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  
  // Start with motors stopped
  stopMotors();
  
  Serial.println("   Motor pins configured");
}

void testMotors() {
  if (!debugEnabled) return;
  
  Serial.println("🧪 Testing motors (2 seconds)...");
  
  // Test both motors forward briefly
  controlL298Motor(ENA, IN1, IN2, 150);   // Left motor forward
  controlL298Motor(ENB, IN3, IN4, 150);   // Right motor forward
  delay(1000);
  
  // Test both motors backward briefly
  controlL298Motor(ENA, IN1, IN2, -150);  // Left motor backward
  controlL298Motor(ENB, IN3, IN4, -150);  // Right motor backward
  delay(1000);
  
  // Stop motors
  stopMotors();
  Serial.println("   Motor test complete");
}

// ===========================================
// NETWORK FUNCTIONS
// ===========================================

void connectToWiFi() {
  WiFi.begin(ssid, password);
  Serial.printf("🔌 Connecting to '%s'", ssid);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(" ❌ Failed!");
    Serial.println("Check if gesture controller is powered on and creating WiFi AP");
    return;
  }
  
  Serial.println(" ✅ Connected!");
  Serial.printf("   IP Address: %s\n", WiFi.localIP().toString().c_str());
  Serial.printf("   Gateway: %s\n", WiFi.gatewayIP().toString().c_str());
}

void startUDP() {
  udp.begin(localPort);
  Serial.printf("📡 UDP listening on port %d\n", localPort);
}

void startMDNS() {
  if (MDNS.begin("car-esp")) {
    Serial.println("🔍 mDNS started as 'car-esp.local'");
    MDNS.addService("car-control", "udp", localPort);
  }
}

// ===========================================
// PACKET HANDLING
// ===========================================

void handlePacket(String packet) {
  if (debugEnabled) {
    Serial.print("📥 Received: ");
    Serial.println(packet);
  }
  
  // Parse packet format: "L100R-50E1"
  int lIndex = packet.indexOf('L');
  int rIndex = packet.indexOf('R');
  int eIndex = packet.indexOf('E');
  
  if (lIndex == -1 || rIndex == -1 || eIndex == -1) {
    if (debugEnabled) Serial.println("   ❌ Invalid packet format");
    return;
  }
  
  // Extract values
  int rawLeft = packet.substring(lIndex + 1, rIndex).toInt();
  int rawRight = packet.substring(rIndex + 1, eIndex).toInt();
  int enable = packet.substring(eIndex + 1).toInt();
  
  // Scale and constrain values
  int left = map(rawLeft, -255, 255, -1023, 1023);
  int right = map(rawRight, -255, 255, -1023, 1023);
  left = constrain(left, -PWM_LIMIT, PWM_LIMIT);
  right = constrain(right, -PWM_LIMIT, PWM_LIMIT);
  
  if (debugEnabled) {
    Serial.printf("   Parsed: L:%d R:%d E:%d → Scaled: L:%d R:%d\n", 
                  rawLeft, rawRight, enable, left, right);
  }
  
  // Control motors
  if (!enable) {
    stopMotors();
    motorsEnabled = false;
    return;
  }
  
  motorsEnabled = true;
  controlL298Motor(ENA, IN1, IN2, left);   // Left motor
  controlL298Motor(ENB, IN3, IN4, right);  // Right motor
  
  // Show motor status
  if (debugEnabled && (left != 0 || right != 0)) {
    String status = "   🚗 ";
    if (left > 0 && right > 0) status += "FORWARD";
    else if (left < 0 && right < 0) status += "BACKWARD"; 
    else if (left > right) status += "TURN LEFT";
    else if (right > left) status += "TURN RIGHT";
    else status += "MOVING";
    
    Serial.println(status);
  }
}

// ===========================================
// MOTOR CONTROL
// ===========================================

void controlL298Motor(int pwmPin, int dirPin1, int dirPin2, int value) {
  // Handle dead zone
  if (abs(value) < DEAD_ZONE) {
    digitalWrite(dirPin1, LOW);
    digitalWrite(dirPin2, LOW);
    analogWrite(pwmPin, 0);
    return;
  }
  
  // Set direction
  if (value >= 0) {
    digitalWrite(dirPin1, HIGH);
    digitalWrite(dirPin2, LOW);
  } else {
    digitalWrite(dirPin1, LOW);
    digitalWrite(dirPin2, HIGH);
  }
  
  // Set speed
  analogWrite(pwmPin, abs(value));
}

void stopMotors() {
  digitalWrite(IN1, LOW); 
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); 
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  
  if (debugEnabled && motorsEnabled) {
    Serial.println("   🛑 Motors stopped");
  }
}